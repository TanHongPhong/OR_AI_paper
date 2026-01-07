#pragma once
// =============================================================================
//  grid_grow_alns.h  (header-only, C++17)
// =============================================================================
//  Grid-grow ALNS for the "cluster-trip last-mile" model (GRID-TENSOR edition).
//
//  Matches your current codebase:
//    - config.h: Instance / Solution / Route / CustomerTensor
//    - data_loader.h: loads grid tensor + tables
//    - initialization.h: seed-growing logic and helper policies
//    - objectives.h: evaluation using roads-only Distance_km
//
//  Core idea:
//    - Work on *occupied grid cells* (cell_linear) as atomic customers.
//    - Each trip (Route) is a set of member cells + a center cell.
//    - Repair uses the same "grow" spirit as initialization: ring-boundary scan
//      around the (possibly shifted) center to absorb nearby unassigned cells.
//    - Destroy operators remove boundary/regions, then (NEW) can re-center the
//      trip to a random cell (adaptive), so the regrow starts "crawling" from a
//      different seed and can expand in a better direction.
//
//  Notes:
//    - This is a pragmatic ALNS skeleton aimed to compile and integrate cleanly.
//    - For speed, operators do in-place changes and call Objectives::evaluate
//      only once per iteration (not per micro-move).
// =============================================================================

#include "config.h"
#include "initialization.h" // reuse spread_ring_limit + ring boundary iterator
#include "objectives.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <limits>
#include <numeric>
#include <ostream>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace gridopt {

// =============================================================================
// Parameters
// =============================================================================
struct ALNSParams {
    int iterations = 800;

    // --- destroy intensity (INCREASED for more aggressive optimization) ---
    // fraction of members to remove (was 15%..50%, now 25%..65%)
    double destroy_frac_min = 0.25;
    double destroy_frac_max = 0.65;

    // connected-region removal target size factor (was 20%..55%, now 30%..75%)
    double region_frac_min = 0.30;
    double region_frac_max = 0.75;

    // how many attempts of local improvement per iteration (was 20, now 40)
    int vnd_tries = 40;

    // --- acceptance ---
    double temp0 = 5.0e4;      // starting temperature
    double temp_end = 0.5;     // ending temperature

    // --- logging ---
    int log_interval = 100;  // print progress every N iterations
    bool verbose = true;     // enable verbose logging

    // Maximum number of occupied cells to add in ONE grow call.
    int max_add_per_step = 12;

    // --- adaptive re-center sampling ---
    int recenter_sample = 20;
    int recenter_probe_R = 25; 
    
    // probability to wipe a random trip to enable reseed (was 0.15, now 0.30)
    double wipe_trip_prob = 0.30;      
    int min_members_reseed = 2;        
    double cap_slack = 0.05; // 5% capacity slack for VND moves
    double ring_limit_mul = 1.0; 
};

// =============================================================================
// Internal helpers (cells)
// =============================================================================
namespace alns_detail {

inline void nbr8_linear(const CustomerTensor& T, int cell, int out[8], int& out_n) {
    out_n = 0;
    int r=0,c=0; T.decode_linear(cell, r, c);
    for (int dr=-1; dr<=1; ++dr) {
        for (int dc=-1; dc<=1; ++dc) {
            if (dr==0 && dc==0) continue;
            int rr = r + dr;
            int cc = c + dc;
            if (!T.in_bounds(rr,cc)) continue;
            out[out_n++] = T.linear(rr,cc);
        }
    }
}

inline double cell_w(const Instance& inst, int cell) {
    int r=0,c=0; inst.customers.decode_linear(cell, r, c);
    return (double)inst.customers.at(r,c,CUST_CH_W);
}
inline double cell_v(const Instance& inst, int cell) {
    int r=0,c=0; inst.customers.decode_linear(cell, r, c);
    return (double)inst.customers.at(r,c,CUST_CH_VOL);
}

inline bool cell_is_occupied_sparse(const Instance& inst, int cell) {
    auto it = inst.id_map.cell_to_customer_indices.find(cell);
    return it != inst.id_map.cell_to_customer_indices.end() && !it->second.empty();
}

inline std::string rep_id_from_cell(const Instance& inst, int cell) {
    std::string cid;
    if (gridopt::rep_customer_id_from_cell(inst, cell, cid)) return cid;
    return "";
}

// fixed territory map used only for "seed must belong to depot territory".
inline std::string nearest_depot_id_for_cell(const Instance& inst, int cell) {
    int r=0,c=0; inst.customers.decode_linear(cell, r, c);
    double best = 1e100;
    std::string best_id;
    for (const auto& d : inst.depots) {
        const double dr = (double)r - (double)d.row;
        const double dc = (double)c - (double)d.col;
        const double d2 = dr*dr + dc*dc;
        if (d2 < best) { best = d2; best_id = d.id; }
    }
    return best_id;
}

// GĐ2: Check if cell touches a set of members (8-neighbor) for frontier-only add
inline bool touches_members_8(const CustomerTensor& grid,
                              int cell,
                              const std::unordered_set<int>& members_set) {
    int r=0, c=0; grid.decode_linear(cell, r, c);
    for (int dr=-1; dr<=1; ++dr) {
        for (int dc=-1; dc<=1; ++dc) {
            if (dr==0 && dc==0) continue;
            int rr = r+dr, cc = c+dc;
            if (!grid.in_bounds(rr, cc)) continue;
            int nb = grid.linear(rr, cc);
            if (members_set.count(nb)) return true;
        }
    }
    return false;
}

} // namespace alns_detail

// =============================================================================
// Optimizer
// =============================================================================
class GridGrowALNS {
public:
    explicit GridGrowALNS(const Instance& inst, int seed = 1234567)
        : inst_(inst), rng_((unsigned)seed) {
        build_occupied_cells();
    }

    // Improve an initial solution.
    // - Returns a new Solution (does not mutate input).
    // - If you want visualization fields updated, call sync_solution_to_id_map().
    Solution run(const Solution& start, ALNSParams P = {}) {
        State S;
        from_solution(start, S);
        Solution cur = to_solution(S);
        Objectives::evaluate(inst_, cur);
        Solution best = cur;

        const double T0 = std::max(1e-9, P.temp0);
        const double Tend = std::max(1e-9, P.temp_end);

        // operator weights (very small set, adaptive but simple)
        std::vector<double> w_destroy = {1.0, 1.0, 1.0, 1.0};
        // 0 boundary-peel, 1 region-removal, 2 recenter-destroy (NEW), 3 center-shift-only
        std::vector<double> w_repair  = {1.0, 1.0};
        // 0 ring-regrow, 1 frontier-regrow

        auto pick = [&](const std::vector<double>& w) {
            double sum = 0.0;
            for (double x : w) sum += x;
            std::uniform_real_distribution<double> U(0.0, sum);
            double r = U(rng_);
            for (int i=0;i<(int)w.size();++i) {
                r -= w[i];
                if (r <= 0.0) return i;
            }
            return (int)w.size()-1;
        };

        // temp schedule (geometric)
        auto temp_at = [&](int it) {
            if (P.iterations <= 1) return T0;
            const double a = (double)it / (double)(P.iterations - 1);
            // geometric interpolation
            return T0 * std::pow(Tend / T0, a);
        };

        // Helper to count served cells
        auto count_served = [](const Solution& sol) {
            size_t served = 0;
            for (const auto& kv : sol.routes) {
                for (const auto& rt : kv.second) {
                    served += rt.member_cells_linear.size();
                }
            }
            return served;
        };

        // Initial logging
        if (P.verbose && P.log_interval > 0) {
            size_t init_served = count_served(best);
            size_t total = init_served + best.unassigned_cells.size();
            std::cout << "      [ALNS] Starting: obj=" << std::fixed << std::setprecision(2) << best.objective
                      << " served=" << init_served << "/" << total
                      << " (" << std::setprecision(1) << (100.0 * init_served / total) << "%)\n";
        }

        int accept_count = 0;
        int improve_count = 0;

        for (int it = 0; it < P.iterations; ++it) {
            const double T = temp_at(it);

            // Periodic updates for depot pressure (every 100 iters)
            if (it % 100 == 0) {
                update_depot_pressure(S);
            }

            // work on a copy of state (cheap: mostly vectors, but still ok per-iter)
            State Cand = S;

            // choose ops
            const int d_op = pick(w_destroy);
            const int r_op = pick(w_repair);

            // GĐ2: Random wipe-trip destroy to trigger reseed
            std::uniform_real_distribution<double> U01(0.0, 1.0);
            if (U01(rng_) < P.wipe_trip_prob) {
                destroy_wipe_trip(Cand, P);
            }

            // destroy
            apply_destroy(Cand, P, d_op);

            // GĐ2: Wipe small trips after destroy
            wipe_small_trips(Cand, P);

            // repair
            apply_repair(Cand, P, r_op);

            // GĐ2: Reseed empty trips
            for (int tid = 0; tid < (int)Cand.trips.size(); ++tid) {
                if (Cand.trips[tid].members.empty()) {
                    int seed = pick_unassigned_seed_for_vehicle(Cand, Cand.trips[tid], P);
                    if (seed >= 0) {
                        commit_seed_to_trip(Cand, tid, seed);
                    }
                }
            }

            // local improvement (VND-style)
            apply_vnd(Cand, P);

            // evaluate
            Solution cand_sol = to_solution(Cand);
            Objectives::evaluate(inst_, cand_sol);

            const double delta = cand_sol.objective - cur.objective;
            bool accept = false;
            if (delta <= 0.0) {
                accept = true;
            } else {
                std::uniform_real_distribution<double> U(0.0, 1.0);
                const double pr = std::exp(-delta / std::max(1e-9, T));
                accept = (U(rng_) < pr);
            }

            // update weights (very lightweight "reinforcement")
            if (accept) {
                S = std::move(Cand);
                cur = std::move(cand_sol);
                w_destroy[d_op] *= 1.02;
                w_repair[r_op]  *= 1.02;
                ++accept_count;
            } else {
                w_destroy[d_op] *= 0.995;
                w_repair[r_op]  *= 0.995;
            }
            // prevent degeneracy
            for (double& x : w_destroy) x = std::clamp(x, 0.10, 50.0);
            for (double& x : w_repair)  x = std::clamp(x, 0.10, 50.0);

            bool new_best = false;
            if (cur.objective < best.objective) {
                best = cur;
                new_best = true;
                ++improve_count;
            }

            // Progress logging
            if (P.verbose && P.log_interval > 0 && ((it + 1) % P.log_interval == 0 || new_best)) {
                size_t served = count_served(best);
                size_t total = served + best.unassigned_cells.size();
                std::cout << "      [ALNS] iter=" << std::setw(5) << (it + 1)
                          << " best=" << std::fixed << std::setprecision(2) << std::setw(14) << best.objective
                          << " served=" << served << "/" << total
                          << " (" << std::setprecision(1) << std::setw(5) << (100.0 * served / total) << "%)"
                          << " T=" << std::setprecision(1) << std::setw(8) << T;
                if (new_best) std::cout << " *NEW BEST*";
                std::cout << "\n";
            }
            
            // KPI logging at key points: iter 0, half, and end
            if (P.verbose && (it == 0 || it == P.iterations / 2 || it == P.iterations - 1)) {
                std::string kpi_tag = (it == 0) ? "START" : ((it == P.iterations / 2) ? "MIDDLE" : "END");
                print_all_kpis(S, kpi_tag);
            }
        }

        // Final summary logging
        if (P.verbose) {
            size_t final_served = count_served(best);
            size_t total = final_served + best.unassigned_cells.size();
            std::cout << "      [ALNS] Finished: " << P.iterations << " iters"
                      << ", accepted=" << accept_count << " (" << std::setprecision(1) << (100.0 * accept_count / P.iterations) << "%)"
                      << ", improved=" << improve_count << "\n";
        }

        return best;
    }

    // Update Instance.id_map rows (cluster/vehicle_id/depot_id) from a solution.
    // This is optional but useful for exporting CSV visualization.
    static void sync_solution_to_id_map(Instance& inst, const Solution& sol) {
        // clear
        for (auto& r : inst.id_map.rows) {
            r.cluster = -1;
            r.vehicle_id.clear();
            r.depot_id.clear();
        }

        int cluster_id = 0;
        for (const auto& kv : sol.routes) {
            const std::string& vid = kv.first;
            std::string depot_id;
            auto itV = inst.vehicles.find(vid);
            if (itV != inst.vehicles.end()) depot_id = itV->second.start_depot;

            for (const auto& rt : kv.second) {
                if (rt.member_cells_linear.empty()) { ++cluster_id; continue; }
                for (int cell : rt.member_cells_linear) {
                    auto it = inst.id_map.cell_to_customer_indices.find(cell);
                    if (it == inst.id_map.cell_to_customer_indices.end()) continue;
                    for (int cust_idx : it->second) {
                        if (cust_idx < 0 || cust_idx >= (int)inst.id_map.rows.size()) continue;
                        auto& row = inst.id_map.rows[(size_t)cust_idx];
                        row.cluster = cluster_id;
                        row.vehicle_id = vid;
                        row.depot_id = depot_id;
                    }
                }
                ++cluster_id;
            }
        }
    }

private:
    // ---------------- internal state ----------------
    struct Trip {
        std::string vehicle_id;
        std::string depot_id;
        VehicleType vtype = VehicleType::UNKNOWN;
        double cap_w = 0.0;
        double cap_v = 0.0;
        int center_cell = -1;
        std::vector<int> members;
        double load_w = 0.0;
        double load_v = 0.0;
        double distance_km = 0.0; // GĐ2: trip distance for veh_used_km tracking
    };

    struct State {
        std::vector<Trip> trips; // flattened trips
        std::unordered_map<std::string, double> veh_used_km; // NEW: sum of trip_km by vehicle

        // occupied cell assignment: cell_linear -> trip_id, -1 if unassigned
        std::unordered_map<int, int> cell_to_trip;
        std::vector<int> unassigned;
        std::unordered_set<int> unassigned_set;

        // Territory-free: cache cohesion penalty by depot_id
        std::unordered_map<std::string, double> depot_center_penalty;
        double total_cost = 0.0; // aggregated objective value (partial)
    };

    const Instance& inst_;
    std::mt19937 rng_;
    std::vector<int> occupied_cells_; // all occupied cells (sparse)
    
    // GĐ2: Load balancing
    std::unordered_map<std::string, double> depot_pressure_;
    SeedDebugInfo seed_dbg_; // Simplified debug counters
    
    // Target-first rescue: high-pressure depots needing rescue
    std::unordered_set<std::string> target_depots_;

    // ---------------- precompute ----------------
    void build_occupied_cells() {
        occupied_cells_.clear();
        occupied_cells_.reserve(inst_.id_map.cell_to_customer_indices.size());
        for (const auto& kv : inst_.id_map.cell_to_customer_indices) {
            occupied_cells_.push_back(kv.first);
        }
    }

    // GĐ2: Update depot pressure for load balancing
    void update_depot_pressure(const State& S) {
        std::unordered_map<std::string, int> fleet, dem;
        for (const auto& kv : inst_.vehicles) fleet[kv.second.start_depot]++;
        for (int cell : S.unassigned) {
            if (!S.unassigned_set.count(cell)) continue;
            dem[alns_detail::nearest_depot_id_for_cell(inst_, cell)]++;
        }
        depot_pressure_.clear();
        for (const auto& dep : inst_.depots) {
            depot_pressure_[dep.id] = (double)(dem[dep.id] + 1) / (fleet[dep.id] + 1);
        }
    }

    // Territory-free: Compute cohesion penalty for a list of centers
    double compute_center_penalty_direct(const std::vector<int>& centers, const SeedPolicyParams& sp) const {
        if (centers.empty()) return 0.0;

        auto get_km_dist = [&](int c1, int c2) {
            int r1=0, cc1=0, r2=0, cc2=0;
            inst_.customers.decode_linear(c1, r1, cc1);
            inst_.customers.decode_linear(c2, r2, cc2);
            double dr = (double)(r1 - r2) * 0.01; // 10m per cell
            double dc = (double)(cc1 - cc2) * 0.01;
            return std::sqrt(dr*dr + dc*dc);
        };

        // 1. Nearest Neighbor Penalty
        double nn_penalty = 0.0;
        if (centers.size() > 1) {
            for (size_t i = 0; i < centers.size(); ++i) {
                double dmin = 1e100;
                for (size_t j = 0; j < centers.size(); ++j) {
                    if (i == j) continue;
                    dmin = std::min(dmin, get_km_dist(centers[i], centers[j]));
                }
                nn_penalty += std::max(0.0, dmin - sp.R_adj) * sp.w_adj;
            }
        }

        // 2. Connected Components Penalty
        int n = (int)centers.size();
        std::vector<std::vector<int>> adj(n);
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                if (get_km_dist(centers[i], centers[j]) <= sp.R_adj + 1e-7) {
                    adj[i].push_back(j);
                    adj[j].push_back(i);
                }
            }
        }

        int num_cc = 0;
        std::vector<bool> visited(n, false);
        for (int i = 0; i < n; ++i) {
            if (!visited[i]) {
                num_cc++;
                std::vector<int> q = {i};
                visited[i] = true;
                int head = 0;
                while(head < (int)q.size()){
                    int u = q[head++];
                    for(int v : adj[u]){
                        if(!visited[v]){
                            visited[v] = true;
                            q.push_back(v);
                        }
                    }
                }
            }
        }
        double cc_penalty = (double)(num_cc - 1) * sp.w_cc;

        return nn_penalty + cc_penalty;
    }

    double compute_depot_center_penalty(const State& S, const std::string& depot_id, const SeedPolicyParams& sp) const {
        std::vector<int> centers;
        for (const auto& T : S.trips) {
            if (T.depot_id == depot_id && !T.members.empty() && T.center_cell >= 0) {
                centers.push_back(T.center_cell);
            }
        }
        return compute_center_penalty_direct(centers, sp);
    }

    // KPI-B: Compute depot balance stats
    DepotBalanceStats compute_depot_balance(const State& S) const {
        DepotBalanceStats stats;
        
        // Count served by depot
        for (const auto& T : S.trips) {
            stats.served_by_depot[T.depot_id] += (int)T.members.size();
        }
        
        // Count unserved with d1 = this depot
        for (int cell : S.unassigned) {
            if (!S.unassigned_set.count(cell)) continue;
            // This is a placeholder for a more complex territory assignment logic
            // For now, we'll just assign to the nearest depot.
            stats.unserved_d1[alns_detail::nearest_depot_id_for_cell(inst_, cell)]++;
        }
        
        // Copy pressure
        stats.pressure = depot_pressure_;
        
        return stats;
    }


    // KPI-C: (Removed legacy territory-based geometry stats)


    // Print all KPIs
    void print_all_kpis(const State& S, const std::string& tag) {
        std::cout << "\n========== KPI Report: " << tag << " ==========\n";
        
        // KPI-A: Seed debug
        seed_dbg_.print_summary(std::cout, tag);
        
        // KPI-B: Depot balance
        DepotBalanceStats db = compute_depot_balance(S);
        db.print(std::cout);
        
        std::cout << "===========================================\n";
    }

    // GĐ2: Wipe a trip -> move all members to unassigned pool
    void wipe_trip_to_pool(State& S, int tid) {
        auto& T = S.trips[tid];
        
        // Subtract distance from vehicle's used km
        auto kit = S.veh_used_km.find(T.vehicle_id);
        if (kit != S.veh_used_km.end()) {
            kit->second -= T.distance_km;
            if (kit->second < 0) kit->second = 0;
        }
        
        // Move members back to unassigned
        for (int cell : T.members) {
            S.cell_to_trip.erase(cell);
            S.unassigned.push_back(cell);
            S.unassigned_set.insert(cell);
        }
        T.members.clear();
        T.load_w = 0; T.load_v = 0;
        T.center_cell = -1;
        T.distance_km = 0;
    }

    // GĐ2: Destroy operator - wipe a random trip to trigger reseed
    void destroy_wipe_trip(State& S, const ALNSParams& P) {
        // Pick random non-empty trip
        std::vector<int> cand;
        for (int i = 0; i < (int)S.trips.size(); ++i) {
            if (!S.trips[i].members.empty()) cand.push_back(i);
        }
        if (cand.empty()) return;
        
        int tid = cand[rand_int(0, (int)cand.size() - 1)];
        wipe_trip_to_pool(S, tid);
    }

    // GĐ2: Wipe small trips (< min_members) to trigger reseed
    void wipe_small_trips(State& S, const ALNSParams& P) {
        for (int tid = 0; tid < (int)S.trips.size(); ++tid) {
            if (!S.trips[tid].members.empty() && 
                (int)S.trips[tid].members.size() < P.min_members_reseed) {
                wipe_trip_to_pool(S, tid);
            }
        }
    }

    // GĐ2: Commit seed to an empty trip
    void commit_seed_to_trip(State& S, int tid, int seed_cell) {
        auto& T = S.trips[tid];
        T.center_cell = seed_cell;
        T.members.push_back(seed_cell);
        T.load_w = alns_detail::cell_w(inst_, seed_cell);
        T.load_v = alns_detail::cell_v(inst_, seed_cell);
        
        // Estimate distance from depot
        std::string cid = alns_detail::rep_id_from_cell(inst_, seed_cell);
        if (!cid.empty()) {
            T.distance_km = 2.0 * inst_.get_dist_km(T.depot_id, cid);
        }
        
        // Remove from unassigned
        S.unassigned_set.erase(seed_cell);
        S.unassigned.erase(
            std::remove(S.unassigned.begin(), S.unassigned.end(), seed_cell),
            S.unassigned.end()
        );
        S.cell_to_trip[seed_cell] = tid;
        
        // Update depot region
        // depot_region_[T.depot_id].insert(seed_cell); // Removed as depot_region_ is not maintained
        
        // Update vehicle used km
        auto kit = S.veh_used_km.find(T.vehicle_id);
        if (kit != S.veh_used_km.end()) {
            kit->second += T.distance_km;
        } else {
            S.veh_used_km[T.vehicle_id] = T.distance_km;
        }
    }

    // ---------------- conversions ----------------
    void from_solution(const Solution& sol, State& S) {
        S.trips.clear();
        S.cell_to_trip.clear();
        S.unassigned.clear();
        S.unassigned_set.clear();
        S.veh_used_km.clear();

        // flatten trips from solution
        for (const auto& kv : sol.routes) {
            const std::string& vid = kv.first;
            auto itV = inst_.vehicles.find(vid);
            if (itV == inst_.vehicles.end()) continue;
            const Vehicle& veh = itV->second;

            for (const auto& rt : kv.second) {
                Trip T;
                T.vehicle_id = vid;
                T.depot_id = veh.start_depot;
                T.vtype = veh.type;
                T.cap_w = veh.cap_weight;
                T.cap_v = veh.cap_volume;
                T.center_cell = rt.center_cell_linear;
                T.members = rt.member_cells_linear;
                recompute_load(T);
                if (T.center_cell < 0 || !contains_member(T, T.center_cell)) {
                    if (!T.members.empty()) T.center_cell = T.members[rand_int(0, (int)T.members.size()-1)];
                }

                const int tid = (int)S.trips.size();
                S.trips.push_back(std::move(T));

                // assign cells
                for (int cell : S.trips.back().members) {
                    S.cell_to_trip[cell] = tid;
                }
            }
        }

        // build unassigned from occupied_cells
        for (int cell : occupied_cells_) {
            if (S.cell_to_trip.find(cell) == S.cell_to_trip.end()) {
                S.unassigned.push_back(cell);
                S.unassigned_set.insert(cell);
            }
        }

        // ensure uniqueness
        enforce_unique_assignments(S);

        // --- NEW: Init veh_used_km ---
        for (const auto& kv : inst_.vehicles) S.veh_used_km[kv.first] = 0.0;
        for (const auto& T : S.trips) {
            S.veh_used_km[T.vehicle_id] += trip_km(T);
        }
    }

    Solution to_solution(const State& S) const {
        Solution sol;
        // group trips back by vehicle
        for (const auto& T : S.trips) {
            Route rt;
            rt.vehicle_id = T.vehicle_id;
            rt.center_cell_linear = T.center_cell;
            rt.member_cells_linear = T.members;
            rt.load_w = T.load_w;
            rt.load_v = T.load_v;
            // distance_km cached (optional)
            rt.distance_km = trip_km(T);
            sol.routes[T.vehicle_id].push_back(std::move(rt));
        }
        sol.unassigned_cells.clear();
        for (int cell : S.unassigned) sol.unassigned_cells.insert(cell);
        return sol;
    }

    // ---------------- random ----------------
    int rand_int(int lo, int hi) {
        if (hi < lo) return lo;
        std::uniform_int_distribution<int> D(lo, hi);
        return D(rng_);
    }
    double rand01() {
        std::uniform_real_distribution<double> U(0.0, 1.0);
        return U(rng_);
    }

    // ---------------- trip stats ----------------
    static bool contains_member(const Trip& T, int cell) {
        for (int x : T.members) if (x == cell) return true;
        return false;
    }

    void recompute_load(Trip& T) const {
        double w=0.0, v=0.0;
        for (int cell : T.members) {
            w += alns_detail::cell_w(inst_, cell);
            v += alns_detail::cell_v(inst_, cell);
        }
        T.load_w = w;
        T.load_v = v;
    }
int cheb_dist_cells(int a, int b) const {
    int ar=0, ac=0, br=0, bc=0;
    inst_.customers.decode_linear(a, ar, ac);
    inst_.customers.decode_linear(b, br, bc);
    return std::max(std::abs(ar - br), std::abs(ac - bc));
}

void recompute_center(Trip& T) const {
    if (T.members.empty()) { T.center_cell = -1; return; }
    // Center = member closest to mean(row,col). Stable tie-break on cell id.
    double sumr = 0.0, sumc = 0.0;
    for (int cell : T.members) {
        int r=0, c=0; inst_.customers.decode_linear(cell, r, c);
        sumr += (double)r;
        sumc += (double)c;
    }
    const double mr = sumr / (double)T.members.size();
    const double mc = sumc / (double)T.members.size();

    int best = T.members[0];
    double bestd = 1e100;
    for (int cell : T.members) {
        int r=0, c=0; inst_.customers.decode_linear(cell, r, c);
        const double dr = (double)r - mr;
        const double dc = (double)c - mc;
        const double d2 = dr*dr + dc*dc;
        if (d2 < bestd - 1e-12 || (std::abs(d2 - bestd) <= 1e-12 && cell < best)) {
            bestd = d2;
            best = cell;
        }
    }
    T.center_cell = best;
}


    double trip_km_from_center(const Trip& T, int center_cell) const {
        if (center_cell < 0) return 0.0;
        const std::string cid = alns_detail::rep_id_from_cell(inst_, center_cell);
        if (cid.empty() || T.depot_id.empty()) return 1e9;
        const double d = inst_.get_dist_km(T.depot_id, cid);
        if (d >= 1e8) return 1e9;
        return 2.0 * d;
    }

    double trip_km(const Trip& T) const {
        return trip_km_from_center(T, T.center_cell);
    }

    bool can_add_cell(const Trip& T, int cell, const ALNSParams& P) const {
        const double w = alns_detail::cell_w(inst_, cell);
        const double v = alns_detail::cell_v(inst_, cell);
        const double capW = (T.cap_w > 0.0) ? T.cap_w * (1.0 + P.cap_slack) : std::numeric_limits<double>::infinity();
        const double capV = (T.cap_v > 0.0) ? T.cap_v * (1.0 + P.cap_slack) : std::numeric_limits<double>::infinity();
        if (T.load_w + w > capW + 1e-9) return false;
        if (T.load_v + v > capV + 1e-9) return false;

        if (T.center_cell >= 0) {
            int base_limit = init_detail::spread_ring_limit(T.vtype);
            int Rmax = (int)std::floor((double)base_limit * std::max(0.1, P.ring_limit_mul));
            Rmax = std::max(1, Rmax);
            if (cheb_dist_cells(T.center_cell, cell) > Rmax) return false;
        }
        return true;
    }

    struct InsertEval {
        bool ok = false;
        int new_center = -1;
        double new_trip_km = 0.0;
        double score = -1e100;
    };

    InsertEval eval_insert_like_init(const State& S, const Trip& T, int cell, int ringR, const ALNSParams& P) const {
        InsertEval out;
        if (!S.unassigned_set.count(cell)) return out;
        if (!alns_detail::cell_is_occupied_sparse(inst_, cell)) return out;
        if (!can_add_cell(T, cell, P)) return out;

        // Territory-free: no hard territory check based on d1 anymore.

        // Simulate new center
        std::vector<int> tmp = T.members;
        tmp.push_back(cell);
        int new_center = init_detail::update_center_by_mean_grid(inst_.customers, tmp);
        if (new_center < 0) return out;

        double new_km = trip_km_from_center(T, new_center);
        if (new_km >= 1e8) return out;

        // max_dist hard check
        auto itV = inst_.vehicles.find(T.vehicle_id);
        if (itV != inst_.vehicles.end() && itV->second.max_dist > 0.0) {
            double old_km = trip_km(T);
            double used_base = S.veh_used_km.at(T.vehicle_id) - old_km;
            if (used_base + new_km > itV->second.max_dist + 1e-9) return out;
        }

        // score info (max-score is better)
        double w = alns_detail::cell_w(inst_, cell);
        double pri = init_detail::cell_priority(inst_, cell);
        double score = 1000.0 * w + 10.0 * pri - 0.1 * (double)ringR;

        // Territory-free: add cohesion penalty delta (optimized)
        double old_pen = 0.0;
        auto itP = S.depot_center_penalty.find(T.depot_id);
        if (itP != S.depot_center_penalty.end()) old_pen = itP->second;

        SeedPolicyParams sp_cfg;
        std::vector<int> sim_centers;
        for (const auto& st : S.trips) {
            if (st.depot_id == T.depot_id && !st.members.empty()) {
                if (st.vehicle_id == T.vehicle_id) {
                    sim_centers.push_back(new_center);
                } else {
                    sim_centers.push_back(st.center_cell);
                }
            }
        }
        // If the current trip was empty, it wasn't in the loop above
        if (T.members.empty()) {
             sim_centers.push_back(new_center);
        }

        double new_pen = compute_center_penalty_direct(sim_centers, sp_cfg);
        double delta_pen = new_pen - old_pen;
        score -= delta_pen; // penalty reduces score

        out.ok = true;
        out.new_center = new_center;
        out.new_trip_km = new_km;
        out.score = score;
        return out;
    }

    void commit_insert(State& S, int tid, int cell, int new_center, double new_trip_km) {
        auto& T = S.trips[tid];
        double old_km = trip_km(T);

        T.members.push_back(cell);
        S.cell_to_trip[cell] = tid;
        auto it = S.unassigned_set.find(cell);
        if (it != S.unassigned_set.end()) {
            S.unassigned_set.erase(it);
            // lazy delete from vector; we clean occasionally
        }

        T.load_w += alns_detail::cell_w(inst_, cell);
        T.load_v += alns_detail::cell_v(inst_, cell);
        T.center_cell = new_center;

        S.veh_used_km[T.vehicle_id] += (new_trip_km - old_km);
        
        // Territory-free: update cohesion penalty incrementally
        double old_dp = 0.0;
        auto itP = S.depot_center_penalty.find(T.depot_id);
        if (itP != S.depot_center_penalty.end()) old_dp = itP->second;
        
        SeedPolicyParams sp_cfg;
        double new_dp = compute_depot_center_penalty(S, T.depot_id, sp_cfg);
        S.total_cost += (new_dp - old_dp);
        S.depot_center_penalty[T.depot_id] = new_dp;

        // Note: we also updated vehicle_km which is part of objective.
        // The calling operator should update total_cost for distance change if needed.
        // GridGrowALNS usually handles trip_km updates in total_cost separately or via full recompute.
        // But here we'll be safe:
        S.total_cost += (new_trip_km - old_km);
    }

    void seed_trip(State& S, int tid, int seed_cell) {
        auto& T = S.trips[tid];
        double km = trip_km_from_center(T, seed_cell);
        commit_insert(S, tid, seed_cell, seed_cell, km);
    }

    void normalize_trip(State& S, int tid) {
        auto& T = S.trips[tid];
        double old_km = trip_km(T);
        double old_cp = (S.depot_center_penalty.count(T.depot_id)) ? S.depot_center_penalty.at(T.depot_id) : 0.0;

        if (T.members.empty()) {
            T.center_cell = -1;
            T.load_w = T.load_v = 0.0;
            S.veh_used_km[T.vehicle_id] -= old_km;
        } else {
            recompute_load(T);
            T.center_cell = init_detail::update_center_by_mean_grid(inst_.customers, T.members);
            double new_km = trip_km(T);
            S.veh_used_km[T.vehicle_id] += (new_km - old_km);
        }

        // Always update cohesion for the affected depot
        SeedPolicyParams sp_cfg;
        double new_cp = compute_depot_center_penalty(S, T.depot_id, sp_cfg);
        S.total_cost += (new_cp - old_cp);
        S.depot_center_penalty[T.depot_id] = new_cp;
    }

    // ---------------- uniqueness & membership ----------------
    void enforce_unique_assignments(State& S) {
        // S.cell_to_trip is a map (unique by key), but duplicates can exist if two trips have same cell.
        // We will rebuild from scratch and drop duplicates.
        std::unordered_map<int,int> new_map;
        new_map.reserve(S.cell_to_trip.size());
        std::unordered_set<int> dup;

        for (int tid = 0; tid < (int)S.trips.size(); ++tid) {
            auto& T = S.trips[tid];
            std::vector<int> kept;
            kept.reserve(T.members.size());
            for (int cell : T.members) {
                auto it = new_map.find(cell);
                if (it == new_map.end()) {
                    new_map[cell] = tid;
                    kept.push_back(cell);
                } else {
                    dup.insert(cell);
                }
            }
            T.members.swap(kept);
            if (!T.members.empty() && (T.center_cell < 0 || !contains_member(T, T.center_cell))) {
                T.center_cell = T.members[0];
            }
            recompute_load(T);
        }

        S.cell_to_trip.swap(new_map);
        // duplicates go to unassigned
        for (int cell : dup) {
            if (!S.unassigned_set.count(cell)) {
                S.unassigned.push_back(cell);
                S.unassigned_set.insert(cell);
            }
        }
    }

    // =============================================================================
    // Operators
    // =============================================================================
    int pick_nonempty_trip(const State& S) {
        std::vector<int> cand;
        cand.reserve(S.trips.size());
        for (int i=0;i<(int)S.trips.size();++i) if (!S.trips[i].members.empty()) cand.push_back(i);
        if (cand.empty()) return -1;
        return cand[rand_int(0, (int)cand.size()-1)];
    }

    int pick_any_trip(const State& S) {
        if (S.trips.empty()) return -1;
        return rand_int(0, (int)S.trips.size()-1);
    }

    // Compute boundary cells: members that have at least one 8-neighbor not in the same trip.
    void boundary_cells(const State& S, int tid, std::vector<int>& out) const {
        out.clear();
        const auto& T = S.trips[tid];
        if (T.members.empty()) return;
        out.reserve(std::min<size_t>(T.members.size(), 2048));
        int nbr[8], nn=0;
        for (int cell : T.members) {
            alns_detail::nbr8_linear(inst_.customers, cell, nbr, nn);
            bool is_boundary = false;
            for (int i=0;i<nn;++i) {
                const int nb = nbr[i];
                auto it = S.cell_to_trip.find(nb);
                // Ignore truly empty cells (no customer in sparse grid). Boundary should reflect contact with
                // other assigned/unassigned *occupied* cells, not the void.
                if (!alns_detail::cell_is_occupied_sparse(inst_, nb)) continue;
                if (it == S.cell_to_trip.end() || it->second != tid) {
                    // neighbor is unassigned or belongs to another trip
                    is_boundary = true;
                    break;
                }
            }
            if (is_boundary) out.push_back(cell);
        }
    }

    void remove_cell(State& S, int tid, int cell) {
        auto& T = S.trips[tid];
        // erase from members (swap-remove)
        for (size_t i=0;i<T.members.size();++i) {
            if (T.members[i] == cell) {
                T.members[i] = T.members.back();
                T.members.pop_back();
                break;
            }
        }
        S.cell_to_trip.erase(cell);
        if (!S.unassigned_set.count(cell)) {
            S.unassigned.push_back(cell);
            S.unassigned_set.insert(cell);
        }
        // update load
        T.load_w -= alns_detail::cell_w(inst_, cell);
        T.load_v -= alns_detail::cell_v(inst_, cell);
        if (T.members.empty()) {
            T.center_cell = -1;
            T.load_w = 0.0;
            T.load_v = 0.0;
        } else if (T.center_cell == cell) {
            // keep center valid
            T.center_cell = T.members[rand_int(0, (int)T.members.size()-1)];
        }
    }

    void add_cell(State& S, int tid, int cell) {
        auto& T = S.trips[tid];
        T.members.push_back(cell);
        S.cell_to_trip[cell] = tid;
        auto it = S.unassigned_set.find(cell);
        if (it != S.unassigned_set.end()) {
            S.unassigned_set.erase(it);
            // lazy delete from vector; we clean occasionally
        }
        T.load_w += alns_detail::cell_w(inst_, cell);
        T.load_v += alns_detail::cell_v(inst_, cell);
        if (T.center_cell < 0) T.center_cell = cell;
    }

    void cleanup_unassigned(State& S) {
        if (S.unassigned.empty()) return;
        std::vector<int> fresh;
        fresh.reserve(S.unassigned_set.size());
        for (int cell : S.unassigned) {
            if (S.unassigned_set.count(cell)) fresh.push_back(cell);
        }
        S.unassigned.swap(fresh);
    }

    // ---------------- Destroy ops ----------------
    void destroy_boundary_peel(State& S, const ALNSParams& P, int tid) {
        std::vector<int> bd;
        boundary_cells(S, tid, bd);
        if (bd.empty()) return;

        auto& T = S.trips[tid];
        const int n = (int)T.members.size();
        const double frac = P.destroy_frac_min + (P.destroy_frac_max - P.destroy_frac_min) * rand01();
        int rem = std::max(1, (int)std::floor(frac * n));
        std::shuffle(bd.begin(), bd.end(), rng_);
        rem = std::min(rem, (int)bd.size());
        for (int i=0;i<rem;++i) remove_cell(S, tid, bd[i]);
        cleanup_unassigned(S);
    }

    void destroy_connected_region(State& S, const ALNSParams& P, int tid) {
        auto& T = S.trips[tid];
        if (T.members.size() <= 2) return;

        const double frac = P.region_frac_min + (P.region_frac_max - P.region_frac_min) * rand01();
        const int target = std::max(1, (int)std::floor(frac * (double)T.members.size()));

        // BFS inside trip
        const int seed = T.members[rand_int(0, (int)T.members.size()-1)];
        std::unordered_set<int> visited;
        visited.reserve((size_t)target * 2);
        std::deque<int> q;
        q.push_back(seed);
        visited.insert(seed);

        std::vector<int> region;
        region.reserve((size_t)target);

        int nbr[8], nn=0;
        while (!q.empty() && (int)region.size() < target) {
            const int u = q.front(); q.pop_front();
            region.push_back(u);
            alns_detail::nbr8_linear(inst_.customers, u, nbr, nn);
            for (int i=0;i<nn;++i) {
                const int v = nbr[i];
                if (visited.count(v)) continue;
                auto it = S.cell_to_trip.find(v);
                if (it != S.cell_to_trip.end() && it->second == tid) {
                    visited.insert(v);
                    q.push_back(v);
                }
            }
        }

        for (int cell : region) remove_cell(S, tid, cell);
        cleanup_unassigned(S);
    }

    // NEW destroy: remove some part and then shift center to a (random-sampled) cell
    // that has many unassigned neighbors within a radius (adaptive crawling).
    void destroy_recenter_adaptive(State& S, const ALNSParams& P, int tid) {
        // destroy some cells first
        if (rand01() < 0.5) destroy_boundary_peel(S, P, tid);
        else destroy_connected_region(S, P, tid);

        auto& T = S.trips[tid];
        // choose new center
        if (!T.members.empty()) {
            T.center_cell = pick_adaptive_center(S, tid, P);
        } else {
            // trip empty -> pick new seed from unassigned in its territory
            int seed = pick_unassigned_seed_for_vehicle(S, T, P);
            if (seed >= 0) {
                seed_trip(S, tid, seed);
            }
        }
    }

    void destroy_center_shift_only(State& S, const ALNSParams& P, int tid) {
        auto& T = S.trips[tid];
        if (T.members.size() <= 1) return;
        (void)P;
        T.center_cell = pick_adaptive_center(S, tid, P);
    }

    // pick a center by sampling candidate cells and scoring by number of nearby unassigned
    int pick_adaptive_center(const State& S, int tid, const ALNSParams& P) {
        const auto& T = S.trips[tid];
        const int K = std::max(1, P.recenter_sample);
        int best_cell = T.members[rand_int(0, (int)T.members.size()-1)];
        int best_score = -1;

        for (int k=0;k<K;++k) {
            int cand = T.members[rand_int(0, (int)T.members.size()-1)];
            // score: count unassigned occupied cells within Chebyshev radius
            int score = count_unassigned_within_R(S, cand, P.recenter_probe_R);
            if (score > best_score) {
                best_score = score;
                best_cell = cand;
            }
        }
        return best_cell;
    }

    int count_unassigned_within_R(const State& S, int center_cell, int R) const {
        if (R <= 0) return 0;
        int cr=0, cc=0; inst_.customers.decode_linear(center_cell, cr, cc);
        int cnt = 0;
        for (int dr=-R; dr<=R; ++dr) {
            for (int dc=-R; dc<=R; ++dc) {
                const int rr = cr + dr;
                const int cc2 = cc + dc;
                if (!inst_.customers.in_bounds(rr, cc2)) continue;
                const int cell = inst_.customers.linear(rr, cc2);
                if (S.unassigned_set.count(cell)) ++cnt;
            }
        }
        return cnt;
    }

    // Seed for cluster empty according to the patch centers of the depot
    int pick_seed_patch(const State& S, const std::string& depot_id, const SeedPolicyParams& sp) {
        // Collect existing centers for this depot
        std::vector<int> centers;
        for (const auto& T : S.trips) {
            if (T.depot_id == depot_id && !T.members.empty() && T.center_cell >= 0) {
                centers.push_back(T.center_cell);
            }
        }

        if (centers.empty()) return -1;

        // Try to pick an unassigned cell near one of the existing centers
        std::shuffle(centers.begin(), centers.end(), rng_);
        for (int center : centers) {
            int cr = 0, cc = 0;
            inst_.customers.decode_linear(center, cr, cc);
            for (int R = 1; R <= sp.seed_ring_max; ++R) {
                // Shuffle edges
                int edges[] = {0, 1, 2, 3};
                std::shuffle(edges, edges + 4, rng_);
                for (int e : edges) {
                    for (int i = -R; i <= R; ++i) {
                        int rr = cr, ccc = cc;
                        if (e == 0) { rr -= R; ccc += i; }
                        else if (e == 1) { rr += R; ccc += i; }
                        else if (e == 2) { rr += i; ccc -= R; }
                        else { rr += i; ccc += R; }

                        if (inst_.customers.in_bounds(rr, ccc)) {
                            int v = inst_.customers.linear(rr, ccc);
                            if (S.unassigned_set.count(v) && alns_detail::cell_is_occupied_sparse(inst_, v)) return v;
                        }
                    }
                }
            }
        }
        return -1;
    }

    int pick_unassigned_seed_for_vehicle(State& S, const Trip& T, const ALNSParams& P) {
        if (S.unassigned.empty()) return -1;
        SeedPolicyParams sp_cfg;
        // seed_dbg_.calls++; // skip for now

        // 1. Try Patch-Based Seeding (geographic clustering)
        int patch_seed = pick_seed_patch(S, T.depot_id, sp_cfg);
        if (patch_seed >= 0 && rand01() < sp_cfg.w_patch) return patch_seed;

        // 2. Near-depot Seed
        int depot_r=0, depot_c=0; bool found_depot=false;
        for(const auto& d : inst_.depots) { if(d.id == T.depot_id){ depot_r=d.row; depot_c=d.col; found_depot=true; break; } }
        if (found_depot) {
            for (int r=1; r<=100; r+=10) {
                std::vector<int> cand;
                for(int it=0; it<30; ++it) {
                    int rr = depot_r + rand_int(-r, r);
                    int cc = depot_c + rand_int(-r, r);
                    if(inst_.customers.in_bounds(rr, cc)) {
                        int cell = inst_.customers.linear(rr, cc);
                        if(S.unassigned_set.count(cell) && alns_detail::cell_is_occupied_sparse(inst_, cell)) cand.push_back(cell);
                    }
                }
                if(!cand.empty()) return cand[rand_int(0, (int)cand.size()-1)];
            }
        }

        // 3. Global Fallback
        int tries = std::min(sp_cfg.seed_sample_global, (int)S.unassigned.size());
        int best = -1; double best_score = -1e100;
        for(int i=0; i<tries; ++i) {
            int cell = S.unassigned[rand_int(0, (int)S.unassigned.size()-1)];
            if(!S.unassigned_set.count(cell)) continue;
            double p = (depot_pressure_.count(T.depot_id)) ? depot_pressure_.at(T.depot_id) : 1.0;
            double dk = inst_.get_dist_km(T.depot_id, alns_detail::rep_id_from_cell(inst_, cell));
            double score = (p >= sp_cfg.high_pressure_nearfirst_th) ? -dk : dk;
            if(score > best_score) { best_score = score; best = cell; }
        }
        return best;
    }

    void apply_destroy(State& S, const ALNSParams& P, int op) {
        int tid = (op == 3) ? pick_nonempty_trip(S) : pick_any_trip(S);
        if (tid < 0) return;
        if (S.trips[tid].members.empty() && op != 2) return;

        switch (op) {
            case 0: destroy_boundary_peel(S, P, tid); break;
            case 1: destroy_connected_region(S, P, tid); break;
            case 2: destroy_recenter_adaptive(S, P, tid); break;
            case 3: destroy_center_shift_only(S, P, tid); break;
            default: destroy_boundary_peel(S, P, tid); break;
        }
        normalize_trip(S, tid);
    }

    // ---------------- Repair ops ----------------
    void repair_ring_regrow(State& S, const ALNSParams& P) {
        // regrow each trip a bit (shuffled)
        std::vector<int> order(S.trips.size());
        std::iota(order.begin(), order.end(), 0);
        std::shuffle(order.begin(), order.end(), rng_);
        for (int tid : order) {
            grow_trip_ring(S, P, tid);
        }
        cleanup_unassigned(S);
    }

    void repair_frontier_regrow(State& S, const ALNSParams& P) {
        std::vector<int> order(S.trips.size());
        std::iota(order.begin(), order.end(), 0);
        std::shuffle(order.begin(), order.end(), rng_);
        for (int tid : order) {
            grow_trip_frontier(S, P, tid);
        }
        cleanup_unassigned(S);
    }

    void apply_repair(State& S, const ALNSParams& P, int op) {
        switch (op) {
            case 0: repair_ring_regrow(S, P); break;
            case 1: repair_frontier_regrow(S, P); break;
            default: repair_frontier_regrow(S, P); break;
        }
    }

    // Ring-boundary growth around center (Chebyshev rings).
    void grow_trip_ring(State& S, const ALNSParams& P, int tid) {
        auto& T = S.trips[tid];
        if (S.unassigned_set.empty()) return;

        if (T.members.empty()) {
            int seed = pick_unassigned_seed_for_vehicle(S, T, P);
            if (seed < 0) return;
            seed_trip(S, tid, seed);
        }

        int base_limit = init_detail::spread_ring_limit(T.vtype);
        int Rmax = std::max(1, (int)std::floor((double)base_limit * std::max(0.1, P.ring_limit_mul)));

        int added = 0;
        while (!S.unassigned_set.empty() && added < P.max_add_per_step) {
            int cr=0, cc=0;
            inst_.customers.decode_linear(T.center_cell, cr, cc);

            bool accepted = false;
            int best_cell = -1;
            InsertEval best;

            for (int R=1; R<=Rmax; ++R) {
                best_cell = -1;
                best = InsertEval();

                init_detail::for_each_ring_boundary(inst_.customers, cr, cc, R, [&](int rr, int cc2){
                    int cell = inst_.customers.linear(rr, cc2);
                    if (!S.unassigned_set.count(cell)) return;
                    if (!alns_detail::cell_is_occupied_sparse(inst_, cell)) return;

                    auto ev = eval_insert_like_init(S, T, cell, R, P);
                    if (!ev.ok) return;
                    if (best_cell < 0 || ev.score > best.score) {
                        best = ev;
                        best_cell = cell;
                    }
                });

                if (best_cell >= 0) {
                    accepted = true;
                    break;
                }
            }

            if (!accepted) break;

            commit_insert(S, tid, best_cell, best.new_center, best.new_trip_km);
            ++added;

            if (T.cap_w > 0.0 && T.load_w >= T.cap_w * (1.0 + P.cap_slack) - 1e-9) break;
            if (T.cap_v > 0.0 && T.load_v >= T.cap_v * (1.0 + P.cap_slack) - 1e-9) break;
        }
    }

    // Frontier-based regrow: BFS-like expansion via 8-neighbors from current members.
    void grow_trip_frontier(State& S, const ALNSParams& P, int tid) {
        auto& T = S.trips[tid];
        if (S.unassigned_set.empty()) return;

        if (T.members.empty()) {
            int seed = pick_unassigned_seed_for_vehicle(S, T, P);
            if (seed < 0) return;
            seed_trip(S, tid, seed);
        }

        int added = 0;
        int nbr[8], nn=0;
        while (!S.unassigned_set.empty() && added < P.max_add_per_step) {
            std::unordered_set<int> seen;
            seen.reserve(T.members.size() * 2 + 32);
            std::vector<int> cand;
            cand.reserve(512);

            for (int u : T.members) {
                alns_detail::nbr8_linear(inst_.customers, u, nbr, nn);
                for (int i=0; i<nn; ++i) {
                    int v = nbr[i];
                    if (!S.unassigned_set.count(v)) continue;
                    if (!alns_detail::cell_is_occupied_sparse(inst_, v)) continue;
                    if (seen.insert(v).second) cand.push_back(v);
                }
            }
            
            // Soft K_d: Sample additional cells from high-pressure depots
            SeedPolicyParams sp_cfg;
            if (sp_cfg.soft_kd_enabled) {
                // Find high-pressure depots
                std::unordered_set<std::string> high_p;
                std::vector<std::pair<std::string, double>> v_p;
                for (const auto& dep : inst_.depots) v_p.push_back({dep.id, depot_pressure_[dep.id]});
                std::sort(v_p.begin(), v_p.end(), [](auto& a, auto& b) { return a.second > b.second; });
                for (int i=0; i<3 && i<(int)v_p.size(); ++i) high_p.insert(v_p[i].first);

                int target_samples = std::min(50, (int)S.unassigned.size());
                for (int t = 0; t < target_samples && cand.size() < 200; ++t) {
                    int idx = rand_int(0, (int)S.unassigned.size() - 1);
                    int v = S.unassigned[idx];
                    if (!S.unassigned_set.count(v)) continue;
                    if (seen.count(v)) continue;
                    
                    std::string d1 = alns_detail::nearest_depot_id_for_cell(inst_, v);
                    if (!high_p.count(d1)) continue;
                    seen.insert(v);
                    cand.push_back(v);
                }
            }
            
            if (cand.empty()) break;

            int best_cell = -1;
            InsertEval best;

            for (int cell : cand) {
                int R = std::max(1, cheb_dist_cells(T.center_cell, cell));
                auto ev = eval_insert_like_init(S, T, cell, R, P);
                if (!ev.ok) continue;
                if (best_cell < 0 || ev.score > best.score) {
                    best = ev;
                    best_cell = cell;
                }
            }
            if (best_cell < 0) break;

            commit_insert(S, tid, best_cell, best.new_center, best.new_trip_km);
            ++added;

            if (T.cap_w > 0.0 && T.load_w >= T.cap_w * (1.0 + P.cap_slack) - 1e-9) break;
            if (T.cap_v > 0.0 && T.load_v >= T.cap_v * (1.0 + P.cap_slack) - 1e-9) break;
        }
    }

    // ---------------- VND ops (very lightweight) ----------------
    void apply_vnd(State& S, const ALNSParams& P) {
        if (P.vnd_tries <= 0) return;
        // simple moves: relocate boundary cell to neighbor trip or swap boundary cells.
        for (int t=0; t<P.vnd_tries; ++t) {
            if (rand01() < 0.5) vnd_relocate_boundary(S, P);
            else vnd_swap_boundary(S, P);
        }
        cleanup_unassigned(S);
    }

    void vnd_relocate_boundary(State& S, const ALNSParams& P) {
        (void)P;
        int a = pick_nonempty_trip(S);
        if (a < 0) return;
        std::vector<int> bd;
        boundary_cells(S, a, bd);
        if (bd.empty()) return;
        const int cell = bd[rand_int(0, (int)bd.size()-1)];

        // find a neighboring trip
        int nbr[8], nn=0;
        alns_detail::nbr8_linear(inst_.customers, cell, nbr, nn);
        std::shuffle(nbr, nbr+nn, rng_);
        int b = -1;
        for (int i=0;i<nn;++i) {
            auto it = S.cell_to_trip.find(nbr[i]);
            if (it != S.cell_to_trip.end() && it->second != a) { b = it->second; break; }
        }
        if (b < 0) return;
        auto& TA = S.trips[a];
        auto& TB = S.trips[b];

        // check if b can take it (use eval_insert)
        auto ev = eval_insert_like_init(S, TB, cell, 1, P);
        if (!ev.ok) return;

        // perform move
        remove_cell(S, a, cell);
        add_cell(S, b, cell);
        normalize_trip(S, a);
        normalize_trip(S, b);
    }

    void vnd_swap_boundary(State& S, const ALNSParams& P) {
        int a = pick_nonempty_trip(S);
        int b = pick_nonempty_trip(S);
        if (a < 0 || b < 0 || a == b) return;

        std::vector<int> ba, bb;
        boundary_cells(S, a, ba);
        boundary_cells(S, b, bb);
        if (ba.empty() || bb.empty()) return;
        const int ca = ba[rand_int(0, (int)ba.size()-1)];
        const int cb = bb[rand_int(0, (int)bb.size()-1)];

        auto& TA = S.trips[a];
        auto& TB = S.trips[b];

        // check swap feasibility (approx via load check first, then normalize will update exactly)
        const double wa = alns_detail::cell_w(inst_, ca);
        const double va = alns_detail::cell_v(inst_, ca);
        const double wb = alns_detail::cell_w(inst_, cb);
        const double vb = alns_detail::cell_v(inst_, cb);

        if (TA.cap_w > 0.0 && TA.load_w - wa + wb > TA.cap_w * (1.0 + P.cap_slack) + 1e-9) return;
        if (TB.cap_w > 0.0 && TB.load_w - wb + wa > TB.cap_w * (1.0 + P.cap_slack) + 1e-9) return;

        remove_cell(S, a, ca);
        remove_cell(S, b, cb);
        add_cell(S, a, cb);
        add_cell(S, b, ca);
        normalize_trip(S, a);
        normalize_trip(S, b);
    }
};

} // namespace gridopt
