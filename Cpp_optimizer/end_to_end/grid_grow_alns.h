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
#include <limits>
#include <numeric>
#include <random>
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
    // fraction of members to remove (was 8%..28%, now 15%..50%)
    double destroy_frac_min = 0.15;
    double destroy_frac_max = 0.50;

    // connected-region removal target size factor (was 12%..35%, now 20%..55%)
    double region_frac_min = 0.20;
    double region_frac_max = 0.55;

    // how many attempts of local improvement per iteration (was 20, now 40)
    int vnd_tries = 40;

    // --- acceptance ---
    double temp0 = 5.0e4;      // starting temperature (increased from 2.5e4)
    double temp_end = 0.5;     // ending temperature (decreased from 1.0)

    // --- feasibility behavior ---
    double cap_slack = 0.00;   // allow a little overflow during grow (0 = keep within cap)

    // growth search radius multiplier (increased from 1.0 to 1.5)
    double ring_limit_mul = 1.5;

    // adaptive re-center sampling size (increased from 10 to 20)
    int recenter_sample = 20;
    int recenter_probe_R = 35; // count unassigned within this Chebyshev radius (was 25)
    
    // --- logging ---
    int log_interval = 100;  // print progress every N iterations (0 = no logging)
    bool verbose = true;     // enable verbose logging
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

} // namespace alns_detail

// =============================================================================
// Optimizer
// =============================================================================
class GridGrowALNS {
public:
    explicit GridGrowALNS(const Instance& inst, int seed = 1234567)
        : inst_(inst), rng_((unsigned)seed) {
        build_occupied_cells();
        build_territory_map();
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

            // work on a copy of state (cheap: mostly vectors, but still ok per-iter)
            State Cand = S;

            // choose ops
            const int d_op = pick(w_destroy);
            const int r_op = pick(w_repair);

            // destroy
            apply_destroy(Cand, P, d_op);

            // repair
            apply_repair(Cand, P, r_op);

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
    };

    struct State {
        std::vector<Trip> trips; // flattened trips
        // occupied cell assignment: cell_linear -> trip_id, -1 if unassigned
        std::unordered_map<int, int> cell_to_trip;
        std::vector<int> unassigned;
        std::unordered_set<int> unassigned_set;
    };

    const Instance& inst_;
    std::mt19937 rng_;
    std::vector<int> occupied_cells_; // all occupied cells (sparse)
    std::unordered_map<int, std::string> territory_depot_for_cell_; // fixed territories

    // ---------------- precompute ----------------
    void build_occupied_cells() {
        occupied_cells_.clear();
        occupied_cells_.reserve(inst_.id_map.cell_to_customer_indices.size());
        for (const auto& kv : inst_.id_map.cell_to_customer_indices) {
            occupied_cells_.push_back(kv.first);
        }
    }

    void build_territory_map() {
        territory_depot_for_cell_.clear();
        territory_depot_for_cell_.reserve(occupied_cells_.size());
        for (int cell : occupied_cells_) {
            territory_depot_for_cell_[cell] = alns_detail::nearest_depot_id_for_cell(inst_, cell);
        }
    }

    // ---------------- conversions ----------------
    void from_solution(const Solution& sol, State& S) {
        S.trips.clear();
        S.cell_to_trip.clear();
        S.unassigned.clear();
        S.unassigned_set.clear();

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

        // ensure uniqueness: if duplicates existed, keep the first and push others to unassigned
        // (Objective has hard constraint; we enforce here)
        enforce_unique_assignments(S);
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

    double trip_km(const Trip& T) const {
        if (T.center_cell < 0) return 0.0;
        const std::string cid = alns_detail::rep_id_from_cell(inst_, T.center_cell);
        if (cid.empty() || T.depot_id.empty()) return 1e9;
        const double d = inst_.get_dist_km(T.depot_id, cid);
        if (d >= 1e8) return 1e9;
        return 2.0 * d;
    }

    bool can_add_cell(const Trip& T, int cell, const ALNSParams& P) const {
        const double w = alns_detail::cell_w(inst_, cell);
        const double v = alns_detail::cell_v(inst_, cell);
        const double capW = (T.cap_w > 0.0) ? T.cap_w * (1.0 + P.cap_slack) : std::numeric_limits<double>::infinity();
        const double capV = (T.cap_v > 0.0) ? T.cap_v * (1.0 + P.cap_slack) : std::numeric_limits<double>::infinity();
        if (T.load_w + w > capW + 1e-9) return false;
        if (T.load_v + v > capV + 1e-9) return false;
        return true;
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
                if (it == S.cell_to_trip.end() || it->second != tid) {
                    // neighbor is unassigned or belongs to another trip or empty
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
            int seed = pick_unassigned_seed_for_depot(S, T.depot_id);
            if (seed >= 0) {
                add_cell(S, tid, seed);
                T.center_cell = seed;
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
                if (!alns_detail::cell_is_occupied_sparse(inst_, cell)) continue;
                if (S.unassigned_set.count(cell)) ++cnt;
            }
        }
        return cnt;
    }

    int pick_unassigned_seed_for_depot(const State& S, const std::string& depot_id) {
        if (S.unassigned.empty()) return -1;
        // try a few random picks
        for (int t=0;t<60;++t) {
            int cell = S.unassigned[rand_int(0, (int)S.unassigned.size()-1)];
            if (!S.unassigned_set.count(cell)) continue;
            auto it = territory_depot_for_cell_.find(cell);
            if (it != territory_depot_for_cell_.end() && it->second == depot_id) return cell;
        }
        // fallback: linear scan
        for (int cell : S.unassigned) {
            if (!S.unassigned_set.count(cell)) continue;
            auto it = territory_depot_for_cell_.find(cell);
            if (it != territory_depot_for_cell_.end() && it->second == depot_id) return cell;
        }
        return -1;
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
            default: repair_ring_regrow(S, P); break;
        }
    }

    // Ring-boundary growth around center (Chebyshev rings).
    void grow_trip_ring(State& S, const ALNSParams& P, int tid) {
        auto& T = S.trips[tid];
        if (S.unassigned_set.empty()) return;

        // If empty, seed from its territory
        if (T.members.empty()) {
            const int seed = pick_unassigned_seed_for_depot(S, T.depot_id);
            if (seed < 0) return;
            add_cell(S, tid, seed);
            T.center_cell = seed;
        }

        // get center (row,col)
        int cr=0, cc=0;
        inst_.customers.decode_linear(T.center_cell, cr, cc);

        int base_limit = init_detail::spread_ring_limit(T.vtype);
        int Rmax = (int)std::floor((double)base_limit * std::max(0.1, P.ring_limit_mul));
        Rmax = std::max(1, Rmax);

        // scan ring boundaries increasing R
        std::vector<int> cand_cells;
        cand_cells.reserve(512);

        for (int R=1; R<=Rmax; ++R) {
            cand_cells.clear();
            init_detail::for_each_ring_boundary(inst_.customers, cr, cc, R, [&](int r, int c) {
                const int cell = inst_.customers.linear(r, c);
                if (!S.unassigned_set.count(cell)) return;
                if (!alns_detail::cell_is_occupied_sparse(inst_, cell)) return;
                cand_cells.push_back(cell);
            });

            if (cand_cells.empty()) continue;
            std::shuffle(cand_cells.begin(), cand_cells.end(), rng_);
            for (int cell : cand_cells) {
                if (!S.unassigned_set.count(cell)) continue;
                if (!can_add_cell(T, cell, P)) continue;
                add_cell(S, tid, cell);
            }

            // early stop if no more slack
            if (T.cap_w > 0.0 && T.load_w >= T.cap_w * (1.0 + P.cap_slack) - 1e-9) break;
            if (T.cap_v > 0.0 && T.load_v >= T.cap_v * (1.0 + P.cap_slack) - 1e-9) break;
            if (S.unassigned_set.empty()) break;
        }
    }

    // Frontier-based regrow: BFS-like expansion via 8-neighbors from current members.
    void grow_trip_frontier(State& S, const ALNSParams& P, int tid) {
        auto& T = S.trips[tid];
        if (S.unassigned_set.empty()) return;

        if (T.members.empty()) {
            const int seed = pick_unassigned_seed_for_depot(S, T.depot_id);
            if (seed < 0) return;
            add_cell(S, tid, seed);
            T.center_cell = seed;
        }

        std::deque<int> q;
        q.insert(q.end(), T.members.begin(), T.members.end());
        std::unordered_set<int> pushed;
        pushed.reserve(T.members.size()*2 + 32);
        for (int x : T.members) pushed.insert(x);

        int nbr[8], nn=0;
        while (!q.empty() && !S.unassigned_set.empty()) {
            const int u = q.front();
            q.pop_front();
            alns_detail::nbr8_linear(inst_.customers, u, nbr, nn);
            // randomize neighbor order for diversification
            for (int i=nn-1;i>0;--i) {
                int j = rand_int(0, i);
                std::swap(nbr[i], nbr[j]);
            }
            for (int i=0;i<nn;++i) {
                const int v = nbr[i];
                if (!S.unassigned_set.count(v)) continue;
                if (!alns_detail::cell_is_occupied_sparse(inst_, v)) continue;
                if (!can_add_cell(T, v, P)) continue;
                add_cell(S, tid, v);
                if (!pushed.count(v)) {
                    pushed.insert(v);
                    q.push_back(v);
                }
            }
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
        if (!can_add_cell(TB, cell, P)) return;

        // perform move
        remove_cell(S, a, cell);
        add_cell(S, b, cell);

        // keep centers valid
        if (TB.center_cell < 0) TB.center_cell = cell;
        if (TA.center_cell < 0 && !TA.members.empty()) TA.center_cell = TA.members[0];
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
        // feasibility
        // remove first (virtual) then add
        const double wa = alns_detail::cell_w(inst_, ca);
        const double va = alns_detail::cell_v(inst_, ca);
        const double wb = alns_detail::cell_w(inst_, cb);
        const double vb = alns_detail::cell_v(inst_, cb);

        // check caps after swap
        Trip TA2 = TA;
        Trip TB2 = TB;
        TA2.load_w = TA.load_w - wa + wb;
        TA2.load_v = TA.load_v - va + vb;
        TB2.load_w = TB.load_w - wb + wa;
        TB2.load_v = TB.load_v - vb + va;
        const double capWA = (TA.cap_w > 0.0) ? TA.cap_w * (1.0 + P.cap_slack) : std::numeric_limits<double>::infinity();
        const double capVA = (TA.cap_v > 0.0) ? TA.cap_v * (1.0 + P.cap_slack) : std::numeric_limits<double>::infinity();
        const double capWB = (TB.cap_w > 0.0) ? TB.cap_w * (1.0 + P.cap_slack) : std::numeric_limits<double>::infinity();
        const double capVB = (TB.cap_v > 0.0) ? TB.cap_v * (1.0 + P.cap_slack) : std::numeric_limits<double>::infinity();
        if (TA2.load_w > capWA + 1e-9 || TA2.load_v > capVA + 1e-9) return;
        if (TB2.load_w > capWB + 1e-9 || TB2.load_v > capVB + 1e-9) return;

        // swap
        remove_cell(S, a, ca);
        remove_cell(S, b, cb);
        add_cell(S, a, cb);
        add_cell(S, b, ca);
        if (!S.trips[a].members.empty() && (S.trips[a].center_cell < 0 || !contains_member(S.trips[a], S.trips[a].center_cell))) {
            S.trips[a].center_cell = S.trips[a].members[0];
        }
        if (!S.trips[b].members.empty() && (S.trips[b].center_cell < 0 || !contains_member(S.trips[b], S.trips[b].center_cell))) {
            S.trips[b].center_cell = S.trips[b].members[0];
        }
    }
};

} // namespace gridopt
