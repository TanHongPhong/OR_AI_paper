#pragma once

// =============================================================
//  initialization.h (GRID-TENSOR edition)
//
//  Goal
//  -----
//  Create an initial clustering solution for the "cluster-trip last-mile" model
//  using the NEW data layout:
//    - Customers: dense grid tensor [H,W,4] (weight, volume, priority, rep_idx)
//    - CustomerIdMap: maps rep_idx -> customer_id (for roads distance lookup)
//    - Depots: (depot_id,row,col,capacity_storage)
//    - Vehicles: (vehicle_id,type,cap, fixed/var cost, max_dist, start_depot)
//    - Roads: distance only via Instance::get_dist_km(from,to)
//
//  Required behavior (per user request)
//  ------------------------------------
//  1) You already have a depot territory assignment logic.
//     In this header we provide a default: nearest depot on the grid.
//     You can replace `assign_territories_nearest()` with your own.
//
//  2) Seeding: random seeds inside each depot's territory.
//     (Seed must belong to its territory; growth may expand outside territory.)
//
//  3) Growth: same constraints as before (cap, depot storage, max_dist,
//     and optional distance-quartile vehicle-class requirement), but implemented
//     on the NEW grid tensor.
//
//  4) Ring growth uses "ring boundary" only (Chebyshev ring):
//     for R = 1..Rmax, scan ONLY cells where max(|dr|,|dc|) == R.
//     This is much faster than scanning the full (2R+1)^2 square.
//
//  NOTE
//  ----
//  Objective evaluation is not performed here (keep init independent).
//  This header only builds Solution.routes + Solution.unassigned_cells and
//  fills Instance.id_map rows (cluster/vehicle_id/depot_id) for visualization.
// =============================================================

#include "config.h"

#include <unordered_map>
#include <unordered_set>
#include <random>
#include <iostream>

namespace gridopt {

namespace init_detail {

// ----------------- small helpers -----------------

inline int type_rank(VehicleType t) {
    switch (t) {
        case VehicleType::BIKE:        return 0;
        case VehicleType::MOTORBIKE:   return 1;
        case VehicleType::CARGO_TRIKE: return 2;
        case VehicleType::EV_VAN:      return 3;
        case VehicleType::VAN:         return 4;
        case VehicleType::UNKNOWN:     return 4; // treat unknown as large to avoid over-filtering
    }
    return 4;
}

inline int spread_ring_limit(VehicleType t) {
    // Tune these numbers for your grid density.
    // They are "ring counts" (Chebyshev distance in cells), not km.
    // Increased 8x to allow maximum cluster growth
    switch (t) {
        case VehicleType::BIKE:        return 200;  // was 25
        case VehicleType::MOTORBIKE:   return 320;  // was 40
        case VehicleType::CARGO_TRIKE: return 360;  // was 45
        case VehicleType::EV_VAN:      return 520;  // was 65
        case VehicleType::VAN:         return 600;  // was 75
        case VehicleType::UNKNOWN:     return 600;  // was 75
    }
    return 600;
}

inline std::string rep_customer_id_from_cell(const Instance& inst, int cell_linear) {
    int r=0,c=0;
    inst.customers.decode_linear(cell_linear, r, c);
    if (!inst.customers.in_bounds(r,c) || !inst.customers.occupied(r,c)) return "";
    int rep_idx = inst.customers.idx_int(r,c);
    if (rep_idx < 0) return "";
    if (rep_idx >= (int)inst.id_map.rows.size()) return "";
    const auto& row = inst.id_map.rows[(size_t)rep_idx];
    return row.customer_id;
}

inline bool cell_has_any_customer(const Instance& inst, int cell_linear) {
    // Prefer the sparse map (loaded from customers_sparse_grid.csv)
    auto it = inst.id_map.cell_to_customer_indices.find(cell_linear);
    return it != inst.id_map.cell_to_customer_indices.end() && !it->second.empty();
}

inline double cell_weight(const Instance& inst, int cell_linear) {
    int r=0,c=0;
    inst.customers.decode_linear(cell_linear, r, c);
    return (double)inst.customers.at(r,c,CUST_CH_W);
}

inline double cell_volume(const Instance& inst, int cell_linear) {
    int r=0,c=0;
    inst.customers.decode_linear(cell_linear, r, c);
    return (double)inst.customers.at(r,c,CUST_CH_VOL);
}

inline double cell_priority(const Instance& inst, int cell_linear) {
    int r=0,c=0;
    inst.customers.decode_linear(cell_linear, r, c);
    return (double)inst.customers.at(r,c,CUST_CH_PRI);
}

// ----------------- territory assignment -----------------

inline std::string nearest_depot_id(const Instance& inst, int r, int c) {
    double best = 1e100;
    std::string best_id;
    for (const auto& d : inst.depots) {
        double dr = (double)r - (double)d.row;
        double dc = (double)c - (double)d.col;
        double d2 = dr*dr + dc*dc;
        if (d2 < best) { best = d2; best_id = d.id; }
    }
    return best_id;
}

// Default territory logic: nearest depot on (row,col).
// Output:
//  - cell_to_depot[cell_linear] = depot_id
//  - depot_to_cells[depot_id] = list of cells in its territory
inline void assign_territories_nearest(
    const Instance& inst,
    const std::vector<int>& occupied_cells,
    std::unordered_map<int, std::string>& cell_to_depot,
    std::unordered_map<std::string, std::vector<int>>& depot_to_cells)
{
    cell_to_depot.clear();
    depot_to_cells.clear();
    cell_to_depot.reserve(occupied_cells.size());

    for (int cell : occupied_cells) {
        int r=0,c=0;
        inst.customers.decode_linear(cell, r, c);
        std::string did = nearest_depot_id(inst, r, c);
        cell_to_depot[cell] = did;
        depot_to_cells[did].push_back(cell);
    }
}

// ----------------- quartile vehicle-class requirement -----------------

struct DepotQuartiles {
    double q25 = 0.0;
    double q50 = 0.0;
    double q75 = 0.0;
    bool ok = false;
};

inline DepotQuartiles compute_depot_quartiles_by_roads(
    const Instance& inst,
    const std::string& depot_id,
    const std::vector<int>& territory_cells)
{
    std::vector<double> ds;
    ds.reserve(territory_cells.size());
    for (int cell : territory_cells) {
        std::string cid = rep_customer_id_from_cell(inst, cell);
        if (cid.empty()) continue;
        double d = inst.get_dist_km(depot_id, cid);
        if (d >= 1e8) continue;
        ds.push_back(d);
    }
    if (ds.empty()) return {};
    std::sort(ds.begin(), ds.end());
    auto q = [&](double frac) {
        size_t idx = (size_t)std::floor(frac * (ds.size() - 1));
        return ds[idx];
    };
    DepotQuartiles out;
    out.q25 = q(0.25);
    out.q50 = q(0.50);
    out.q75 = q(0.75);
    out.ok = true;
    return out;
}

inline VehicleType required_type_from_dist(double d, const DepotQuartiles& qq) {
    // Map distance quartiles -> required vehicle class
    if (!qq.ok) return VehicleType::BIKE; // if no stats, do not restrict
    if (d <= qq.q25) return VehicleType::BIKE;
    if (d <= qq.q50) return VehicleType::MOTORBIKE;
    if (d <= qq.q75) return VehicleType::EV_VAN;
    return VehicleType::VAN;
}

// ----------------- ring boundary iterator -----------------

template <class Fn>
inline void for_each_ring_boundary(
    const CustomerTensor& tensor,
    int cr, int cc,
    int R,
    Fn&& fn)
{
    if (R <= 0) return;

    const int r0 = cr - R;
    const int r1 = cr + R;
    const int c0 = cc - R;
    const int c1 = cc + R;

    // Top edge (r0, c0..c1)
    if (r0 >= 0 && r0 < tensor.H) {
        for (int c = c0; c <= c1; ++c) {
            if (c < 0 || c >= tensor.W) continue;
            fn(r0, c);
        }
    }
    // Bottom edge (r1, c0..c1)
    if (r1 >= 0 && r1 < tensor.H && r1 != r0) {
        for (int c = c0; c <= c1; ++c) {
            if (c < 0 || c >= tensor.W) continue;
            fn(r1, c);
        }
    }
    // Left edge (r0+1..r1-1, c0)
    if (c0 >= 0 && c0 < tensor.W) {
        for (int r = r0 + 1; r <= r1 - 1; ++r) {
            if (r < 0 || r >= tensor.H) continue;
            fn(r, c0);
        }
    }
    // Right edge (r0+1..r1-1, c1)
    if (c1 >= 0 && c1 < tensor.W && c1 != c0) {
        for (int r = r0 + 1; r <= r1 - 1; ++r) {
            if (r < 0 || r >= tensor.H) continue;
            fn(r, c1);
        }
    }
}

// Compute "center cell" as the member closest to mean(row,col).
inline int update_center_by_mean_grid(const CustomerTensor& tensor, const std::vector<int>& members) {
    if (members.empty()) return -1;
    double mr = 0.0, mc = 0.0;
    for (int cell : members) {
        int r=0,c=0;
        tensor.decode_linear(cell, r, c);
        mr += (double)r;
        mc += (double)c;
    }
    mr /= (double)members.size();
    mc /= (double)members.size();

    int best_cell = members.front();
    double best = 1e100;
    for (int cell : members) {
        int r=0,c=0;
        tensor.decode_linear(cell, r, c);
        double dr = (double)r - mr;
        double dc = (double)c - mc;
        double d2 = dr*dr + dc*dc;
        if (d2 < best) { best = d2; best_cell = cell; }
    }
    return best_cell;
}

} // namespace init_detail

// =============================================================
// Initializer
// =============================================================

class Initializer {
public:
    // Builds initial solution and updates inst.id_map rows:
    //   - cluster (int)
    //   - vehicle_id
    //   - depot_id
    static Solution create_initial_solution(Instance& inst, int seed = 42) {
        using namespace init_detail;

        Solution sol;
        sol.routes.clear();
        sol.unassigned_cells.clear();
        sol.objective = 1e18;
        sol.fixed_cost = sol.travel_cost = sol.penalty = 0.0;

        std::mt19937 rng((unsigned)seed);

        // 0) Collect occupied cells efficiently from the sparse link map
        std::vector<int> occupied_cells;
        occupied_cells.reserve(inst.id_map.cell_to_customer_indices.size());
        for (const auto& kv : inst.id_map.cell_to_customer_indices) {
            int cell = kv.first;
            // Ensure tensor marks it occupied (defensive)
            int r=0,c=0;
            inst.customers.decode_linear(cell, r, c);
            if (!inst.customers.in_bounds(r,c)) continue;
            if (!inst.customers.occupied(r,c)) continue;
            occupied_cells.push_back(cell);
            sol.unassigned_cells.insert(cell);
        }

        // Reset id_map assignment fields
        for (auto &row : inst.id_map.rows) {
            row.cluster = -1;
            row.vehicle_id.clear();
            row.depot_id.clear();
        }

        // 1) Default territory assignment (replaceable)
        std::unordered_map<int, std::string> cell_to_depot;
        std::unordered_map<std::string, std::vector<int>> depot_to_cells;
        assign_territories_nearest(inst, occupied_cells, cell_to_depot, depot_to_cells);

        // 2) Depot remaining storage (weight)
        std::unordered_map<std::string, double> depot_rem_w;
        depot_rem_w.reserve(inst.depots.size());
        for (const auto& d : inst.depots) {
            depot_rem_w[d.id] = d.cap_weight_storage;
        }

        // 3) Vehicles grouped by depot
        std::unordered_map<std::string, std::vector<std::string>> depot_vehicles;
        depot_vehicles.reserve(inst.depots.size());
        for (const auto& kv : inst.vehicles) {
            const auto& v = kv.second;
            depot_vehicles[v.start_depot].push_back(v.id);
        }
        for (auto& kv : depot_vehicles) {
            auto& vec = kv.second;
            std::sort(vec.begin(), vec.end(), [&](const std::string& a, const std::string& b) {
                const auto& va = inst.vehicles.at(a);
                const auto& vb = inst.vehicles.at(b);
                if (va.cap_weight != vb.cap_weight) return va.cap_weight > vb.cap_weight;
                return va.cap_volume > vb.cap_volume;
            });
        }

        // 4) Track used km per vehicle
        std::unordered_map<std::string, double> veh_used_km;
        veh_used_km.reserve(inst.vehicles.size());
        for (const auto& kv : inst.vehicles) veh_used_km[kv.first] = 0.0;

        // 5) Precompute quartiles per depot territory for class requirement
        std::unordered_map<std::string, DepotQuartiles> depot_q;
        depot_q.reserve(inst.depots.size());
        for (const auto& d : inst.depots) {
            auto it = depot_to_cells.find(d.id);
            if (it == depot_to_cells.end()) continue;
            depot_q[d.id] = compute_depot_quartiles_by_roads(inst, d.id, it->second);
        }

        // 6) Main loop: per depot, random seeding within territory, ring-boundary growth.
        int next_cluster_id = 1;
        const int MAX_PASSES = 100;
        const int MAX_TRIPS_PER_VEH_SOFT = 1000000;

        for (const auto& d : inst.depots) {
            const std::string did = d.id;
            auto itCells = depot_to_cells.find(did);
            if (itCells == depot_to_cells.end() || itCells->second.empty()) continue;
            if (depot_vehicles[did].empty()) continue;

            auto& territory_cells = itCells->second;

            bool progressed = true;
            int pass = 0;
            while (progressed && pass < MAX_PASSES) {
                progressed = false;
                ++pass;

                // Use vehicle list sorted by capacity DESC (large vehicles first, small vehicles last)
                // This ensures larger vehicles get priority to grow clusters first
                const auto& veh_list = depot_vehicles[did];

                for (const auto& vid : veh_list) {
                    const Vehicle& veh = inst.vehicles.at(vid);
                    if (veh.start_depot != did) continue;
                    if ((int)sol.routes[vid].size() >= MAX_TRIPS_PER_VEH_SOFT) continue;

                    bool vehicle_active = true;
                    while (vehicle_active) {
                        vehicle_active = false;

                        const double rem_km = veh.max_dist - veh_used_km[vid];
                        if (rem_km <= 1e-9) break;
                        if (depot_rem_w[did] <= 1e-9) break;

                        // ---- choose random seed inside territory (must be in territory) ----
                        std::vector<int> seed_cands;
                        seed_cands.reserve(256);
                        for (int cell : territory_cells) {
                            if (sol.unassigned_cells.count(cell)) seed_cands.push_back(cell);
                        }
                        if (seed_cands.empty()) break;
                        std::shuffle(seed_cands.begin(), seed_cands.end(), rng);

                        int seed_cell = -1;
                        double seed_trip_km = 0.0;
                        for (int cell : seed_cands) {
                            // basic demand
                            const double w = cell_weight(inst, cell);
                            const double v = cell_volume(inst, cell);
                            if (w <= 0.0 && v <= 0.0) continue;
                            if (w > veh.cap_weight + 1e-9) continue;
                            if (v > veh.cap_volume + 1e-9) continue;
                            if (depot_rem_w[did] < w - 1e-9) continue;

                            std::string rep_cid = rep_customer_id_from_cell(inst, cell);
                            if (rep_cid.empty()) continue;
                            double dkm = inst.get_dist_km(did, rep_cid);
                            if (dkm >= 1e8) continue;

                            // optional class requirement by quartiles
                            VehicleType req = required_type_from_dist(dkm, depot_q[did]);
                            if (type_rank(veh.type) < type_rank(req)) continue;

                            double trip_km = 2.0 * dkm;
                            if (trip_km > rem_km + 1e-9) continue;

                            seed_cell = cell;
                            seed_trip_km = trip_km;
                            break;
                        }
                        if (seed_cell < 0) break;

                        // ---- create route from seed ----
                        Route r;
                        r.vehicle_id = vid;
                        r.center_cell_linear = seed_cell;
                        r.member_cells_linear.clear();
                        r.member_cells_linear.push_back(seed_cell);
                        r.load_w = cell_weight(inst, seed_cell);
                        r.load_v = cell_volume(inst, seed_cell);
                        r.distance_km = seed_trip_km;

                        const double used_base = veh_used_km[vid];
                        // reserve seed
                        sol.unassigned_cells.erase(seed_cell);

                        // ---- grow by ring boundary around the *current center* ----
                        int ring_max = spread_ring_limit(veh.type);
                        while (true) {
                            int cr=0, cc=0;
                            inst.customers.decode_linear(r.center_cell_linear, cr, cc);

                            int best_cell = -1;
                            int best_new_center = -1;
                            double best_new_trip_km = 0.0;
                            double best_score = -1e100;

                            // Expand R=1..ring_max; when accept one, restart from R=1 around new center.
                            bool accepted_any = false;
                            for (int R = 1; R <= ring_max; ++R) {
                                for_each_ring_boundary(inst.customers, cr, cc, R, [&](int rr, int cc2) {
                                    if (!inst.customers.occupied(rr, cc2)) return;
                                    const int cell = inst.customers.linear(rr, cc2);
                                    if (!sol.unassigned_cells.count(cell)) return;
                                    if (!cell_has_any_customer(inst, cell)) return;

                                    const double w = (double)inst.customers.at(rr, cc2, CUST_CH_W);
                                    const double v = (double)inst.customers.at(rr, cc2, CUST_CH_VOL);
                                    if (w <= 0.0 && v <= 0.0) return;
                                    if (r.load_w + w > veh.cap_weight + 1e-9) return;
                                    if (r.load_v + v > veh.cap_volume + 1e-9) return;
                                    if (depot_rem_w[did] < (r.load_w + w) - 1e-9) return;

                                    // new center by mean(row,col)
                                    std::vector<int> tmp = r.member_cells_linear;
                                    tmp.push_back(cell);
                                    int new_center_cell = update_center_by_mean_grid(inst.customers, tmp);
                                    if (new_center_cell < 0) return;

                                    std::string rep_cid = rep_customer_id_from_cell(inst, new_center_cell);
                                    if (rep_cid.empty()) return;
                                    double dkm = inst.get_dist_km(did, rep_cid);
                                    if (dkm >= 1e8) return;

                                    // optional class requirement on the CENTER distance
                                    VehicleType req = required_type_from_dist(dkm, depot_q[did]);
                                    if (type_rank(veh.type) < type_rank(req)) return;

                                    const double new_trip_km = 2.0 * dkm;
                                    if (used_base + new_trip_km > veh.max_dist + 1e-9) return;

                                    // score: heavy first, then priority, then closer ring
                                    double score = 0.0;
                                    score += 1000.0 * w;
                                    score += 10.0 * cell_priority(inst, cell);
                                    score -= 0.1 * (double)R;

                                    if (score > best_score) {
                                        best_score = score;
                                        best_cell = cell;
                                        best_new_center = new_center_cell;
                                        best_new_trip_km = new_trip_km;
                                    }
                                });

                                if (best_cell >= 0) {
                                    // accept best in this expansion (not necessarily smallest R)
                                    accepted_any = true;
                                    break;
                                }
                            }

                            if (!accepted_any || best_cell < 0) break;

                            // commit best insertion
                            r.member_cells_linear.push_back(best_cell);
                            r.load_w += cell_weight(inst, best_cell);
                            r.load_v += cell_volume(inst, best_cell);
                            r.center_cell_linear = best_new_center;
                            r.distance_km = best_new_trip_km;
                            sol.unassigned_cells.erase(best_cell);
                        }

                        // finalize usage + depot storage
                        veh_used_km[vid] = used_base + r.distance_km;
                        depot_rem_w[did] -= r.load_w;

                        // write back to id_map (for visualization / optimizer bridge)
                        const int cluster_id = next_cluster_id++;
                        for (int cell : r.member_cells_linear) {
                            auto it = inst.id_map.cell_to_customer_indices.find(cell);
                            if (it == inst.id_map.cell_to_customer_indices.end()) continue;
                            for (int cust_idx : it->second) {
                                if (cust_idx < 0 || cust_idx >= (int)inst.id_map.rows.size()) continue;
                                auto &row = inst.id_map.rows[(size_t)cust_idx];
                                row.cluster = cluster_id;
                                row.vehicle_id = vid;
                                row.depot_id = did;
                            }
                        }

                        // push to solution
                        sol.routes[vid].push_back(r);

                        progressed = true;
                        vehicle_active = true;
                    }
                }
            }
        }

        return sol;
    }
};

} // namespace gridopt
