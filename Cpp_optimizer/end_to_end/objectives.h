#pragma once

#include "config.h"

#include <unordered_map>
#include <unordered_set>
#include <limits>

// ============================================================================
// Objective for "cluster-trip last-mile" (GRID-TENSOR edition)
//
// REQUIREMENT:
// - Travel cost uses ONLY Distance_km from the roads table (Instance::get_dist_km).
// - No geometric distance computations (no haversine, no euclid, no grid-size conversion).
// - Vehicle movement is D -> Center -> D, so distance queried is depot -> center.
//
// Total cost:
//   sum_trip( veh.fixed_cost + 2 * Dist_km(depot, center_customer_id) * veh.var_cost )
//
// Soft constraints (penalties):
//   - Unserved occupied cells
//   - Vehicle capacity (weight/volume) per trip
//   - Depot storage capacity (weight)
//   - Vehicle max distance budget (sum of round-trips)
//   - K_d (vehicle must start & end at its depot)
//   - Duplicate assignment (same cell assigned to >=2 trips)
//
// NOTE:
// - "center_customer_id" is taken from the representative customer index stored
//   in the tensor cell (CUST_CH_IDX) via Instance::id_map.
// ============================================================================

namespace gridopt {

// -------------------------- penalty weights --------------------------
// Tune later; they keep the same constraints, only change search landscape.
#ifndef GRIDOPT_PEN_UNSERVED_BASE
#define GRIDOPT_PEN_UNSERVED_BASE 1e8
#endif
#ifndef GRIDOPT_PEN_UNSERVED_W
#define GRIDOPT_PEN_UNSERVED_W 200.0
#endif
#ifndef GRIDOPT_PEN_UNSERVED_V
#define GRIDOPT_PEN_UNSERVED_V 100.0
#endif

#ifndef GRIDOPT_PEN_VEH_CAP
#define GRIDOPT_PEN_VEH_CAP 1e5
#endif
#ifndef GRIDOPT_PEN_DEPOT_CAP
#define GRIDOPT_PEN_DEPOT_CAP 1e4
#endif
#ifndef GRIDOPT_PEN_MAXDIST
#define GRIDOPT_PEN_MAXDIST 1e4
#endif
#ifndef GRIDOPT_PEN_KD
#define GRIDOPT_PEN_KD 1e15  // HARD CONSTRAINT: vehicle must return to start depot
#endif
#ifndef GRIDOPT_PEN_DUPLICATE_CELL
#define GRIDOPT_PEN_DUPLICATE_CELL 1e15  // HARD CONSTRAINT: cell cannot be assigned to multiple trips
#endif

// Get representative customer_id for a grid cell.
// Convention: tensor idx channel stores a customer_idx (int) pointing into CustomerIdMap.
inline bool rep_customer_id_from_cell(const Instance& inst, int cell_linear, std::string& out_customer_id) {
    int r = 0, c = 0;
    inst.customers.decode_linear(cell_linear, r, c);
    if (!inst.customers.in_bounds(r, c)) return false;
    if (!inst.customers.occupied(r, c)) return false;

    int idx = inst.customers.idx_int(r, c);
    if (idx < 0) return false;
    if (!inst.id_map.has_idx(idx)) return false;

    const auto& row = inst.id_map.rows[(size_t)idx];
    if (row.customer_id.empty()) return false;

    out_customer_id = row.customer_id;
    return true;
}

// Compute load for a list of member cells using tensor (aggregated weight/volume per cell).
inline void compute_route_load_tensor(const Instance& inst,
                                     const std::vector<int>& member_cells,
                                     double& out_w,
                                     double& out_v) {
    out_w = 0.0;
    out_v = 0.0;
    for (int cell : member_cells) {
        int r = 0, c = 0;
        inst.customers.decode_linear(cell, r, c);
        if (!inst.customers.in_bounds(r, c)) continue;
        if (!inst.customers.occupied(r, c)) continue;
        out_w += (double)inst.customers.at(r, c, CUST_CH_W);
        out_v += (double)inst.customers.at(r, c, CUST_CH_VOL);
    }
}

struct Objectives {
    static void evaluate(const Instance& inst, Solution& sol) {
        // reset outputs
        sol.fixed_cost = 0.0;
        sol.travel_cost = 0.0;
        sol.penalty = 0.0;
        sol.objective = 1e18;  // default = infeasible

        // ========== PHASE 1: HARD CONSTRAINTS (early reject) ==========
        
        // Track assigned cells (to detect duplicates)
        std::unordered_set<int> assigned_cells;
        
        for (const auto& kv : sol.routes) {
            const std::string& vid = kv.first;
            auto itV = inst.vehicles.find(vid);
            if (itV == inst.vehicles.end()) continue;
            const Vehicle& veh = itV->second;

            // HARD CONSTRAINT 1: K_d - vehicle must return to start depot
            if (!veh.end_depot.empty() && veh.end_depot != veh.start_depot) {
                // INFEASIBLE: vehicle does not return to start depot
                sol.objective = 1e18;
                return;  // Early reject
            }

            // HARD CONSTRAINT 2: Duplicate cell assignment
            for (const Route& rt : kv.second) {
                for (int cell : rt.member_cells_linear) {
                    if (assigned_cells.count(cell) > 0) {
                        // INFEASIBLE: cell already assigned to another trip
                        sol.objective = 1e18;
                        return;  // Early reject
                    }
                    assigned_cells.insert(cell);
                }
            }
        }

        // ========== PHASE 2: COMPUTE COSTS & SOFT CONSTRAINTS ==========
        
        std::unordered_map<std::string, double> depot_w;
        std::unordered_map<std::string, double> veh_total_km;

        for (const auto& d : inst.depots) depot_w[d.id] = 0.0;
        for (const auto& kv : inst.vehicles) veh_total_km[kv.first] = 0.0;

        double pen_unserved = 0.0;
        double pen_veh_cap = 0.0;
        double pen_depot_cap = 0.0;
        double pen_maxdist = 0.0;

        // -------- evaluate each vehicle routes --------
        for (const auto& kv : sol.routes) {
            const std::string& vid = kv.first;
            auto itV = inst.vehicles.find(vid);
            if (itV == inst.vehicles.end()) continue;
            const Vehicle& veh = itV->second;

            const std::string depot_id = veh.start_depot;

            const auto& vecR = kv.second;
            for (const Route& rt : vecR) {
                if (rt.member_cells_linear.empty()) continue;

                // center customer id for road lookup
                std::string center_cust_id;
                if (!rep_customer_id_from_cell(inst, rt.center_cell_linear, center_cust_id)) {
                    // invalid center -> heavy penalty
                    pen_maxdist += GRIDOPT_PEN_MAXDIST * 100.0;
                    continue;
                }

                // road distance depot -> center (from roads Distance_km)
                const double d_km = inst.get_dist_km(depot_id, center_cust_id);
                if (d_km >= 1e8) {
                    // road missing in table
                    pen_maxdist += GRIDOPT_PEN_MAXDIST * 100.0;
                    continue;
                }
                const double trip_km = 2.0 * d_km;

                // costs
                sol.fixed_cost += veh.fixed_cost;
                sol.travel_cost += trip_km * veh.var_cost;
                veh_total_km[vid] += trip_km;

                // route load (from tensor)
                double load_w = 0.0, load_v = 0.0;
                compute_route_load_tensor(inst, rt.member_cells_linear, load_w, load_v);

                // capacity constraints (soft)
                if (veh.cap_weight > 0.0 && load_w > veh.cap_weight + 1e-9) {
                    pen_veh_cap += (load_w - veh.cap_weight) * GRIDOPT_PEN_VEH_CAP;
                }
                if (veh.cap_volume > 0.0 && load_v > veh.cap_volume + 1e-9) {
                    pen_veh_cap += (load_v - veh.cap_volume) * GRIDOPT_PEN_VEH_CAP;
                }

                // depot storage (weight)
                depot_w[depot_id] += load_w;
            }
        }

        // -------- per-vehicle max distance budget --------
        for (const auto& kv : veh_total_km) {
            const std::string& vid = kv.first;
            auto itV = inst.vehicles.find(vid);
            if (itV == inst.vehicles.end()) continue;
            const Vehicle& veh = itV->second;
            if (veh.max_dist > 0.0 && kv.second > veh.max_dist + 1e-9) {
                pen_maxdist += (kv.second - veh.max_dist) * GRIDOPT_PEN_MAXDIST;
            }
        }

        // -------- depot storage capacity (weight) --------
        for (const auto& d : inst.depots) {
            if (d.cap_weight_storage > 0.0) {
                const double used = depot_w[d.id];
                if (used > d.cap_weight_storage + 1e-9) {
                    pen_depot_cap += (used - d.cap_weight_storage) * GRIDOPT_PEN_DEPOT_CAP;
                }
            }
        }

        // -------- unserved occupied cells (sparse scan) --------
        sol.unassigned_cells.clear();

        // occupied cells are the keys of id_map.cell_to_customer_indices (sparse)
        for (const auto& kv : inst.id_map.cell_to_customer_indices) {
            const int cell = kv.first;
            if (assigned_cells.count(cell) > 0) continue;  // Already assigned (should not happen after Phase 1)

            // unserved
            sol.unassigned_cells.insert(cell);

            int r = 0, c = 0;
            inst.customers.decode_linear(cell, r, c);
            if (!inst.customers.in_bounds(r, c) || !inst.customers.occupied(r, c)) continue;

            const double w = (double)inst.customers.at(r, c, CUST_CH_W);
            const double v = (double)inst.customers.at(r, c, CUST_CH_VOL);

            pen_unserved += GRIDOPT_PEN_UNSERVED_BASE + GRIDOPT_PEN_UNSERVED_W * w + GRIDOPT_PEN_UNSERVED_V * v;
        }

        sol.penalty = pen_unserved + pen_veh_cap + pen_depot_cap + pen_maxdist;  // No pen_kd/pen_duplicate - they are hard constraints
        sol.objective = sol.fixed_cost + sol.travel_cost + sol.penalty;
    }
};

} // namespace gridopt
