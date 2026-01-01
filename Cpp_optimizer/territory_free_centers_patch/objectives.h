#pragma once

#include "config.h"
#include "utils.h"

#include <unordered_map>

// Objective implementation (no time).
// Cost = sum_trip( fixed_cost(vehicle) + 2*dist(depot, center)*var_cost*traffic_mult )
// Penalty: unserved
// Hard constraints (if violated => objective = 1e15):
// - K_d: route depot == vehicle.start_depot == vehicle.end_depot
// - per-trip capacity (weight, volume)
// - per-depot storage (weight, optional volume)
// - per-trip spread limit by vehicle type (max dist(center, member) <= limit)
// - per-vehicle total distance budget: sum trips' round-trip dist <= max_dist

struct Objectives {
    static void evaluate(const Instance& inst, Solution& sol) {
        sol.calculate_objective(inst);
    }
};

inline double Solution::calculate_objective(const Instance& inst) {
    // ---- tunable penalties ---- SYNC WITH State::PEN_BASE in alns_vnd.h
    const double INF_OBJ = 1e15;
    const double PENALTY_UNSERVED_BASE = 100000000.0;  // 1e8 to match ALNS optimization
    const double PENALTY_UNSERVED_W = 200.0;
    const double PENALTY_UNSERVED_V = 100.0;

    // reset
    total_cost = fixed_cost = travel_cost = penalty = 0.0;
    penalty_unserved = penalty_maxdist = penalty_veh_cap = penalty_depot_cap = 0.0;
    penalty_cluster_spread = penalty_kd = 0.0;

    // track served, depot loads, vehicle distance used
    std::set<std::string> served;
    std::unordered_map<std::string, double> depot_w, depot_v;
    std::unordered_map<std::string, double> veh_total_km;

    for (const auto& dp : inst.depots) {
        depot_w[dp.first] = 0.0;
        depot_v[dp.first] = 0.0;
    }
    for (const auto& kv : inst.vehicles) {
        veh_total_km[kv.first] = 0.0;
    }

    auto hard_violate = [&](double& bucket, double amount = 1.0) {
        bucket += amount;
    };

    bool infeasible = false;

    // check duplicates
    std::unordered_map<std::string, int> assigned_count;

    for (const auto& kv : routes) {
        const std::string& vid = kv.first;
        auto itV = inst.vehicles.find(vid);
        if (itV == inst.vehicles.end()) continue;
        const Vehicle& veh = itV->second;

        const std::string depot_id = veh.start_depot;
        if (!veh.end_depot.empty() && trim(veh.end_depot) != trim(veh.start_depot)) {
            // vehicle table says it ends elsewhere => violate model assumption
            // infeasible = true; // SOFT CONSTRAINT
            hard_violate(penalty_kd, 1.0);
        }

        for (const Route& r : kv.second) {
            if (r.cluster_customers.empty()) continue;

            // K_d: depot must match start/end depot and route stops if provided
            if (!r.stops.empty()) {
                if (trim(r.stops.front()) != trim(veh.start_depot) || trim(r.stops.back()) != trim(veh.start_depot)) {
                    // infeasible = true; // SOFT CONSTRAINT
                    hard_violate(penalty_kd, 1.0);
                }
            }

            if (r.centroid_id.empty()) {
                // infeasible = true; // SOFT CONSTRAINT
                continue;
            }

            // road restriction (if exists)
            if (!inst.edge_allowed(depot_id, r.centroid_id, veh.type)) {
                // infeasible = true; // SOFT CONSTRAINT
                hard_violate(penalty_kd, 1.0);
                continue;
            }

            // compute trip distance
            double dist = inst.get_dist_km(depot_id, r.centroid_id);
            if (dist >= 1e8) {
                // infeasible = true; // SOFT CONSTRAINT
                hard_violate(penalty_maxdist, 1.0);
                continue;
            }
            double trip_km = 2.0 * dist;

            // add fixed cost per cluster
            fixed_cost += veh.fixed_cost;

            // variable travel cost with traffic multiplier
            double mult = inst.traffic_multiplier(inst.edge_traffic_level(depot_id, r.centroid_id));
            travel_cost += trip_km * veh.var_cost * mult;

            veh_total_km[vid] += trip_km;

            // capacity + served
            double load_w = 0.0, load_v = 0.0;
            for (const auto& cid : r.cluster_customers) {
                auto itC = inst.customers.find(cid);
                if (itC == inst.customers.end()) continue;
                load_w += itC->second.weight;
                load_v += itC->second.volume;
                served.insert(cid);
                assigned_count[cid]++;
            }

            if (load_w > veh.cap_weight + 1e-9 || load_v > veh.cap_volume + 1e-9) {
                // infeasible = true; // SOFT CONSTRAINT
                hard_violate(penalty_veh_cap, (load_w - veh.cap_weight) + (load_v - veh.cap_volume));
            }

            // depot storage
            depot_w[depot_id] += load_w;
            depot_v[depot_id] += load_v;

            // spread (max distance from center to members)
            double lim = spread_limit_km(veh.type);
            double max_r = 0.0;
            const auto& cen = inst.customers.at(r.centroid_id);
            for (const auto& cid : r.cluster_customers) {
                const auto& c = inst.customers.at(cid);
                double d = Instance::haversine_km(cen.lat, cen.lon, c.lat, c.lon);
                if (d > max_r) max_r = d;
                if (d > lim + 1e-9) {
                    // infeasible = true; // SOFT CONSTRAINT
                }
            }
            if (max_r > lim + 1e-9) {
                hard_violate(penalty_cluster_spread, max_r - lim);
            }
        }
    }

    // per-vehicle total distance
    for (const auto& kv : veh_total_km) {
        const std::string& vid = kv.first;
        auto itV = inst.vehicles.find(vid);
        if (itV == inst.vehicles.end()) continue;
        const Vehicle& veh = itV->second;
        if (kv.second > veh.max_dist + 1e-9) {
            // infeasible = true; // SOFT CONSTRAINT
            hard_violate(penalty_maxdist, kv.second - veh.max_dist);
        }
    }

    // depot storage capacity
    for (const auto& kv : inst.depots) {
        const std::string& did = kv.first;
        const Depot& d = kv.second;
        if (d.cap_weight_storage > 0.0 && depot_w[did] > d.cap_weight_storage + 1e-9) {
            // infeasible = true; // SOFT CONSTRAINT
            hard_violate(penalty_depot_cap, depot_w[did] - d.cap_weight_storage);
        }
        if (d.cap_volume_storage > 0.0 && depot_v[did] > d.cap_volume_storage + 1e-9) {
            // infeasible = true; // SOFT CONSTRAINT
            hard_violate(penalty_depot_cap, depot_v[did] - d.cap_volume_storage);
        }
    }

    // customer assigned > 1 clusters
    for (const auto& kv : assigned_count) {
        if (kv.second > 1) {
            // infeasible = true; // SOFT CONSTRAINT
        }
    }

    // unserved penalty
    unassigned_customers.clear();
    for (const auto& kv : inst.customers) {
        if (!served.count(kv.first)) {
            unassigned_customers.insert(kv.first);
            const auto& c = kv.second;
            penalty_unserved += PENALTY_UNSERVED_BASE
                             + PENALTY_UNSERVED_W * c.weight
                             + PENALTY_UNSERVED_V * c.volume;
        }
    }

    // combine
    penalty = penalty_unserved;
    total_cost = fixed_cost + travel_cost + penalty;

    if (infeasible) {
        objective = INF_OBJ;
        return objective;
    }

    objective = total_cost;
    return objective;
}
