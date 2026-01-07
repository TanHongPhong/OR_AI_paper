#pragma once

#include "config.h"
#include "utils.h"

#include <unordered_map>
#include <unordered_set>
#include <map>
#include <set>
#include <random>
#include <limits>

// ===============================
// Initial solution (seed-growing) for cluster-trip model
// - Works on lat/lon by using planar km coords (x_km,y_km) computed by DataLoader.
// - Territory: if exactly 4 depots => plus-sign split, else Voronoi nearest depot.
// - For each depot: run multi-pass over vehicles (sorted by capacity asc)
//   Each vehicle tries to open a new cluster (seed) from remaining customers,
//   then grow by kNN-in-radius (spread limit by vehicle type) while respecting:
//     * vehicle cap weight/volume
//     * vehicle remaining max_dist (total daily distance budget)
//     * depot storage capacity
// - If a cluster ends with size < MIN_CLUSTER_SIZE, it's rolled back.
// ===============================

namespace init_detail {

struct GridCoord {
    int row, col;
    
    bool operator<(const GridCoord& o) const {
        if (row != o.row) return row < o.row;
        return col < o.col;
    }
    
    bool operator==(const GridCoord& o) const {
        return row == o.row && col == o.col;
    }
};

// Simple grid index for sparse grids.
// - You can query customers at a cell.
// - You can expand neighborhood in "rings" around a cell (R=1..Rmax).
class GridIndex {
public:
    void build(const Instance& inst, const std::vector<std::string>& customer_ids) {
        grid_map.clear();
        for (const auto& cid : customer_ids) {
            const auto& c = inst.customers.at(cid);
            GridCoord gc{c.row, c.col};
            grid_map[gc].push_back(cid);
        }
    }
    
    // Returns a reference (no copy). If empty, returns a shared static empty vector.
    const std::vector<std::string>& customers_at(const GridCoord& gc) const {
        static const std::vector<std::string> empty;
        auto it = grid_map.find(gc);
        if (it == grid_map.end()) return empty;
        return it->second;
    }

    // Square neighborhood of radius R (in grid cells). When include_center=false, excludes (0,0).
    // NOTE: This is intentionally "square" (Chebyshev distance). For a true ring, filter by max(|dr|,|dc|)==R.
    std::vector<GridCoord> get_neighbors(const GridCoord& center, int R = 1, bool include_center = true) const {
        std::vector<GridCoord> res;
        if (R < 0) return res;
        res.reserve((2*R+1)*(2*R+1));
        for (int dr = -R; dr <= R; ++dr) {
            for (int dc = -R; dc <= R; ++dc) {
                if (!include_center && dr == 0 && dc == 0) continue;
                res.push_back({center.row + dr, center.col + dc});
            }
        }
        return res;
    }
    
private:
    std::map<GridCoord, std::vector<std::string>> grid_map;
};

inline double max_radius_km_to_center(const Instance& inst,
                                     const std::string& center_cid,
                                     const std::vector<std::string>& members) {
    if (members.empty() || center_cid.empty()) return 0.0;
    const auto& cen = inst.customers.at(center_cid);
    double maxr = 0.0;
    for (const auto& cid : members) {
        const auto& c = inst.customers.at(cid);
        double d = Instance::haversine_km(cen.lat, cen.lon, c.lat, c.lon);
        if (d > maxr) maxr = d;
    }
    return maxr;
}

inline std::string nearest_depot(const Instance& inst, const Customer& c) {
    double best = 1e18;
    std::string best_id;
    for (const auto& kv : inst.depots) {
        const auto& d = kv.second;
        double dd = std::hypot(c.x_km - d.x_km, c.y_km - d.y_km);
        if (dd < best) { best = dd; best_id = d.id; }
    }
    return best_id;
}

inline void assign_territory(Instance& inst) {
    // Balanced territory assignment for N depots
    // Goal: Each depot gets approximately equal number of customers
    
    std::vector<std::string> depot_ids;
    for (const auto& kv : inst.depots) {
        depot_ids.push_back(kv.first);
    }
    
    int num_depots = depot_ids.size();
    if (num_depots == 0) return;
    
    // Simple balanced assignment: sort customers spatially, then round-robin
    std::vector<std::pair<double, std::string>> sorted_customers;
    for (const auto& kv : inst.customers) {
        // Use lat+lon as simple spatial key
        double key = kv.second.lat * 1000.0 + kv.second.lon;
        sorted_customers.push_back({key, kv.first});
    }
    std::sort(sorted_customers.begin(), sorted_customers.end());
    
    // Calculate customers per depot
    int total_customers = sorted_customers.size();
    int customers_per_depot = (total_customers + num_depots - 1) / num_depots;
    
    // Assign in blocks for spatial locality
    for (size_t i = 0; i < sorted_customers.size(); i++) {
        size_t depot_idx = (i / customers_per_depot) % num_depots;
        std::string depot_id = depot_ids[depot_idx];
        inst.customers[sorted_customers[i].second].territory_depot = depot_id;
    }
    
    // Log distribution
    std::map<std::string, int> counts;
    for (const auto& kv : inst.customers) {
        counts[kv.second.territory_depot]++;
    }
    std::cout << "[Territory] Balanced assignment:\n";
    for (const auto& kv : counts) {
        std::cout << "  " << kv.first << ": " << kv.second << " customers\n";
    }
}

inline void assign_quartile_classes(Instance& inst) {
    // For each depot territory, compute quartiles of dist(depot, customer)
    // then map into 4 classes: bike, motorbike, ev_van, van
    std::unordered_map<std::string, std::vector<std::pair<double,std::string>>> by_depot;
    by_depot.reserve(inst.depots.size());
    for (const auto& kv : inst.customers) {
        const auto& c = kv.second;
        const std::string& did = c.territory_depot;
        double d = inst.get_dist_km(did, c.id);
        if (d >= 1e8) {
            // fallback using haversine to depot coordinates
            const auto& dep = inst.depots.at(did);
            d = Instance::haversine_km(dep.lat, dep.lon, c.lat, c.lon);
        }
        by_depot[did].push_back({d, c.id});
    }

    for (auto& kv : by_depot) {
        auto& vec = kv.second;
        if (vec.empty()) continue;
        std::vector<double> ds;
        ds.reserve(vec.size());
        for (auto& p : vec) ds.push_back(p.first);
        std::sort(ds.begin(), ds.end());
        auto q = [&](double frac) {
            size_t idx = (size_t)std::floor(frac * (ds.size()-1));
            return ds[idx];
        };
        double q25 = q(0.25), q50 = q(0.50), q75 = q(0.75);

        for (auto& p : vec) {
            auto& c = inst.customers.at(p.second);
            double d = p.first;
            if (d <= q25) c.class_req = VehicleType::BIKE;
            else if (d <= q50) c.class_req = VehicleType::MOTORBIKE;
            else if (d <= q75) c.class_req = VehicleType::EV_VAN;
            else c.class_req = VehicleType::VAN;
        }
    }
}

inline double customer_score(const Customer& c) {
    // Heavy-first: prioritize weight only
    // Distance bonus added separately in choose_seed
    return c.weight * 1000.0 + c.volume;
}

inline std::string choose_seed(const Instance& inst,
                               const std::vector<std::string>& candidates,
                               const Vehicle& veh,
                               const std::string& depot_id,
                               double depot_rem_w,
                               double veh_rem_km)
{
    // Filter feasible candidates
    std::vector<std::string> feasible;
    for (const auto& cid : candidates) {
        const auto& c = inst.customers.at(cid);

        if (type_rank(veh.type) < type_rank(c.class_req)) continue; // vehicle too small for class
        if (c.weight > veh.cap_weight || c.volume > veh.cap_volume) continue;
        if (depot_rem_w < c.weight) continue;
        
        // Check road restrictions
        if (!inst.edge_allowed(depot_id, cid, veh.type)) continue;

        // trip distance budget (center initially = seed)
        double d = inst.get_dist_km(depot_id, cid);
        if (d >= 1e8) continue;
        double trip_km = 2.0 * d;
        if (trip_km > veh_rem_km + 1e-9) continue;

        feasible.push_back(cid);
    }
    
    if (feasible.empty()) return "";
    
    // Sort by heavy-first: weight DESC, distance DESC, (row,col) ASC
    std::sort(feasible.begin(), feasible.end(), [&](const std::string& a, const std::string& b) {
        const auto& ca = inst.customers.at(a);
        const auto& cb = inst.customers.at(b);
        
        // 1. Weight descending (heavier first)
        if (std::abs(ca.weight - cb.weight) > 1e-6) {
            return ca.weight > cb.weight;
        }
        
        // 2. Distance descending (farther first)
        double da = inst.get_dist_km(depot_id, a);
        double db = inst.get_dist_km(depot_id, b);
        if (da >= 1e8) da = 0; 
        if (db >= 1e8) db = 0;
        
        if (std::abs(da - db) > 1e-6) {
            return da > db;
        }
        
        // 3. Grid position (row, col) ascending
        if (ca.row != cb.row) return ca.row < cb.row;
        return ca.col < cb.col;
    });
    
    return feasible.front();
}

inline std::string update_center_by_mean(const Instance& inst, const std::vector<std::string>& members) {
    if (members.empty()) return "";
    double mx=0.0, my=0.0;
    for (const auto& cid : members) {
        const auto& c = inst.customers.at(cid);
        mx += c.x_km; my += c.y_km;
    }
    mx /= members.size();
    my /= members.size();

    // pick member closest to mean
    double best = 1e18;
    std::string best_id = members.front();
    for (const auto& cid : members) {
        const auto& c = inst.customers.at(cid);
        double dd = std::hypot(c.x_km - mx, c.y_km - my);
        if (dd < best) { best = dd; best_id = cid; }
    }
    return best_id;
}

} // namespace init_detail


class Initializer {
public:
    static Solution create_initial_solution(Instance inst, int seed = 42) {
        using namespace init_detail;
        std::mt19937 rng(seed);

        // preprocess territory + quartile classes
        assign_territory(inst);
        assign_quartile_classes(inst);

        Solution sol;
        for (const auto& kv : inst.customers) sol.unassigned_customers.insert(kv.first);

        // depot remaining storage
        std::unordered_map<std::string, double> depot_rem_w;
        for (const auto& kv : inst.depots) depot_rem_w[kv.first] = kv.second.cap_weight_storage;

        // group customers by territory depot
        std::unordered_map<std::string, std::vector<std::string>> territory_customers;
        for (const auto& kv : inst.customers) {
            territory_customers[kv.second.territory_depot].push_back(kv.first);
        }

        // vehicles by depot
        std::unordered_map<std::string, std::vector<std::string>> depot_vehicles;
        for (const auto& kv : inst.vehicles) {
            depot_vehicles[kv.second.start_depot].push_back(kv.first);
        }
        for (auto& kv : depot_vehicles) {
            auto& vec = kv.second;
            // Sort by capacity DESC (Largest vehicles first to maximize coverage)
            std::sort(vec.begin(), vec.end(), [&](const std::string& a, const std::string& b) {
                const auto& va = inst.vehicles.at(a);
                const auto& vb = inst.vehicles.at(b);
                if (va.cap_weight != vb.cap_weight) return va.cap_weight > vb.cap_weight; // DESC
                return va.cap_volume > vb.cap_volume; // DESC
            });
        }

        // track used km per vehicle
        std::unordered_map<std::string, double> veh_used_km;
        for (const auto& kv : inst.vehicles) veh_used_km[kv.first] = 0.0;

        const int MIN_CLUSTER_SIZE = 1;  
        const int MAX_TRIPS_PER_VEH_SOFT = 1000000; // no limit here

        // For each depot, iteratively create clusters
        for (const auto& dk : inst.depots) {
            const std::string did = dk.first;
            auto& cust_pool = territory_customers[did];
            if (cust_pool.empty()) continue;

            // build grid index over territory customers
            GridIndex grid_idx;
            grid_idx.build(inst, cust_pool);
            
            // Grid index built

            bool progressed = true;
            int pass_count = 0;
            const int MAX_PASSES = 100;  // Safety limit
            
            while (progressed && pass_count < MAX_PASSES) {
                progressed = false;
                pass_count++;
                
                std::cout << "[Depot " << did << "] Pass " << pass_count 
                          << " | Unassigned: " << sol.unassigned_customers.size() << std::endl;
                
                // pass = iterate vehicles (cap desc)
                for (const auto& vid : depot_vehicles[did]) {
                    const Vehicle& veh = inst.vehicles.at(vid);
                    if (trim(veh.start_depot) != trim(did)) continue;

                    // GREEDY TRIP CREATION: Keep adding trips to this vehicle until full/failed
                    bool vehicle_active = true;
                    while (vehicle_active) {
                        vehicle_active = false; // assume done unless we succeed
                        
                        if ((int)sol.routes[vid].size() >= MAX_TRIPS_PER_VEH_SOFT) break;

                        double rem_km = veh.max_dist - veh_used_km[vid];
                        if (rem_km <= 1e-9) break;
                        double rem_dep_w = depot_rem_w[did];
                        if (rem_dep_w <= 1e-9) break;

                        // seed candidates = unassigned customers in territory
                        std::vector<std::string> candidates;
                        candidates.reserve(cust_pool.size());
                        for (const auto& cid : cust_pool) {
                            if (sol.unassigned_customers.count(cid)) candidates.push_back(cid);
                        }
                        if (candidates.empty()) break; // No unassigned left in territory

                        std::string seed_cid = choose_seed(inst, candidates, veh, did, rem_dep_w, rem_km);
                        if (seed_cid.empty()) {
                            // No feasible seed for this vehicle currently
                            // Move to next vehicle
                            break;
                        }
                        
                        // Seed created

                        // start cluster
                        Route r;
                        r.vehicle_id = vid;
                        r.cluster_id = std::to_string((int)sol.routes[vid].size() + 1);
                        r.centroid_id = seed_cid;
                        r.cluster_customers.clear();
                        r.cluster_customers.push_back(seed_cid);
                        r.stops = {did, seed_cid, did};

                        double load_w = inst.customers.at(seed_cid).weight;
                        double load_v = inst.customers.at(seed_cid).volume;

                        double d0 = inst.get_dist_km(did, seed_cid);
                        if (d0 >= 1e8) continue;
                        double trip_km = 2.0 * d0;
                        if (trip_km > rem_km + 1e-9) break;

                        // reserve seed so other vehicles (or next iter) don't grab it
                        sol.unassigned_customers.erase(seed_cid);

                        // maxDist accounting (robust)
                        const double used_base = veh_used_km[vid];
                        if (used_base + trip_km > veh.max_dist + 1e-9) {
                            sol.unassigned_customers.insert(seed_cid); // put back
                            break;
                        }

                        // ===== sparse-grid growth =====
                        std::unordered_set<std::string> in_cluster;
                        in_cluster.insert(seed_cid);

                        constexpr double GRID_CELL_KM = 0.1;
                        int ring = 1;
                        int ring_max = std::max(1, (int)std::ceil(spread_limit_km(veh.type) / GRID_CELL_KM) + 1);
                        const int CAND_LIMIT = 2000;

                        std::set<GridCoord> frontier;
                        auto rebuild_frontier = [&](const GridCoord& base, int R) {
                            frontier.clear();
                            auto neigh = grid_idx.get_neighbors(base, R, true);
                            for (const auto& g : neigh) frontier.insert(g);
                        };

                        GridCoord seed_gc{inst.customers.at(seed_cid).row, inst.customers.at(seed_cid).col};
                        rebuild_frontier(seed_gc, ring);

                        while (true) {
                            // Collect candidates from current frontier
                            std::vector<std::string> cand_ids;
                            cand_ids.reserve(256);

                            for (const auto& gc : frontier) {
                                const auto& cell = grid_idx.customers_at(gc);
                                for (const auto& cid : cell) {
                                    if (!sol.unassigned_customers.count(cid)) continue;
                                    if (in_cluster.count(cid)) continue;
                                    cand_ids.push_back(cid);
                                    if ((int)cand_ids.size() >= CAND_LIMIT) break;
                                }
                                if ((int)cand_ids.size() >= CAND_LIMIT) break;
                            }

                            // Nothing in this ring -> expand
                            if (cand_ids.empty()) {
                                ring++;
                                if (ring > ring_max) break;
                                GridCoord cur_gc{inst.customers.at(r.centroid_id).row, inst.customers.at(r.centroid_id).col};
                                rebuild_frontier(cur_gc, ring);
                                continue;
                            }

                            // Sort candidates
                            const auto& cent = inst.customers.at(r.centroid_id);
                            std::sort(cand_ids.begin(), cand_ids.end(), [&](const std::string& a, const std::string& b) {
                                const auto& ca = inst.customers.at(a);
                                const auto& cb = inst.customers.at(b);
                                if (std::abs(ca.weight - cb.weight) > 1e-9) return ca.weight > cb.weight; // heavy-first
                                double da = std::hypot(ca.x_km - cent.x_km, ca.y_km - cent.y_km);
                                double db = std::hypot(cb.x_km - cent.x_km, cb.y_km - cent.y_km);
                                if (std::abs(da - db) > 1e-9) return da < db; // compact
                                if (ca.row != cb.row) return ca.row < cb.row;
                                if (ca.col != cb.col) return ca.col < cb.col;
                                return a < b;
                            });

                            bool accepted = false;

                            for (const auto& cid : cand_ids) {
                                const auto& cand = inst.customers.at(cid);

                                if (type_rank(veh.type) < type_rank(cand.class_req)) continue;
                                if (load_w + cand.weight > veh.cap_weight + 1e-9) continue;
                                if (load_v + cand.volume > veh.cap_volume + 1e-9) continue;
                                if (depot_rem_w[did] < (load_w + cand.weight) - 1e-9) continue;

                                auto members_tmp = r.cluster_customers;
                                members_tmp.push_back(cid);
                                std::string new_center = update_center_by_mean(inst, members_tmp);
                                
                                // Check if new center is reachable by vehicle type (road restrictions)
                                if (!inst.edge_allowed(did, new_center, veh.type)) continue;

                                double dnew = inst.get_dist_km(did, new_center);
                                if (dnew >= 1e8) continue;
                                double new_trip_km = 2.0 * dnew;

                                if (used_base + new_trip_km > veh.max_dist + 1e-9) continue;

                                double lim = spread_limit_km(veh.type);
                                double maxr = max_radius_km_to_center(inst, new_center, members_tmp);
                                if (maxr > lim + 1e-9) continue;

                                r.cluster_customers.push_back(cid);
                                in_cluster.insert(cid);
                                sol.unassigned_customers.erase(cid);
                                load_w += cand.weight;
                                load_v += cand.volume;
                                r.centroid_id = new_center;
                                trip_km = new_trip_km;

                                ring = 1;
                                GridCoord new_gc{inst.customers.at(r.centroid_id).row, inst.customers.at(r.centroid_id).col};
                                rebuild_frontier(new_gc, ring);

                                accepted = true;
                                break;
                            }

                            if (!accepted) {
                                ring++;
                                if (ring > ring_max) break;
                                GridCoord cur_gc{inst.customers.at(r.centroid_id).row, inst.customers.at(r.centroid_id).col};
                                rebuild_frontier(cur_gc, ring);
                            }
                        }

                        // commit maxDist usage
                        veh_used_km[vid] = used_base + trip_km;
                        r.stops = {did, r.centroid_id, did};
                        r.load_w = load_w;
                        r.load_v = load_v;
                        r.distance = trip_km;

                        if ((int)r.cluster_customers.size() < MIN_CLUSTER_SIZE) {
                            // rollback
                            std::cout << "    [DEBUG] Vehicle " << vid << ": ROLLBACK cluster size=" 
                                      << r.cluster_customers.size() << " < " << MIN_CLUSTER_SIZE << std::endl;
                            for (const auto& cid : r.cluster_customers) sol.unassigned_customers.insert(cid);
                            // revert used km
                            double used = 0.0;
                            for (const auto& rr : sol.routes[vid]) used += rr.distance;
                            veh_used_km[vid] = used;
                            continue; // Try greedy loop again? No, if rollback logic fails, probably can't create trip.
                            // But maybe we picked bad seed.
                            // For safety, let's break greedy loop if we fail to form a valid cluster.
                            // This signals "cannot use this vehicle anymore here".
                            break; 
                        }
                        
                        std::cout << "    [DEBUG] Vehicle " << vid << ": COMMIT cluster size=" 
                                  << r.cluster_customers.size() << ", load=" << load_w << "kg" << std::endl;

                        depot_rem_w[did] -= load_w;
                        sol.routes[vid].push_back(r);
                        
                        progressed = true; // Global progress
                        vehicle_active = true; // Continue using THIS vehicle
                    } // end greedy while
                }
            }
        }

        // evaluate
        sol.calculate_objective(inst);
        return sol;
    }
};
