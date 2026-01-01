#pragma once

#include "config.h"
#include "utils.h"

#include <unordered_map>
#include <unordered_set>
#include <map>
#include <set>
#include <random>
#include <limits>

/*
Initialization (NEW, territory-free)
===================================
- NO "customer belongs to depot" pre-assignment.
- Vehicles are still fixed to their depot (K_d).
- We build "implicit depot territories" by forcing *cluster centers* of the same depot to be adjacent:
    * When a depot already has some clusters, new seeds are chosen near existing centers (patch seeding).
    * This creates a contiguous center-patch per depot without any hard territory checks on customers.
- Customers remain globally unassigned until inserted into some cluster.

The initializer is intentionally simple & robust:
- Seed selection: near depot or near depot's existing center patch.
- Growth: grid frontier expansion (8-neighbor) with soft-ish feasibility checks:
    * per-trip cap (weight/volume) HARD in init (avoid nonsense)
    * depot storage cap (weight) HARD in init
    * per-vehicle remaining km HARD in init
  (ALNS later can soften these via penalties.)
*/

namespace init_detail {

struct GridCoord {
    int row = 0, col = 0;
    bool operator<(const GridCoord& o) const {
        if (row != o.row) return row < o.row;
        return col < o.col;
    }
    bool operator==(const GridCoord& o) const {
        return row == o.row && col == o.col;
    }
};

// Sparse grid index (based on Customer.row/col).
class GridIndex {
public:
    void build(const Instance& inst) {
        grid_map.clear();
        grid_map.reserve(inst.customers.size() * 2);
        for (const auto& kv : inst.customers) {
            const auto& c = kv.second;
            grid_map[GridCoord{c.row, c.col}].push_back(kv.first);
        }
    }

    const std::vector<std::string>& customers_at(const GridCoord& gc) const {
        static const std::vector<std::string> empty;
        auto it = grid_map.find(gc);
        if (it == grid_map.end()) return empty;
        return it->second;
    }

private:
    struct Hash {
        std::size_t operator()(const GridCoord& g) const noexcept {
            return (std::size_t)((uint32_t)g.row * 1315423911u) ^ (uint32_t)g.col;
        }
    };
    std::unordered_map<GridCoord, std::vector<std::string>, Hash> grid_map;
};

inline VehicleType class_from_quartile(int q) {
    if (q <= 0) return VehicleType::BIKE;
    if (q == 1) return VehicleType::MOTORBIKE;
    if (q == 2) return VehicleType::EV_VAN;
    return VehicleType::VAN;
}

// Assign customer.class_req by GLOBAL quartiles of distance to nearest depot.
// (No territory; only "how far is this customer from its nearest depot".)
inline void assign_class_by_nearest_depot_quartiles(Instance& inst) {
    std::vector<double> dmin;
    dmin.reserve(inst.customers.size());

    auto nearest_depot_dist = [&](const Customer& c) -> double {
        double best = 1e18;
        for (const auto& kv : inst.depots) {
            const auto& d = kv.second;
            const double dd = std::hypot(c.x_km - d.x_km, c.y_km - d.y_km);
            if (dd < best) best = dd;
        }
        return (best >= 1e17) ? 0.0 : best;
    };

    for (const auto& kv : inst.customers) dmin.push_back(nearest_depot_dist(kv.second));
    if (dmin.empty()) return;
    std::sort(dmin.begin(), dmin.end());

    auto qv = [&](double frac) {
        size_t idx = (size_t)std::floor(frac * (dmin.size()-1));
        return dmin[idx];
    };
    const double q25 = qv(0.25), q50 = qv(0.50), q75 = qv(0.75);

    for (auto& kv : inst.customers) {
        auto& c = kv.second;
        const double d = nearest_depot_dist(c);
        if (d <= q25) c.class_req = VehicleType::BIKE;
        else if (d <= q50) c.class_req = VehicleType::MOTORBIKE;
        else if (d <= q75) c.class_req = VehicleType::EV_VAN;
        else c.class_req = VehicleType::VAN;
    }
}

// Choose a seed among candidates (unassigned already filtered outside).
// Score = heavy-first + proximity-to-anchor (patch).
inline std::string choose_seed(const Instance& inst,
                               const std::vector<std::string>& candidates,
                               const Vehicle& veh,
                               const std::string& depot_id,
                               const GridCoord* anchor_cell, // optional (for patch seeding)
                               double depot_rem_w,
                               double veh_rem_km)
{
    if (candidates.empty()) return "";

    auto cell_dist = [&](const Customer& c, const GridCoord& a) -> double {
        // Chebyshev distance in grid cells (cheap)
        return (double)std::max(std::abs(c.row - a.row), std::abs(c.col - a.col));
    };

    std::string best_id;
    double best_score = -1e100;

    for (const auto& cid : candidates) {
        const auto& c = inst.customers.at(cid);

        if (type_rank(veh.type) < type_rank(c.class_req)) continue;
        if (c.weight > veh.cap_weight + 1e-9) continue;
        if (c.volume > veh.cap_volume + 1e-9) continue;
        if (depot_rem_w < c.weight - 1e-9) continue;

        if (!inst.edge_allowed(depot_id, cid, veh.type)) continue;

        double d = inst.get_dist_km(depot_id, cid);
        if (d >= 1e8) continue;
        double trip_km = 2.0 * d;
        if (trip_km > veh_rem_km + 1e-9) continue;

        // heavy-first + patch proximity
        double score = c.weight * 1000.0 + c.volume;

        if (anchor_cell) {
            score -= 2.0 * cell_dist(c, *anchor_cell); // pull toward patch
        } else {
            // if no anchor, prefer closer to depot
            const auto& dep = inst.depots.at(depot_id);
            score -= 50.0 * std::hypot(c.x_km - dep.x_km, c.y_km - dep.y_km);
        }

        if (score > best_score) { best_score = score; best_id = cid; }
    }

    return best_id;
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

    double best = 1e18;
    std::string best_id = members.front();
    for (const auto& cid : members) {
        const auto& c = inst.customers.at(cid);
        double dd = std::hypot(c.x_km - mx, c.y_km - my);
        if (dd < best) { best = dd; best_id = cid; }
    }
    return best_id;
}

// Collect unassigned candidates in expanding ring around an anchor cell.
// Ring boundary: max(|dr|,|dc|)==R.
inline void ring_candidates(const GridIndex& gidx,
                            const Instance& inst,
                            const std::set<std::string>& unassigned,
                            const GridCoord& anchor,
                            int Rmax,
                            int need,
                            std::vector<std::string>& out)
{
    out.clear();
    out.reserve((size_t)need);

    for (int R=0; R<=Rmax && (int)out.size()<need; ++R) {
        for (int dr=-R; dr<=R; ++dr) {
            for (int dc=-R; dc<=R; ++dc) {
                if (std::max(std::abs(dr), std::abs(dc)) != R) continue;
                GridCoord gc{anchor.row + dr, anchor.col + dc};
                const auto& vec = gidx.customers_at(gc);
                for (const auto& cid : vec) {
                    if ((int)out.size() >= need) break;
                    if (unassigned.count(cid)) out.push_back(cid);
                }
            }
        }
    }
}

// Build a depot "anchor cell": nearest customer to depot (robust even if depot has no grid row/col).
inline GridCoord depot_anchor_cell(const Instance& inst, const std::string& depot_id) {
    const auto& dep = inst.depots.at(depot_id);
    double best = 1e18;
    GridCoord best_gc{0,0};
    for (const auto& kv : inst.customers) {
        const auto& c = kv.second;
        double d = std::hypot(c.x_km - dep.x_km, c.y_km - dep.y_km);
        if (d < best) { best = d; best_gc = GridCoord{c.row, c.col}; }
    }
    return best_gc;
}

} // namespace init_detail


class Initializer {
public:
    static Solution create_initial_solution(Instance inst, int seed = 42) {
        using namespace init_detail;
        std::mt19937 rng(seed);

        // --- preprocess class requirements (no territory) ---
        assign_class_by_nearest_depot_quartiles(inst);

        Solution sol;
        for (const auto& kv : inst.customers) sol.unassigned_customers.insert(kv.first);

        // depot remaining storage (weight only)
        std::unordered_map<std::string, double> depot_rem_w;
        for (const auto& kv : inst.depots) depot_rem_w[kv.first] = kv.second.cap_weight_storage;

        // vehicles by depot (sort by capacity DESC)
        std::unordered_map<std::string, std::vector<std::string>> depot_vehicles;
        for (const auto& kv : inst.vehicles) depot_vehicles[kv.second.start_depot].push_back(kv.first);
        for (auto& kv : depot_vehicles) {
            auto& vec = kv.second;
            std::sort(vec.begin(), vec.end(), [&](const std::string& a, const std::string& b) {
                const auto& va = inst.vehicles.at(a);
                const auto& vb = inst.vehicles.at(b);
                if (va.cap_weight != vb.cap_weight) return va.cap_weight > vb.cap_weight;
                return va.cap_volume > vb.cap_volume;
            });
        }

        // track used km per vehicle
        std::unordered_map<std::string, double> veh_used_km;
        for (const auto& kv : inst.vehicles) veh_used_km[kv.first] = 0.0;

        // global grid
        GridIndex gidx;
        gidx.build(inst);

        // per-depot center patch (store center customer ids)
        std::unordered_map<std::string, std::vector<std::string>> depot_centers;

        // precompute depot anchor cell
        std::unordered_map<std::string, GridCoord> depot_anchor;
        for (const auto& kv : inst.depots) depot_anchor[kv.first] = depot_anchor_cell(inst, kv.first);

        const int MAX_PASSES = 200;      // safety
        const int SEED_RING_MAX = 30;    // in grid cells
        const int SEED_NEED = 200;       // candidate pool size
        const int GROW_ITERS = 5000;     // hard cap per trip

        for (int pass=1; pass<=MAX_PASSES; ++pass) {
            bool progressed = false;

            // stop early if everything served
            if (sol.unassigned_customers.empty()) break;

            for (const auto& dk : inst.depots) {
                const std::string did = dk.first;
                double& dep_rem = depot_rem_w[did];
                if (dep_rem <= 1e-9) continue;

                auto itVehList = depot_vehicles.find(did);
                if (itVehList == depot_vehicles.end()) continue;

                // choose a patch anchor cell: random existing center, else depot anchor
                auto pick_patch_anchor_cell = [&]() -> GridCoord {
                    auto it = depot_centers.find(did);
                    if (it == depot_centers.end() || it->second.empty()) return depot_anchor[did];
                    std::uniform_int_distribution<int> D(0, (int)it->second.size()-1);
                    const std::string& cen_cid = it->second[D(rng)];
                    const auto& cen = inst.customers.at(cen_cid);
                    return GridCoord{cen.row, cen.col};
                };

                for (const auto& vid : itVehList->second) {
                    const Vehicle& veh = inst.vehicles.at(vid);
                    if (trim(veh.start_depot) != trim(did)) continue;

                    while (true) {
                        double rem_km = veh.max_dist - veh_used_km[vid];
                        if (rem_km <= 1e-9) break;
                        if (dep_rem <= 1e-9) break;
                        if (sol.unassigned_customers.empty()) break;

                        // --- seed candidates ---
                        std::vector<std::string> seed_cands;
                        GridCoord patch_cell = pick_patch_anchor_cell();
                        ring_candidates(gidx, inst, sol.unassigned_customers, patch_cell, SEED_RING_MAX, SEED_NEED, seed_cands);

                        // fallback: if patch ring is empty, just sample some unassigned (cheap)
                        if (seed_cands.empty()) {
                            seed_cands.reserve(256);
                            int cnt = 0;
                            for (const auto& cid : sol.unassigned_customers) {
                                seed_cands.push_back(cid);
                                if (++cnt >= 256) break;
                            }
                        }

                        std::string seed_cid = choose_seed(inst, seed_cands, veh, did, &patch_cell, dep_rem, rem_km);
                        if (seed_cid.empty()) break;

                        // --- start new trip ---
                        Route r;
                        r.vehicle_id = vid;
                        r.cluster_customers.clear();
                        r.centroid_id.clear();
                        r.cluster_id = vid + "_t" + std::to_string((int)sol.routes[vid].size() + 1);

                        // init stats
                        double load_w = 0.0, load_v = 0.0;
                        double sumx = 0.0, sumy = 0.0;

                        auto try_add = [&](const std::string& cid) -> bool {
                            const auto& c = inst.customers.at(cid);
                            if (!sol.unassigned_customers.count(cid)) return false;
                            if (type_rank(veh.type) < type_rank(c.class_req)) return false;
                            if (load_w + c.weight > veh.cap_weight + 1e-9) return false;
                            if (load_v + c.volume > veh.cap_volume + 1e-9) return false;
                            if (dep_rem < c.weight - 1e-9) return false;

                            // optimistic add (final checks later)
                            r.cluster_customers.push_back(cid);
                            load_w += c.weight; load_v += c.volume;
                            sumx += c.x_km; sumy += c.y_km;
                            return true;
                        };

                        if (!try_add(seed_cid)) break;
                        sol.unassigned_customers.erase(seed_cid);

                        // grow frontier as set of member cells
                        std::set<GridCoord> frontier;
                        const auto& seedc = inst.customers.at(seed_cid);
                        frontier.insert(GridCoord{seedc.row, seedc.col});

                        auto current_center = [&]() -> std::string {
                            return update_center_by_mean(inst, r.cluster_customers);
                        };

                        auto trip_km_of_center = [&](const std::string& center_cid) -> double {
                            double d = inst.get_dist_km(did, center_cid);
                            if (d >= 1e8) return 1e18;
                            return 2.0 * d;
                        };

                        // finalize seed center
                        r.centroid_id = current_center();
                        double trip_km = trip_km_of_center(r.centroid_id);
                        if (!inst.edge_allowed(did, r.centroid_id, veh.type) || trip_km > rem_km + 1e-9) {
                            // rollback seed
                            for (const auto& cid : r.cluster_customers) sol.unassigned_customers.insert(cid);
                            r.cluster_customers.clear();
                            r.centroid_id.clear();
                            break;
                        }

                        // grow loop
                        bool grew = true;
                        int iters = 0;
                        while (grew && iters++ < GROW_ITERS) {
                            grew = false;

                            // collect candidates from 8-neighbor of frontier (ring boundary R=1)
                            std::vector<std::string> cands;
                            cands.reserve(512);
                            for (const auto& fc : frontier) {
                                for (int dr=-1; dr<=1; ++dr) {
                                    for (int dc=-1; dc<=1; ++dc) {
                                        GridCoord gc{fc.row + dr, fc.col + dc};
                                        const auto& vec = gidx.customers_at(gc);
                                        for (const auto& cid : vec) {
                                            if (sol.unassigned_customers.count(cid)) cands.push_back(cid);
                                        }
                                    }
                                }
                            }
                            if (cands.empty()) break;

                            // score candidates: close to centroid + patch pull
                            const double cx = sumx / (double)r.cluster_customers.size();
                            const double cy = sumy / (double)r.cluster_customers.size();

                            std::string best_cid;
                            double best_score = 1e100;

                            for (const auto& cid : cands) {
                                const auto& c = inst.customers.at(cid);
                                if (type_rank(veh.type) < type_rank(c.class_req)) continue;
                                if (load_w + c.weight > veh.cap_weight + 1e-9) continue;
                                if (load_v + c.volume > veh.cap_volume + 1e-9) continue;
                                if (dep_rem < c.weight - 1e-9) continue;

                                // tentative new center and km
                                std::vector<std::string> tmp = r.cluster_customers;
                                tmp.push_back(cid);
                                std::string cen2 = update_center_by_mean(inst, tmp);

                                double trip2 = trip_km_of_center(cen2);
                                if (trip2 > rem_km + 1e-9) continue;
                                if (!inst.edge_allowed(did, cen2, veh.type)) continue;

                                // spread (soft in model, but keep as guard here)
                                double lim = spread_limit_km(veh.type);
                                const auto& cenC = inst.customers.at(cen2);
                                double maxr = 0.0;
                                for (const auto& mid : tmp) {
                                    const auto& mm = inst.customers.at(mid);
                                    maxr = std::max(maxr, std::hypot(mm.x_km - cenC.x_km, mm.y_km - cenC.y_km));
                                    if (maxr > lim + 1e-9) break;
                                }
                                if (maxr > lim + 1e-9) continue;

                                double s = 0.0;
                                s += std::hypot(c.x_km - cx, c.y_km - cy); // centroid pull
                                s += 0.25 * std::hypot(c.row - patch_cell.row, c.col - patch_cell.col); // patch pull
                                s -= 0.001 * c.weight; // tiny heavy bias

                                if (s < best_score) { best_score = s; best_cid = cid; }
                            }

                            if (best_cid.empty()) break;

                            // accept best
                            if (try_add(best_cid)) {
                                sol.unassigned_customers.erase(best_cid);
                                const auto& bc = inst.customers.at(best_cid);
                                frontier.insert(GridCoord{bc.row, bc.col});

                                r.centroid_id = current_center();
                                trip_km = trip_km_of_center(r.centroid_id);
                                grew = true;
                                progressed = true;
                            }
                        }

                        // commit this trip
                        if (!r.cluster_customers.empty()) {
                            r.stops = {did, r.centroid_id, did};
                            r.distance = trip_km;
                            sol.routes[vid].push_back(r);

                            // update budgets
                            veh_used_km[vid] += trip_km;
                            dep_rem -= load_w;

                            // update depot patch centers
                            depot_centers[did].push_back(r.centroid_id);
                        } else {
                            // nothing committed (shouldn't happen)
                            break;
                        }
                    } // while true (create trips)
                } // for vehicles
            } // for depots

            if (!progressed) break;
        } // passes

        return sol;
    }
};
