#pragma once

#include "config.h"
#include "initialization.h" // for init_detail::assign_territory / assign_quartile_classes
#include "objectives.h"

#include <unordered_set>
#include <unordered_map>
#include <random>
#include <array>
#include <limits>
#include <deque>

// ==============================================================
//  ALNS + VND (seed-growing flavor) – aligned with current structs
//
//  Mapping to your existing code:
//  - One Route in Solution == one Trip/Cluster in the ALNS script.
//  - Route.vehicle_id determines depot (K_d) via inst.vehicles[vehicle_id].depot_id.
//  - Cluster cost matches Objectives:
//      fixed_cost_per_trip + 2*dist(depot, center) * var_cost * traffic_multiplier
//  - Hard feasibility checks follow objectives.h (cap, spread, max_dist, depot_storage,
//    and edge_allowed(depot->center, vehicle_type)).
//
//  Design intent:
//  - Keep Solution as I/O format.
//  - Maintain an internal State with cached cluster stats (sumx/sumy/load/center/trip_km).
//  - Implement destroy/repair operators + light VND around touched clusters.
//
//  This is a **working scaffold**: it is correct and compilable, and you can tune
//  selection heuristics/parameters later without changing the data model.
// ==============================================================

namespace alns_vnd {

static constexpr double PI = 3.141592653589793238462643383279502884;

struct Params {
    int iters = 10000;  // Increased for deeper optimization
    unsigned seed = 123;

    // SA acceptance - MORE EXPLORATION
    double T0 = 100.0;        // Higher initial temperature for more exploration
    double cooling = 0.998;   // Slower cooling for better search

    // Neighbor / boundary - REDUCED to preserve clusters
    int nbr_k = 8;              // number of neighboring clusters per cluster
    int knn_k = 10;             // for patch removal
    double boundary_frac = 0.10; // boundary size = frac*|C| (reduced from 0.15)
    int boundary_min = 5;
    int boundary_max = 20;

    // Patch removal - REDUCED to avoid destroying too much
    double patch_frac = 0.15;    // reduced from 0.25
    int patch_min = 5;
    double patch_max_frac = 0.25; // reduced from 0.40

    // Repair - EXTREMELY AGGRESSIVE
    int batch_M = 30;          // Try inserting many customers at once (increased from 20)
    bool allow_ejection = true;
    int ejection_L = 15;       // More ejection attempts to make room (increased from 10)
    int tabu_ttl = 25;
    bool allow_open_trip = true;
    double open_trip_trigger = 0.2; // Open trips very early (when 20%+ unassigned, from 0.5)

    // Constraints - RELAXED for better utilization
    bool enforce_territory = false;  // DISABLED: Allow cross-territory assignment
    bool enforce_class_req = false;  // DISABLED: Allow any vehicle to serve any customer
};

enum class DestroyOp { Boundary = 0, Patch = 1, WholeCluster = 2, Greedy = 3, ShuffleAll = 4 };
enum class RepairOp  { Grow1 = 0, GrowBatch = 1, Greedy = 2, Drain = 3, Regret = 4 };

// --------- small helpers ----------
inline int sample_discrete(std::mt19937& rng, const std::vector<double>& w) {
    double sum = 0.0;
    for (double x : w) sum += std::max(0.0, x);
    if (sum <= 0) {
        std::uniform_int_distribution<int> uni(0, (int)w.size()-1);
        return uni(rng);
    }
    std::uniform_real_distribution<double> uni(0.0, sum);
    double r = uni(rng);
    double acc = 0.0;
    for (int i=0;i<(int)w.size();++i) {
        acc += std::max(0.0, w[i]);
        if (r <= acc) return i;
    }
    return (int)w.size()-1;
}

inline int sector8(double dx, double dy) {
    double ang = std::atan2(dy, dx);           // [-pi,pi]
    double t = (ang + PI) / (PI/4.0);          // [0,8)
    int s = (int)std::floor(t);
    if (s < 0) s = 0;
    if (s > 7) s = 7;
    return s;
}

// ==============================================================
// Context: preprocessed Instance + fast arrays
// ==============================================================
struct CustView {
    std::string id;
    double x=0, y=0;
    double lat=0, lon=0;
    double w=0, v=0;
    std::string territory;
    VehicleType class_req = VehicleType::UNKNOWN;
};

struct VehView {
    std::string id;
    std::string depot_id;
    VehicleType type = VehicleType::UNKNOWN;
    double cap_w=0, cap_v=0;
    double fixed=0, var=0;
    double max_dist=0;
};

struct DepotView {
    std::string id;
    double cap_w=0, cap_v=0;
};

struct Context {
    Instance inst; // local preprocessed copy owned by Context
    Params p;

    std::vector<CustView> cust;
    std::unordered_map<std::string,int> cidx;

    std::vector<VehView> veh;
    std::unordered_map<std::string,int> vidx;

    std::vector<DepotView> dep;
    std::unordered_map<std::string,int> didx;

    // kNN graph for customers (by index)
    std::vector<std::vector<int>> knn;

    explicit Context(Instance inst_in, Params p_in) : inst(std::move(inst_in)), p(p_in) {
        // IMPORTANT: Initializer computes these on a copy; optimizer must do the same.
        init_detail::assign_territory(inst);
        init_detail::assign_quartile_classes(inst);

        // build views
        cust.reserve(inst.customers.size());
        for (const auto& kv : inst.customers) {
            const Customer& c = kv.second;
            int idx = (int)cust.size();
            cidx[c.id] = idx;
            cust.push_back({c.id, c.x_km, c.y_km, c.lat, c.lon, c.weight, c.volume,
                            c.territory_depot, c.class_req});
        }
        veh.reserve(inst.vehicles.size());
        for (const auto& kv : inst.vehicles) {
            const Vehicle& v = kv.second;
            int idx = (int)veh.size();
            vidx[v.id] = idx;
            veh.push_back({v.id, v.start_depot, v.type, v.cap_weight, v.cap_volume,
                           v.fixed_cost, v.var_cost, v.max_dist});
        }
        dep.reserve(inst.depots.size());
        for (const auto& kv : inst.depots) {
            const Depot& d = kv.second;
            int idx = (int)dep.size();
            didx[d.id] = idx;
            dep.push_back({d.id, d.cap_weight_storage, d.cap_volume_storage});
        }

        build_knn();
    }

    // Cost multiplier on the depot->center edge (by traffic level). If missing, treat as "normal".
    double traffic_mult(const std::string& from, const std::string& to) const {
        std::string level = "normal";
        auto it1 = inst.edge_traffic.find(from);
        if (it1 != inst.edge_traffic.end()) {
            auto it2 = it1->second.find(to);
            if (it2 != it1->second.end() && !it2->second.empty()) level = it2->second;
        }
        return inst.traffic_multiplier(level);
    }

    void build_knn() {
        int n = (int)cust.size();
        knn.assign(n, {});
        if (n == 0) return;
        int k = std::max(1, p.knn_k);

        // simple brute for moderate sizes
        const int BRUTE_N = 6000;
        if (n <= BRUTE_N) {
            for (int i=0;i<n;++i) {
                std::vector<std::pair<double,int>> d;
                d.reserve(n-1);
                for (int j=0;j<n;++j) if (j!=i) {
                    double dd = std::hypot(cust[i].x - cust[j].x, cust[i].y - cust[j].y);
                    d.push_back({dd, j});
                }
                int take = std::min(k, (int)d.size());
                std::partial_sort(d.begin(), d.begin()+take, d.end(),
                                  [](auto& a, auto& b){ return a.first < b.first; });
                knn[i].reserve(take);
                for (int t=0;t<take;++t) knn[i].push_back(d[t].second);
            }
            return;
        }

        // approximate grid (cell size 0.5km), expand rings until enough candidates
        const double cell = 0.5;
        auto pack = [](int x, int y)->long long { return ((long long)x<<32) ^ (unsigned int)y; };
        std::unordered_map<long long, std::vector<int>> buckets;
        buckets.reserve((size_t)n*2);
        for (int i=0;i<n;++i) {
            int ix=(int)std::floor(cust[i].x / cell);
            int iy=(int)std::floor(cust[i].y / cell);
            buckets[pack(ix,iy)].push_back(i);
        }

        for (int i=0;i<n;++i) {
            int ix=(int)std::floor(cust[i].x / cell);
            int iy=(int)std::floor(cust[i].y / cell);
            std::vector<int> cand;
            cand.reserve(128);
            for (int r=0; r<=6 && (int)cand.size()<k*6; ++r) {
                for (int dx=-r; dx<=r; ++dx) for (int dy=-r; dy<=r; ++dy) {
                    auto it = buckets.find(pack(ix+dx, iy+dy));
                    if (it==buckets.end()) continue;
                    for (int id : it->second) if (id!=i) cand.push_back(id);
                }
            }
            std::sort(cand.begin(), cand.end());
            cand.erase(std::unique(cand.begin(), cand.end()), cand.end());
            std::vector<std::pair<double,int>> d;
            d.reserve(cand.size());
            for (int j : cand) {
                double dd = std::hypot(cust[i].x - cust[j].x, cust[i].y - cust[j].y);
                d.push_back({dd, j});
            }
            int take = std::min(k, (int)d.size());
            std::partial_sort(d.begin(), d.begin()+take, d.end(),
                              [](auto& a, auto& b){ return a.first < b.first; });
            knn[i].reserve(take);
            for (int t=0;t<take;++t) knn[i].push_back(d[t].second);
        }
    }
};

// ==============================================================
// State: assignment + cached trip stats
// ==============================================================
struct Cluster {
    int vid = -1;
    int did = -1;
    bool active = true;

    std::vector<int> mem;  // customer indices
    int n=0;
    double sumx=0, sumy=0;
    double load_w=0, load_v=0;

    int center = -1;       // customer index
    double trip_km = 0;    // 2*dist(depot, center)

    std::vector<int> nbr;      // neighboring cluster ids
    std::vector<int> boundary; // boundary points (customer indices)
    bool dirty = true;
};

struct State {
    const Context* ctx = nullptr;
    std::vector<Cluster> clusters;
    std::vector<int> cust_cluster; // -1 if unassigned

    std::vector<double> depot_w, depot_v;
    std::vector<double> veh_km;

    std::unordered_set<int> unassigned;
    std::unordered_map<long long, int> tabu_until;

    int total_customer_count = 0;  // Total customers in Instance (for accurate unserved count)
    double obj=1e15, fixed=0, travel=0, pen=0;

    // constants - MASSIVELY INCREASED unserved penalty to absolutely prioritize customer service
    static constexpr double PEN_BASE = 2e6;  // Increased from 5e5 to 2M - DOMINANT PENALTY
    static constexpr double PEN_W = 200.0;   // Increased from 100.0
    static constexpr double PEN_V = 100.0;   // Increased from 50.0
    // NOTE: Customer priority is not in your current data model, so we don't weight it here.

    // ---------- build from Solution ----------
    static State from_solution(const Context& ctx, const Solution& sol) {
        State st;
        st.ctx = &ctx;
        int nC = (int)ctx.cust.size();
        st.total_customer_count = nC;  // Track total for accurate unserved count
        st.cust_cluster.assign(nC, -1);
        st.depot_w.assign(ctx.dep.size(), 0.0);
        st.depot_v.assign(ctx.dep.size(), 0.0);
        st.veh_km.assign(ctx.veh.size(), 0.0);

        // build clusters from routes
        std::vector<std::string> vids;
        vids.reserve(sol.routes.size());
        for (const auto& kv : sol.routes) vids.push_back(kv.first);
        std::sort(vids.begin(), vids.end());

        for (const std::string& vehicle_id : vids) {
            auto itV = ctx.vidx.find(vehicle_id);
            if (itV==ctx.vidx.end()) continue;
            int vid = itV->second;
            int did = ctx.didx.at(ctx.veh[vid].depot_id);
            const auto& routes = sol.routes.at(vehicle_id);
            for (const auto& r : routes) {
                if (r.cluster_customers.empty()) continue;
                Cluster c;
                c.vid = vid;
                c.did = did;
                c.active = true;
                c.mem.reserve(r.cluster_customers.size());
                for (const std::string& cid : r.cluster_customers) {
                    auto itC = ctx.cidx.find(cid);
                    if (itC==ctx.cidx.end()) continue;
                    int ui = itC->second;
                    if (st.cust_cluster[ui] != -1) continue; // ignore duplicates
                    st.cust_cluster[ui] = (int)st.clusters.size();
                    c.mem.push_back(ui);
                    const auto& u = ctx.cust[ui];
                    c.n++;
                    c.sumx += u.x; c.sumy += u.y;
                    c.load_w += u.w; c.load_v += u.v;
                }
                st.clusters.push_back(std::move(c));
            }
        }

        // any remaining customer => unassigned
        for (int i=0;i<(int)ctx.cust.size();++i) if (st.cust_cluster[i] == -1) {
            st.unassigned.insert(i);
        }

        // recompute centers + totals
        for (int cid=0; cid<(int)st.clusters.size(); ++cid) {
            st.recompute_cluster(cid);
            const auto& C = st.clusters[cid];
            if (C.mem.empty()) continue;
            st.depot_w[C.did] += C.load_w;
            st.depot_v[C.did] += C.load_v;
            st.veh_km[C.vid] += C.trip_km;
        }
        st.refresh_all_neighbors_and_boundaries();
        st.recompute_objective();
        return st;
    }

    // ---------- conversion to Solution ----------
    Solution to_solution() const {
        Solution sol;
        sol.routes.clear();
        for (const auto& v : ctx->veh) sol.routes[v.id] = {};

        // DEBUG: Track which customers we're adding
        std::unordered_set<std::string> added_customers;
        int total_clusters_converted = 0;

        // per vehicle trip counter
        std::vector<int> tcount(ctx->veh.size(), 0);
        for (int cid=0; cid<(int)clusters.size(); ++cid) {
            const auto& C = clusters[cid];
            if (!C.active || C.mem.empty()) continue;
            total_clusters_converted++;
            const auto& V = ctx->veh[C.vid];
            const auto& D = ctx->dep[C.did];
            Route r;
            r.vehicle_id = V.id;
            r.cluster_id = std::to_string(tcount[C.vid]++);
            r.load_w = C.load_w;
            r.load_v = C.load_v;
            if (C.center >= 0) r.centroid_id = ctx->cust[C.center].id;
            r.cluster_customers.reserve(C.mem.size());
            for (int ui : C.mem) {
                std::string cust_id = ctx->cust[ui].id;
                r.cluster_customers.push_back(cust_id);
                added_customers.insert(cust_id);
            }
            r.stops = {D.id, r.centroid_id, D.id};
            r.distance = C.trip_km;
            sol.routes[V.id].push_back(std::move(r));
        }

        std::cout << "[DEBUG to_solution] Converted " << total_clusters_converted 
                  << " clusters, added " << added_customers.size() 
                  << " unique customers to routes" << std::endl;

        Objectives::evaluate(ctx->inst, sol);
        
        std::cout << "[DEBUG to_solution] After Objectives::evaluate: " 
                  << sol.unassigned_customers.size() << " unassigned" << std::endl;
        
        if (sol.unassigned_customers.size() > added_customers.size() * 0.1) {
            // Significant loss - debug it
            std::cout << "[DEBUG to_solution] WARNING: Lost " 
                      << (sol.unassigned_customers.size()) 
                      << " customers after evaluate!" << std::endl;
            
            // Check if unassigned includes customers we added
            int wrongly_unassigned = 0;
            for (const auto& cid : sol.unassigned_customers) {
                if (added_customers.count(cid)) {
                    wrongly_unassigned++;
                }
            }
            if (wrongly_unassigned > 0) {
                std::cout << "[DEBUG to_solution] ERROR: " << wrongly_unassigned 
                          << " customers were in routes but marked unassigned!" << std::endl;
            }
        }
        
        return sol;
    }

    // ---------- objective ----------
    // authoritative recompute (use sparingly)
    // LEXICOGRAPHIC: Minimize unserved FIRST, then cost
    double recompute_objective() {
        fixed = travel = pen = 0.0;
        for (int cid = 0; cid < (int)clusters.size(); ++cid) {
            const auto& C = clusters[cid];
            if (!C.active || C.mem.empty()) continue;
            const auto& V = ctx->veh[C.vid];
            const auto& D = ctx->dep[C.did];
            fixed += V.fixed;
            if (C.center >= 0) {
                double mult = ctx->traffic_mult(D.id, ctx->cust[C.center].id);
                travel += C.trip_km * V.var * mult;
            }
        }
        
        // Count ACTUAL assigned customers (not just unassigned.size())
        int assigned_count = 0;
        for (int i = 0; i < (int)cust_cluster.size(); ++i) {
            if (cust_cluster[i] >= 0) assigned_count++;
        }
        int unserved_count = total_customer_count - assigned_count;
        
        // penalties for unassigned (for cost component)
        for (int ui : unassigned) {
            const auto& u = ctx->cust[ui];
            pen += PEN_BASE + PEN_W*u.w + PEN_V*u.v;
        }
        
        // LEXICOGRAPHIC OBJECTIVE: TRUE unserved count is primary, cost is secondary
        // Use massive multiplier to ensure unserved dominates
        obj = unserved_count * 1e10 + fixed + travel + pen;
        return obj;
    }

    // Incremental update for objective when a cluster changes
    // out_delta = new_obj - old_obj
    void incremental_update_objective(int cid, double old_trip_km, double new_trip_km, double* out_delta = nullptr) {
        if (cid < 0 || cid >= (int)clusters.size()) return;
        const auto& C = clusters[cid];
        const auto& V = ctx->veh[C.vid];
        const auto& D = ctx->dep[C.did];

        double old_val = 0, new_val = 0;
        if (old_trip_km > 1e-9 && C.center >= 0) {
            double mult = ctx->traffic_mult(D.id, ctx->cust[C.center].id);
            old_val = old_trip_km * V.var * mult;
        }
        if (new_trip_km > 1e-9 && C.center >= 0) {
            // After change, center might be different, but mult is usually similar.
            // Authority: we use ctx->cust[C.center].id
            double mult = ctx->traffic_mult(D.id, ctx->cust[C.center].id);
            new_val = new_trip_km * V.var * mult;
        }

        double delta = new_val - old_val;
        travel += delta;
        obj += delta;
        if (out_delta) *out_delta = delta;
    }

    // ---------- cluster recomputation + feasibility ----------
    bool choose_center(int cid, int& out_center, double& out_trip_km) const {
        const auto& C = clusters[cid];
        if (C.mem.empty()) { out_center=-1; out_trip_km=0; return true; }
        const auto& V = ctx->veh[C.vid];
        const auto& D = ctx->dep[C.did];
        double cx = C.sumx / std::max(1, C.n);
        double cy = C.sumy / std::max(1, C.n);

        struct Cand { double dMean; int ui; };
        std::vector<Cand> candidates;
        candidates.reserve(C.mem.size());
        for (int ui : C.mem) {
            double dm = std::hypot(ctx->cust[ui].x - cx, ctx->cust[ui].y - cy);
            candidates.push_back({dm, ui});
        }
        // Only check top 5 closest to mean for speed
        int take = std::min(5, (int)candidates.size());
        std::partial_sort(candidates.begin(), candidates.begin()+take, candidates.end(), [](auto& a, auto& b){ return a.dMean < b.dMean; });

        double limit_sq = std::pow(spread_limit_km(V.type), 2);
        for (int i=0; i<take; ++i) {
            int c_ui = candidates[i].ui;
            const auto& cu = ctx->cust[c_ui];
            if (!ctx->inst.edge_allowed(D.id, cu.id, V.type)) continue;
            double dk = ctx->inst.get_dist_km(D.id, cu.id);
            if (dk >= 1e8) continue;

            bool ok = true;
            for (int uj : C.mem) {
                const auto& m = ctx->cust[uj];
                double dx = cu.x - m.x, dy = cu.y - m.y;
                if (dx*dx + dy*dy > limit_sq + 1e-7) { ok=false; break; }
            }
            if (ok) {
                out_center = c_ui;
                out_trip_km = 2.0 * dk;
                return true;
            }
        }
        return false;
    }

    bool recompute_cluster(int cid) {
        auto& C = clusters[cid];
        if (C.mem.empty()) {
            C.n=0; C.sumx=C.sumy=0; C.load_w=C.load_v=0;
            C.center=-1; C.trip_km=0; C.dirty=true;
            return true;
        }
        C.n = (int)C.mem.size();
        C.sumx=C.sumy=0; C.load_w=C.load_v=0;
        for (int ui : C.mem) {
            const auto& u = ctx->cust[ui];
            C.sumx += u.x; C.sumy += u.y;
            C.load_w += u.w; C.load_v += u.v;
        }

        int center; double trip;
        if (!choose_center(cid, center, trip)) return false;
        C.center = center;
        C.trip_km = trip;
        C.dirty = true;
        return true;
    }

    bool check_cluster_caps(int cid) const {
        const auto& C = clusters[cid];
        if (C.mem.empty()) return true;
        const auto& V = ctx->veh[C.vid];
        return (C.load_w <= V.cap_w + 1e-9) && (C.load_v <= V.cap_v + 1e-9);
    }

    bool check_depot_caps(int did, double w, double v) const {
        const auto& D = ctx->dep[did];
        return (w <= D.cap_w + 1e-9) && (v <= D.cap_v + 1e-9);
    }

    bool check_vehicle_km(int vid, double km) const {
        return km <= ctx->veh[vid].max_dist + 1e-9;
    }

    bool customer_allowed_in_cluster(int ui, int cid) const {
        const auto& u = ctx->cust[ui];
        const auto& C = clusters[cid];
        const auto& V = ctx->veh[C.vid];
        if (ctx->p.enforce_territory) {
            if (!u.territory.empty() && u.territory != V.depot_id) return false;
        }
        if (ctx->p.enforce_class_req) {
            if (type_rank(V.type) < type_rank(u.class_req)) return false;
        }
        return true;
    }

    // ---------- boundary + neighbors ----------
    void refresh_neighbors(int cid) {
        auto& C = clusters[cid];
        C.nbr.clear();
        if (!C.active || C.mem.empty()) return;
        double cx = C.sumx / std::max(1, C.n);
        double cy = C.sumy / std::max(1, C.n);
        struct K { double d; int id; };
        std::vector<K> all;
        all.reserve(clusters.size());
        for (int j=0;j<(int)clusters.size();++j) {
            if (j==cid) continue;
            const auto& D = clusters[j];
            if (!D.active || D.mem.empty()) continue;
            if (D.did != C.did) continue;
            double dx = (D.sumx / std::max(1, D.n)) - cx;
            double dy = (D.sumy / std::max(1, D.n)) - cy;
            all.push_back({std::hypot(dx,dy), j});
        }
        std::sort(all.begin(), all.end(), [](const K& a, const K& b){ return a.d < b.d; });
        int take = std::min(ctx->p.nbr_k, (int)all.size());
        for (int i=0;i<take;++i) C.nbr.push_back(all[i].id);
    }

    void refresh_boundary(int cid) {
        auto& C = clusters[cid];
        C.boundary.clear();
        if (!C.active || C.mem.empty()) return;
        if (C.nbr.empty()) refresh_neighbors(cid);
        if (C.nbr.empty()) return;
        double cx = C.sumx / std::max(1, C.n);
        double cy = C.sumy / std::max(1, C.n);
        struct B { double score; int ui; };
        std::vector<B> Bvec;
        Bvec.reserve(C.mem.size());
        for (int ui : C.mem) {
            double dC = std::hypot(ctx->cust[ui].x - cx, ctx->cust[ui].y - cy);
            double best = 1e18;
            for (int nb : C.nbr) {
                const auto& D = clusters[nb];
                double dx = ctx->cust[ui].x - (D.sumx / std::max(1, D.n));
                double dy = ctx->cust[ui].y - (D.sumy / std::max(1, D.n));
                best = std::min(best, std::hypot(dx,dy));
            }
            Bvec.push_back({std::fabs(dC - best), ui});
        }
        int K = (int)std::round(ctx->p.boundary_frac * (double)C.mem.size());
        K = std::max(ctx->p.boundary_min, std::min(ctx->p.boundary_max, K));
        K = std::min(K, (int)Bvec.size());
        std::nth_element(Bvec.begin(), Bvec.begin()+K, Bvec.end(),
                         [](const B& a, const B& b){ return a.score < b.score; });
        std::sort(Bvec.begin(), Bvec.begin()+K, [](const B& a, const B& b){ return a.score < b.score; });
        for (int i=0;i<K;++i) C.boundary.push_back(Bvec[i].ui);
    }

    void refresh_all_neighbors_and_boundaries() {
        for (int cid=0; cid<(int)clusters.size(); ++cid) {
            refresh_neighbors(cid);
        }
        for (int cid=0; cid<(int)clusters.size(); ++cid) {
            refresh_boundary(cid);
            clusters[cid].dirty = false;
        }
    }

    // ---------- primitives: remove / insert ----------
    void remove_customer_from_cluster(int ui, int cid) {
        auto& C = clusters[cid];
        auto it = std::find(C.mem.begin(), C.mem.end(), ui);
        if (it == C.mem.end()) return;
        
        const auto& u = ctx->cust[ui];
        double old_trip_km = C.trip_km;
        bool was_empty = C.mem.empty();

        // incremental update for penalties
        double pen_val = PEN_BASE + PEN_W*u.w + PEN_V*u.v;
        pen += pen_val;
        obj += pen_val;

        depot_w[C.did] -= u.w;
        depot_v[C.did] -= u.v;
        veh_km[C.vid] -= old_trip_km;

        C.mem.erase(it);
        cust_cluster[ui] = -1;
        unassigned.insert(ui);

        if (recompute_cluster(cid) && !C.mem.empty()) {
            veh_km[C.vid] += C.trip_km;
            incremental_update_objective(cid, old_trip_km, C.trip_km);
        } else {
            // Cluster became empty
            if (!was_empty) {
                const auto& V = ctx->veh[C.vid];
                fixed -= V.fixed;
                obj -= V.fixed;
                // travel already updated by subtracting old_trip_km above
            }
        }
        C.dirty = true;
    }

    bool tabu_forbidden(int ui, int cid, int iter) const {
        long long key = ((long long)ui<<32) ^ (unsigned int)cid;
        auto it = tabu_until.find(key);
        return (it != tabu_until.end() && it->second > iter);
    }

    void tabu_add(int ui, int cid, int iter) {
        long long key = ((long long)ui<<32) ^ (unsigned int)cid;
        tabu_until[key] = iter + ctx->p.tabu_ttl;
    }

    // Non-destructive check: returns true if inserting ui into cid is feasible.
    // Also returns the objective delta if requested.
    bool check_insert_feasibility(int ui, int cid, int iter, double* out_delta = nullptr) const {
        if (ui<0 || cid<0 || cid>=(int)clusters.size()) return false;
        if (cust_cluster[ui] != -1) return false;
        const auto& C = clusters[cid];
        if (!C.active) return false;
        if (!customer_allowed_in_cluster(ui, cid)) return false;
        if (tabu_forbidden(ui, cid, iter)) return false;

        const auto& u = ctx->cust[ui];
        const auto& V = ctx->veh[C.vid];

        // 1. Capacity checks - SOFT CONSTRAINTS
        if (C.load_w + u.w > V.cap_w + 1e-9) return false;  // Vehicle cap still hard
        if (C.load_v + u.v > V.cap_v + 1e-9) return false;  // Vehicle cap still hard
        // SOFT: Allow depot cap violation up to 20% (will be penalized in Objectives::evaluate)
        double depot_limit = ctx->dep[C.did].cap_w * 1.2; // 20% tolerance
        if (depot_w[C.did] + u.w > depot_limit + 1e-9) return false;

        // 2. Spread & Distance (requires tentative center)
        // For speed, we estimate the center. Exact check:
        double new_sumx = C.sumx + u.x;
        double new_sumy = C.sumy + u.y;
        int new_n = C.n + 1;
        double cx = new_sumx / new_n;
        double cy = new_sumy / new_n;

        // pick center closest to mean among a small subset of members + u
        int best_ui = -1;
        double best_d_mean = 1e18;
        double limit_sq = std::pow(spread_limit_km(V.type), 2);
        
        auto look = [&](int uj, double xj, double yj) {
            // O(1) Pre-filter 1: candidate must be within limit of the NEW member
            double dx_u = xj - u.x, dy_u = yj - u.y;
            if (dx_u*dx_u + dy_u*dy_u > limit_sq + 1e-7) return false;
            
            // O(1) Pre-filter 2: check if it's actually allowed (could cache this but get_dist is okay)
            if (!ctx->inst.edge_allowed(ctx->dep[C.did].id, ctx->cust[uj].id, V.type)) return false;
            double dj = ctx->inst.get_dist_km(ctx->dep[C.did].id, ctx->cust[uj].id);
            if (dj >= 1e8) return false;
            
            // O(N) Scan members
            for (int mem_id : C.mem) {
                const auto& m = ctx->cust[mem_id];
                double dx = xj - m.x, dy = yj - m.y;
                if (dx*dx + dy*dy > limit_sq + 1e-7) return false;
            }
            
            double d_mean = std::hypot(xj - cx, yj - cy);
            if (d_mean < best_d_mean) {
                best_d_mean = d_mean;
                best_ui = uj;
            }
            return true;
        };

        // Candidates: 1. Current center, 2. New guy, 3. Few closets to mean
        if (C.center >= 0) look(C.center, ctx->cust[C.center].x, ctx->cust[C.center].y);
        look(ui, u.x, u.y);
        
        if (best_ui == -1 || best_d_mean > 0.1) { // if current/new not perfect, check few more
            struct M { double d; int id; };
            std::vector<M> nears;
            for (int m_id : C.mem) {
                const auto& m = ctx->cust[m_id];
                nears.push_back({std::hypot(m.x - cx, m.y - cy), m_id});
            }
            int take = std::min(5, (int)nears.size());
            std::nth_element(nears.begin(), nears.begin()+take, nears.end(), [](auto& a, auto& b){ return a.d < b.d; });
            for (int i=0; i<take; ++i) {
                int mid = nears[i].id;
                if (mid == C.center) continue;
                const auto& m = ctx->cust[mid];
                look(mid, m.x, m.y);
            }
        }

        if (best_ui == -1) return false;
        
        double trip_km = 2.0 * ctx->inst.get_dist_km(ctx->dep[C.did].id, ctx->cust[best_ui].id);
        // SOFT: Allow max_dist violation up to 10% (will be penalized in Objectives::evaluate)
        double dist_limit = V.max_dist * 1.1; // 10% tolerance
        if (veh_km[C.vid] - C.trip_km + trip_km > dist_limit + 1e-9) return false;

        if (out_delta) {
            double pen_val = PEN_BASE + PEN_W*u.w + PEN_V*u.v;
            double dPEN = -pen_val;
            double dFIXED = (C.n == 0) ? V.fixed : 0.0;
            
            double old_trip_val = 0;
            if (C.n > 0 && C.center >= 0) {
                double mult = ctx->traffic_mult(ctx->dep[C.did].id, ctx->cust[C.center].id);
                old_trip_val = C.trip_km * V.var * mult;
            }
            double mult = ctx->traffic_mult(ctx->dep[C.did].id, ctx->cust[best_ui].id);
            double new_trip_val = trip_km * V.var * mult;
            double dTRAV = new_trip_val - old_trip_val;

            *out_delta = dPEN + dFIXED + dTRAV;
        }

        return true;
    }


    bool check_remove_delta(int ui, int cid, double* out_delta = nullptr) const {
        if (ui<0 || cid<0 || cid>=(int)clusters.size()) return false;
        const auto& C = clusters[cid];
        if (C.mem.size() <= 0) return false;
        
        const auto& u = ctx->cust[ui];
        const auto& V = ctx->veh[C.vid];
        
        // Base penalty delta
        double pen_val = PEN_BASE + PEN_W*u.w + PEN_V*u.v;
        double dPEN = pen_val;
        
        // Fixed cost delta
        double dFIXED = (C.n == 1) ? -V.fixed : 0.0;
        
        // Travel cost delta (requires tentative center of remaining)
        // For remove, we estimate the new center by just keeping the old one if it's still a member
        // or picking a random one if the old one was 'ui'.
        // For speed, let's just use a simplified delta.
        
        double old_trip_val = 0;
        if (C.center >= 0) {
            double mult = ctx->traffic_mult(ctx->dep[C.did].id, ctx->cust[C.center].id);
            old_trip_val = C.trip_km * V.var * mult;
        }
        
        double new_trip_val = 0;
        if (C.n > 1) {
            // Pick a new center candidate (simply first member that is not ui)
            int new_c = -1;
            for (int m : C.mem) if (m != ui) { new_c = m; break; }
            if (new_c != -1) {
                double dnew = ctx->inst.get_dist_km(ctx->dep[C.did].id, ctx->cust[new_c].id);
                double m2 = ctx->traffic_mult(ctx->dep[C.did].id, ctx->cust[new_c].id);
                new_trip_val = 2.0 * dnew * V.var * m2;
            }
        }
        
        double dTRAV = new_trip_val - old_trip_val;
        if (out_delta) *out_delta = dPEN + dFIXED + dTRAV;
        return true;
    }

    // Try insert ui into cluster cid; returns true if applied.
    bool try_insert(int ui, int cid, int iter, double* out_delta=nullptr) {
        if (ui<0 || cid<0 || cid>=(int)clusters.size()) return false;
        if (cust_cluster[ui] != -1) return false;
        if (!customer_allowed_in_cluster(ui, cid)) return false;
        if (tabu_forbidden(ui, cid, iter)) return false;

        Cluster& C = clusters[cid];
        if (!C.active) return false;
        const auto& u = ctx->cust[ui];
        const auto& V = ctx->veh[C.vid];

        double old_obj = obj;
        double old_trip_km = C.trip_km;
        bool was_empty = C.mem.empty();

        // tentative changes
        C.mem.push_back(ui);
        if (!recompute_cluster(cid)) {
            C.mem.pop_back();
            return false;
        }

        // feasibility checks
        double new_load_w = C.load_w;
        double new_load_v = C.load_v;
        double new_dep_w = depot_w[C.did] + u.w;
        double new_dep_v = depot_v[C.did] + u.v;
        double new_veh_km = veh_km[C.vid] - old_trip_km + C.trip_km;

        if (new_load_w > V.cap_w + 1e-9 || new_load_v > V.cap_v + 1e-9 ||
            !check_depot_caps(C.did, new_dep_w, new_dep_v) ||
            !check_vehicle_km(C.vid, new_veh_km)) {
            // rollback recompute
            C.mem.pop_back();
            recompute_cluster(cid);
            return false;
        }

        // Apply changes
        cust_cluster[ui] = cid;
        unassigned.erase(ui);
        depot_w[C.did] = new_dep_w;
        depot_v[C.did] = new_dep_v;
        veh_km[C.vid] = new_veh_km;

        // incremental objective update
        double pen_val = PEN_BASE + PEN_W*u.w + PEN_V*u.v;
        pen -= pen_val;
        obj -= pen_val;

        if (was_empty) {
            fixed += V.fixed;
            obj += V.fixed;
        }
        
        incremental_update_objective(cid, old_trip_km, C.trip_km);

        if (out_delta) *out_delta = obj - old_obj;
        C.dirty = true;
        return true;
    }

    // Force-insert a customer into any cluster, even if it violates soft constraints.
    // This adds heavy penalties for violations instead of rejecting.
    bool try_force_insert(int ui, int cid, int iter) {
        if (ui<0 || cid<0 || cid>=(int)clusters.size()) return false;
        if (cust_cluster[ui] != -1) return false;

        Cluster& C = clusters[cid];
        if (!C.active) return false;
        if (C.mem.empty()) return false;  // Need at least one customer to have a center
        
        const auto& u = ctx->cust[ui];
        const auto& V = ctx->veh[C.vid];

        // Direct manual update - bypass recompute_cluster entirely
        C.mem.push_back(ui);
        C.n = (int)C.mem.size();
        C.sumx += u.x;
        C.sumy += u.y;
        C.load_w += u.w;
        C.load_v += u.v;
        // Keep existing center - don't recalculate

        // Calculate new vehicle km (simple approximation - use current trip_km)
        double new_veh_km = veh_km[C.vid];  // Keep same km since we're not changing center
        
        // Calculate violation penalties
        double violation_penalty = 0;
        if (C.load_w > V.cap_w + 1e-9) {
            violation_penalty += 10000.0 * (C.load_w - V.cap_w);
        }
        if (C.load_v > V.cap_v + 1e-9) {
            violation_penalty += 10000.0 * (C.load_v - V.cap_v);
        }
        if (new_veh_km > V.max_dist + 1e-9) {
            violation_penalty += 10000.0 * (new_veh_km - V.max_dist);
        }

        // Apply changes
        cust_cluster[ui] = cid;
        unassigned.erase(ui);
        depot_w[C.did] += u.w;
        depot_v[C.did] += u.v;

        // Update objective - remove unserved penalty (NO violation penalty - we WANT this to be better)
        double pen_val = PEN_BASE + PEN_W*u.w + PEN_V*u.v;
        pen -= pen_val;
        obj -= pen_val;
        // Removed: obj += violation_penalty;  -- this prevented best from being updated

        C.dirty = true;
        return true;
    }
};

// ==============================================================
// Operator weights (simple EMA)
// ==============================================================
struct Weights {
    std::vector<double> wd = {1.0, 1.0, 0.8, 1.0, 0.5}; // 5 destroy ops (reduced WholeCluster & ShuffleAll)
    std::vector<double> wr = {2.5, 3.0, 3.0, 1.5, 2.0}; // 5 repair ops (GREEDY PRIORITIZED: GrowBatch=3.0, Greedy=3.0)
    double alpha = 0.2;
    void reward_destroy(DestroyOp op, double s) {
        int i=(int)op; wd[i] = (1.0-alpha)*wd[i] + alpha*std::max(0.1, wd[i] + s);
    }
    void reward_repair(RepairOp op, double s) {
        int i=(int)op; wr[i] = (1.0-alpha)*wr[i] + alpha*std::max(0.1, wr[i] + s);
    }
};

// ==============================================================
// Optimizer
// ==============================================================
class Optimizer {
public:
    explicit Optimizer(Params p = {}) : p_(p) {}

    Solution solve(const Instance& inst_in, const Solution& init) {
        Context ctx(inst_in, p_);
        std::mt19937 rng(p_.seed);
        Weights W;

        State cur = State::from_solution(ctx, init);
        State best = cur;
        double best_obj = best.obj;
        double T = p_.T0;

        // DEBUG: Check initial state
        int init_clusters = 0, init_customers = 0;
        for (const auto& C : cur.clusters) {
            if (C.active && !C.mem.empty()) {
                init_clusters++;
                init_customers += C.mem.size();
            }
        }
        std::cout << "[DEBUG] Initial State: " << init_clusters << " clusters, " 
                  << init_customers << " customers, " 
                  << cur.unassigned.size() << " unassigned" << std::endl;

        std::cout << "[ALNS] Starting optimizer (p.iters=" << p_.iters << ", initial_obj=" << cur.obj << ")" << std::endl;

        for (int iter=1; iter<=p_.iters; ++iter) {
            DestroyOp dop = (DestroyOp)sample_discrete(rng, W.wd);
            RepairOp  rop = (RepairOp)sample_discrete(rng, W.wr);

            State cand = cur;
            std::unordered_set<int> touched;
            
            // Validation: ALL customers must be either assigned or unassigned
            auto validate_customers = [](const State& st, const char* label) {
                int total_customers = (int)st.ctx->cust.size();
                int assigned = 0;
                int unassigned_count = (int)st.unassigned.size();
                
                for (int i = 0; i < (int)st.cust_cluster.size(); ++i) {
                    if (st.cust_cluster[i] >= 0) assigned++;
                }
                
                int accounted = assigned + unassigned_count;
                if (accounted != total_customers) {
                    std::cout << "\n[CRITICAL ERROR] " << label << ": LOST CUSTOMERS!" 
                              << "\n  Total should be: " << total_customers
                              << "\n  Assigned: " << assigned
                              << "\n  Unassigned: " << unassigned_count  
                              << "\n  Accounted: " << accounted
                              << "\n  LOST: " << (total_customers - accounted) << std::endl;
                    return false;
                }
                return true;
            };
            
            validate_customers(cand, "Before destroy");
            
            // Log every 10 iterations to show some life
            if (iter <= 10 || iter % 100 == 0) {
                std::cout << "[ALNS] Iter " << iter << " | Op: D=" << (int)dop << " R=" << (int)rop << "..." << std::flush;
            }

            destroy(cand, dop, rng, touched);
            
            if (!validate_customers(cand, "After destroy")) {
                std::cout << "  Destroy Op: " << (int)dop << std::endl;
                break; // Stop iteration to debug
            }
            
            repair(cand, rop, rng, iter, touched);
            
            if (!validate_customers(cand, "After repair")) {
                std::cout << "  Repair Op: " << (int)rop << std::endl;
                break; // Stop iteration to debug
            }
            
            if (iter <= 10 || iter % 100 == 0) std::cout << " VND(" << touched.size() << ")..." << std::flush;
            vnd(cand, rng, iter, touched);
            if (iter <= 10 || iter % 100 == 0) std::cout << " done." << std::endl;

            // redundant but safe recompute for main loop
            double old_obj = cur.obj;
            double new_obj = cand.recompute_objective();

            bool accept = false;
            if (new_obj < old_obj - 1e-9) accept = true;
            else {
                double delta = new_obj - old_obj;
                std::uniform_real_distribution<double> uni(0.0, 1.0);
                double pr = std::exp(-delta / std::max(1e-9, T));
                if (uni(rng) < pr) accept = true;
            }

            double score = 0.0;
            if (new_obj < best_obj - 1e-9) score = 5.0;
            else if (new_obj < old_obj - 1e-9) score = 2.0;
            else if (accept) score = 0.5;
            W.reward_destroy(dop, score);
            W.reward_repair(rop, score);

            if (accept) {
                cur = std::move(cand);
                
                // CRITICAL VALIDATION: Check customer count after accept
                int total_in_instance = (int)ctx.cust.size();
                int assigned = 0;
                for (int i = 0; i < (int)cur.cust_cluster.size(); ++i) {
                    if (cur.cust_cluster[i] >= 0) assigned++;
                }
                int unassigned_count = (int)cur.unassigned.size();
                int total_tracked = assigned + unassigned_count;
                
                if (total_tracked != total_in_instance) {
                    std::cout << "\n[CRITICAL BUG] Customer loss detected after accept!" 
                              << "\n  Instance total: " << total_in_instance
                              << "\n  Assigned: " << assigned
                              << "\n  Unassigned: " << unassigned_count
                              << "\n  Total tracked: " << total_tracked
                              << "\n  LOST: " << (total_in_instance - total_tracked) 
                              << "\n  Iteration: " << iter << std::endl;
                    // Don't break, but log heavily
                }
            }
            
            // SOFT CONSTRAINTS: Accept solutions even with violations if they serve more customers
            if (cur.obj < best_obj - 1e-9) {
                best = cur; 
                best_obj = cur.obj;
                
                // DEBUG: Count routes in best
                int best_clusters = 0, best_customers = 0;
                for (const auto& C : best.clusters) {
                    if (C.active && !C.mem.empty()) {
                        best_clusters++;
                        best_customers += C.mem.size();
                    }
                }
                std::cout << "[ALNS] Iter " << iter << " New best obj: " << best_obj 
                          << " | Clusters: " << best_clusters 
                          << ", Customers: " << best_customers 
                          << ", Unassigned: " << best.unassigned.size() << std::endl;
            }
            if (iter % 100 == 0) {
                 std::cout << "[ALNS] Iter " << iter << " Current obj: " << cur.obj << " T: " << T << std::endl;
            }
            T *= p_.cooling;
        }

        // DEBUG: Final best state before conversion
        int final_clusters = 0, final_customers = 0;
        for (const auto& C : best.clusters) {
            if (C.active && !C.mem.empty()) {
                final_clusters++;
                final_customers += C.mem.size();
            }
        }
        std::cout << "[DEBUG] Final Best State: " << final_clusters << " clusters, " 
                  << final_customers << " customers, " 
                  << best.unassigned.size() << " unassigned, obj=" << best_obj << std::endl;

        Solution out = best.to_solution();
        // authoritative evaluation using original inst (same distances)
        Objectives::evaluate(inst_in, out);
        
        // DEBUG: Check converted solution
        int sol_routes = 0;
        for (const auto& kv : out.routes) {
            sol_routes += kv.second.size();
        }
        std::cout << "[DEBUG] Converted Solution: " << sol_routes << " routes, " 
                  << out.unassigned_customers.size() << " unassigned" << std::endl;
        
        return out;
    }

private:
    Params p_;

    // ---------- destroy operators ----------
    static bool is_stable_small_cluster(const State& st, int cid) {
        const auto& C = st.clusters[cid];
        if (!C.active || C.mem.empty()) return false;
        if (C.mem.size() >= 10) return false;
        // Check feasibility: if it's already over cap, it's not stable
        const auto& V = st.ctx->veh[C.vid];
        if (C.load_w > V.cap_w + 1e-9 || C.load_v > V.cap_v + 1e-9) return false;
        if (st.veh_km[C.vid] > V.max_dist + 1e-9) return false;
        return true;
    }

    static int pick_nonempty_cluster(State& st, std::mt19937& rng) {
        std::vector<int> ids;
        for (int cid=0; cid<(int)st.clusters.size(); ++cid)
            if (st.clusters[cid].active && !st.clusters[cid].mem.empty()) ids.push_back(cid);
        if (ids.empty()) return -1;
        std::uniform_int_distribution<int> uni(0, (int)ids.size()-1);
        return ids[uni(rng)];
    }

    static void destroy(State& st, DestroyOp op, std::mt19937& rng, std::unordered_set<int>& touched) {
        if (st.clusters.empty()) return;

        // Refresh boundaries if needed for all clusters touched in previous iters
        for (auto& C : st.clusters) {
            if (C.dirty) { st.refresh_neighbors((int)(&C - st.clusters.data())); st.refresh_boundary((int)(&C - st.clusters.data())); C.dirty = false; }
        }

        if (op == DestroyOp::Boundary) {
            // choose some clusters (biased by long trip)
            std::vector<std::pair<double,int>> cand;
            for (int cid=0; cid<(int)st.clusters.size(); ++cid) {
                const auto& C = st.clusters[cid];
                if (!C.active || C.mem.empty()) continue;
                if (is_stable_small_cluster(st, cid)) continue; // PROTECT
                cand.push_back({C.trip_km, cid});
            }
            if (cand.empty()) {
                // If all are small/stable, maybe just pick one anyway otherwise we do nothing
                int c = pick_nonempty_cluster(st, rng);
                if (c >= 0) cand.push_back({0.0, c});
            }
            if (cand.empty()) return;
            std::sort(cand.begin(), cand.end(), std::greater<>());
            int m = std::min(3, (int)cand.size());
            std::uniform_int_distribution<int> uni(0, m-1);
            int chosen = cand[uni(rng)].second;

            auto& C = st.clusters[chosen];
            if (C.boundary.empty()) st.refresh_boundary(chosen);

            // remove all boundary points
            for (int ui : C.boundary) {
                if (st.cust_cluster[ui] == chosen) {
                    st.remove_customer_from_cluster(ui, chosen);
                }
            }
            touched.insert(chosen);
        }
        else if (op == DestroyOp::Patch) {
            int cid = pick_nonempty_cluster(st, rng);
            if (cid < 0) return;
            auto& C = st.clusters[cid];
            if (C.boundary.empty()) st.refresh_boundary(cid);
            if (C.boundary.empty()) return;

            // seed from boundary
            std::uniform_int_distribution<int> uni(0, (int)C.boundary.size()-1);
            int seed_u = C.boundary[uni(rng)];
            int target = (int)std::round(st.ctx->p.patch_frac * (double)C.mem.size());
            int pmin = st.ctx->p.patch_min;
            int pmax = (int)std::floor(st.ctx->p.patch_max_frac * (double)C.mem.size());
            target = std::max(pmin, std::min(pmax, target));
            target = std::min(target, (int)C.mem.size());

            // BFS over kNN restricted to members
            std::unordered_set<int> inC(C.mem.begin(), C.mem.end());
            std::vector<int> patch;
            patch.reserve(target);
            std::deque<int> dq;
            dq.push_back(seed_u);
            std::unordered_set<int> seen;
            seen.insert(seed_u);
            while (!dq.empty() && (int)patch.size() < target) {
                int u = dq.front(); dq.pop_front();
                if (!inC.count(u)) continue;
                patch.push_back(u);
                for (int nb : st.ctx->knn[u]) {
                    if (seen.insert(nb).second) dq.push_back(nb);
                }
            }
            for (int ui : patch) {
                if (st.cust_cluster[ui] == cid) st.remove_customer_from_cluster(ui, cid);
            }
            touched.insert(cid);
        }
        else if (op == DestroyOp::Greedy) {
            // Liquidate expensive clusters (high cost per member) - now safe with C.center checks
            struct S { double ratio; int cid; };
            std::vector<S> scores;
            for (int cid=0; cid<(int)st.clusters.size(); ++cid) {
                const auto& C = st.clusters[cid];
                if (C.active && !C.mem.empty() && C.center >= 0) {
                    // Skip very small clusters to avoid thrashing
                    if ((int)C.mem.size() <= 2) continue;
                    scores.push_back({ C.trip_km / (double)C.mem.size(), cid });
                }
            }
            if (scores.empty()) return;
            std::sort(scores.begin(), scores.end(), [](auto& a, auto& b){ return a.ratio > b.ratio; });
            
            // Destroy top 2 expensive clusters
            int liquidations = std::min(2, (int)scores.size());
            for (int i=0; i<liquidations; ++i) {
                int cid = scores[i].cid;
                auto mem = st.clusters[cid].mem; // copy
                for (int ui : mem) {
                    if (st.cust_cluster[ui] == cid) st.remove_customer_from_cluster(ui, cid);
                }
                touched.insert(cid);
            }
        }
        else if (op == DestroyOp::ShuffleAll) {
            // REDUCED: Remove random 15% of ALL assigned customers (reduced from 30%)
            std::vector<int> assigned;
            for (int i = 0; i < (int)st.cust_cluster.size(); ++i) {
                if (st.cust_cluster[i] >= 0) assigned.push_back(i);
            }
            if (assigned.empty()) return;
            
            std::shuffle(assigned.begin(), assigned.end(), rng);
            int to_remove = std::max(3, (int)(assigned.size() * 0.15));
            to_remove = std::min(to_remove, (int)assigned.size());
            
            for (int i = 0; i < to_remove; ++i) {
                int ui = assigned[i];
                int cid = st.cust_cluster[ui];
                if (cid >= 0 && cid < (int)st.clusters.size()) {
                    st.remove_customer_from_cluster(ui, cid);
                    touched.insert(cid);
                }
            }
        }
        else { // WholeCluster
            int cid = pick_nonempty_cluster(st, rng);
            if (cid < 0) return;
            auto mem = st.clusters[cid].mem; // copy
            for (int ui : mem) {
                if (st.cust_cluster[ui] == cid) st.remove_customer_from_cluster(ui, cid);
            }
            touched.insert(cid);
        }
    }

    static void greedy_repair(State& st, int iter, std::unordered_set<int>& touched) {
        // Simple cheapest insertion among active clusters
        std::vector<int> U(st.unassigned.begin(), st.unassigned.end());
        std::shuffle(U.begin(), U.end(), std::mt19937(iter));

        for (int ui : U) {
            if (!st.unassigned.count(ui)) continue;
            int best_cid = -1;
            double best_d = 1e18;
            for (int cid=0; cid<(int)st.clusters.size(); ++cid) {
                if (!st.clusters[cid].active || st.clusters[cid].mem.empty()) continue;
                if (!st.customer_allowed_in_cluster(ui, cid)) continue;
                double d = 0;
                if (st.check_insert_feasibility(ui, cid, iter, &d)) {
                    if (d < best_d) { best_d = d; best_cid = cid; }
                }
            }
            if (best_cid != -1) {
                double dummy;
                if (st.try_insert(ui, best_cid, iter, &dummy)) touched.insert(best_cid);
            }
        }
    }

    // Drain operator: try to relocate all members of small clusters to other trips
    // If a cluster is emptied, it is effectively "killed".
    static void drain_small_clusters(State& st, std::mt19937& rng, int iter,
                                     std::unordered_set<int>& touched) {
        // Find small clusters (<=5 members) sorted by size ascending
        struct Cand { int size; int cid; };
        std::vector<Cand> cands;
        for (int cid=0; cid<(int)st.clusters.size(); ++cid) {
            const auto& C = st.clusters[cid];
            if (C.active && !C.mem.empty() && (int)C.mem.size() <= 5) {
                cands.push_back({(int)C.mem.size(), cid});
            }
        }
        std::sort(cands.begin(), cands.end(), [](auto& a, auto& b){ return a.size < b.size; });

        int drained_count = 0;
        for (auto& [sz, src_cid] : cands) {
            if (drained_count >= 3) break; // Limit drain attempts per iteration
            auto& srcC = st.clusters[src_cid];
            if (!srcC.active || srcC.mem.empty()) continue;

            std::vector<int> mem_copy = srcC.mem;
            bool all_relocated = true;

            for (int ui : mem_copy) {
                if (st.cust_cluster[ui] != src_cid) continue; // Already moved

                // Find best alternative cluster for this customer
                int best_cid = -1;
                double best_d = 1e18;
                for (int cid=0; cid<(int)st.clusters.size(); ++cid) {
                    if (cid == src_cid) continue;
                    if (!st.clusters[cid].active || st.clusters[cid].mem.empty()) continue;
                    if (!st.customer_allowed_in_cluster(ui, cid)) continue;
                    double d = 0;
                    if (st.check_insert_feasibility(ui, cid, iter, &d)) {
                        if (d < best_d) { best_d = d; best_cid = cid; }
                    }
                }

                if (best_cid != -1) {
                    st.remove_customer_from_cluster(ui, src_cid);
                    double dummy;
                    st.try_insert(ui, best_cid, iter, &dummy);
                    touched.insert(best_cid);
                } else {
                    all_relocated = false;
                }
            }

            if (all_relocated && srcC.mem.empty()) {
                // Cluster successfully drained and killed
                drained_count++;
                touched.insert(src_cid);
            }
        }
    }

    // ---------- repair operators ----------
    static void repair(State& st, RepairOp op, std::mt19937& rng, int iter,
                       std::unordered_set<int>& touched) {
        if (st.unassigned.empty()) return;

        // EXHAUSTIVE REPAIR: Loop until we can't insert any more customers
        int max_passes = 10;  // Prevent infinite loops
        for (int pass = 0; pass < max_passes; ++pass) {
            size_t unassigned_before = st.unassigned.size();
            
            // 1) directional grow around touched clusters first
            std::vector<int> cluster_order;
            cluster_order.reserve(st.clusters.size());
            for (int cid=0; cid<(int)st.clusters.size(); ++cid)
                if (st.clusters[cid].active && !st.clusters[cid].mem.empty()) cluster_order.push_back(cid);
            std::shuffle(cluster_order.begin(), cluster_order.end(), rng);

            if (op == RepairOp::Greedy) {
                greedy_repair(st, iter, touched);
            } else if (op == RepairOp::Drain) {
                // Drain operator: relocate members of small clusters elsewhere
                drain_small_clusters(st, rng, iter, touched);
            } else if (op == RepairOp::Regret) {
                // REGRET-2 REPAIR: Insert customer with highest regret first
                // Regret = delta(2nd best) - delta(best). High regret = critical to place now
                while (!st.unassigned.empty()) {
                    int best_ui = -1;
                    int best_cid = -1;
                    double max_regret = -1e18;
                    double best_delta = 1e18;
                    
                    for (int ui : st.unassigned) {
                        double d1 = 1e18, d2 = 1e18; // best and 2nd best
                        int c1 = -1;
                        
                        for (int cid = 0; cid < (int)st.clusters.size(); ++cid) {
                            if (!st.clusters[cid].active || st.clusters[cid].mem.empty()) continue;
                            if (!st.customer_allowed_in_cluster(ui, cid)) continue;
                            double d = 0;
                            if (st.check_insert_feasibility(ui, cid, iter, &d)) {
                                if (d < d1) { d2 = d1; d1 = d; c1 = cid; }
                                else if (d < d2) { d2 = d; }
                            }
                        }
                        
                        if (c1 >= 0) {
                            double regret = (d2 < 1e17) ? (d2 - d1) : 1e15; // high regret if only 1 option
                            if (regret > max_regret) {
                                max_regret = regret;
                                best_ui = ui;
                                best_cid = c1;
                                best_delta = d1;
                            }
                        }
                    }
                    
                    if (best_ui < 0 || best_cid < 0) break;
                    
                    double dummy;
                    if (st.try_insert(best_ui, best_cid, iter, &dummy)) {
                        touched.insert(best_cid);
                    } else {
                        break; // Shouldn't happen but safety
                    }
                }
            } else {
                int batch = (op==RepairOp::GrowBatch) ? st.ctx->p.batch_M : 1;
                // Linearized repair: try each cluster once
                for (int cid : cluster_order) {
                    if (st.unassigned.empty()) break;
                    if (!st.clusters[cid].active || st.clusters[cid].mem.empty()) continue;
                    int inserted = grow_one_cluster(st, cid, rng, iter, batch);
                    if (inserted > 0) touched.insert(cid);
                }
            }

            // 2) if still unassigned, try ejection insert (simple, chain limited)
            if (st.ctx->p.allow_ejection && !st.unassigned.empty()) {
                std::vector<int> U(st.unassigned.begin(), st.unassigned.end());
                std::shuffle(U.begin(), U.end(), rng);
                for (int ui : U) {
                    if (!st.unassigned.count(ui)) continue;
                    if (try_ejection(st, ui, rng, iter, touched)) {
                        // assigned via ejection
                    }
                }
            }

            // 3) AGGRESSIVE STEALING: Try to steal light customers from clusters to make room
            if (!st.unassigned.empty()) {
                steal_to_make_room(st, rng, iter, touched);
            }
            
            // Check if we made progress
            size_t unassigned_after = st.unassigned.size();
            if (unassigned_after >= unassigned_before) {
                // No progress, stop looping
                break;
            }
            
            // If all assigned, done
            if (st.unassigned.empty()) break;
        }

        // 4) open trips while unassigned and possible - VERY AGGRESSIVE
        if (st.ctx->p.allow_open_trip) {
            int total_customers = (int)st.ctx->cust.size();
            double unassigned_ratio = (double)st.unassigned.size() / std::max(1, total_customers);
            
            // Open trips more aggressively when many unassigned
            int max_attempts = (unassigned_ratio > st.ctx->p.open_trip_trigger) ? 200 : 100;
            int attempts = 0;
            
            while (!st.unassigned.empty() && attempts < max_attempts) {
                bool opened = open_trip(st, rng, iter, touched);
                if (!opened) break; // Can't open more trips
                attempts++;
            }
        }

        // 5) MAXIMIZE CLUSTER SIZES by stealing from neighbors
        maximize_clusters_by_stealing(st, rng, iter, touched);

        // NOTE: Customers that cannot be served remain unassigned with penalty.
        // This is the correct behavior - forcing them into overloaded clusters
        // creates infeasible solutions. The objective function already includes
        // heavy penalties for unserved customers.
    }

    // Steal light customers from clusters to make room for heavy unassigned ones
    static void steal_to_make_room(State& st, std::mt19937& rng, int iter,
                                   std::unordered_set<int>& touched) {
        // Sort unassigned by weight DESC (prioritize heavy customers)
        std::vector<std::pair<double, int>> u_sorted;
        for (int ui : st.unassigned) {
            const auto& u = st.ctx->cust[ui];
            u_sorted.push_back({u.w, ui});
        }
        std::sort(u_sorted.begin(), u_sorted.end(), std::greater<>());
        
        int steal_limit = std::min(50, (int)u_sorted.size());
        for (int i = 0; i < steal_limit; ++i) {
            int ui = u_sorted[i].second;
            if (!st.unassigned.count(ui)) continue;
            
            const auto& u = st.ctx->cust[ui];
            
            // Find clusters where this customer could fit if we remove light members
            for (int cid = 0; cid < (int)st.clusters.size(); ++cid) {
                auto& C = st.clusters[cid];
                if (!C.active || C.mem.empty()) continue;
                if (!st.customer_allowed_in_cluster(ui, cid)) continue;
                
                const auto& V = st.ctx->veh[C.vid];
                double room_needed = (C.load_w + u.w) - V.cap_w;
                if (room_needed <= 1e-9) {
                    // Already fits, try direct insert
                    double delta;
                    if (st.try_insert(ui, cid, iter, &delta)) {
                        touched.insert(cid);
                        break;
                    }
                    continue;
                }
                
                // Need to kick out some light members to make room
                // Find lightest members to steal
                std::vector<std::pair<double, int>> light_members;
                for (int m : C.mem) {
                    const auto& mc = st.ctx->cust[m];
                    // Only consider members lighter than the new customer
                    if (mc.w < u.w * 0.8) {
                        light_members.push_back({mc.w, m});
                    }
                }
                std::sort(light_members.begin(), light_members.end()); // lightest first
                
                double to_remove = room_needed;
                std::vector<int> victims;
                for (auto& [w, m] : light_members) {
                    if (to_remove <= 0) break;
                    victims.push_back(m);
                    to_remove -= w;
                }
                
                if (to_remove > 0) continue; // Can't make enough room
                
                // Try the steal: remove victims, insert new, relocate victims
                bool success = false;
                std::vector<int> relocated_to;
                
                // Backup
                double backup_obj = st.obj;
                
                for (int v : victims) {
                    st.remove_customer_from_cluster(v, cid);
                }
                
                double delta;
                if (st.try_insert(ui, cid, iter, &delta)) {
                    success = true;
                    touched.insert(cid);
                    
                    // Now try to place the victims elsewhere
                    for (int v : victims) {
                        bool placed = false;
                        for (int other = 0; other < (int)st.clusters.size(); ++other) {
                            if (other == cid) continue;
                            if (!st.clusters[other].active || st.clusters[other].mem.empty()) continue;
                            if (st.try_insert(v, other, iter, &delta)) {
                                touched.insert(other);
                                placed = true;
                                break;
                            }
                        }
                        // If can't place, victim stays unassigned (with penalty)
                    }
                }
                
                if (success) break;
                
                // Rollback if insert failed
                for (int v : victims) {
                    if (st.unassigned.count(v)) {
                        st.unassigned.erase(v);
                        st.cust_cluster[v] = cid;
                        C.mem.push_back(v);
                    }
                }
                st.recompute_cluster(cid);
            }
        }
    }

    // Maximize cluster sizes by stealing customers from neighboring clusters
    static void maximize_clusters_by_stealing(State& st, std::mt19937& rng, int iter,
                                              std::unordered_set<int>& touched) {
        // For each cluster, try to steal nearby customers from neighbors
        std::vector<int> clusters;
        for (int cid = 0; cid < (int)st.clusters.size(); ++cid) {
            if (st.clusters[cid].active && !st.clusters[cid].mem.empty()) {
                clusters.push_back(cid);
            }
        }
        std::shuffle(clusters.begin(), clusters.end(), rng);
        
        for (int cid : clusters) {
            auto& C = st.clusters[cid];
            if (!C.active || C.mem.empty()) continue;
            
            // Refresh neighbors
            st.refresh_neighbors(cid);
            if (C.nbr.empty()) continue;
            
            const auto& V = st.ctx->veh[C.vid];
            double remaining_cap = V.cap_w - C.load_w;
            if (remaining_cap < 1.0) continue; // No room
            
            // Try to steal customers from neighbors that are closer to our centroid
            double cx = C.sumx / std::max(1, C.n);
            double cy = C.sumy / std::max(1, C.n);
            
            for (int nbr_cid : C.nbr) {
                if (nbr_cid < 0 || nbr_cid >= (int)st.clusters.size()) continue;
                auto& NbrC = st.clusters[nbr_cid];
                if (!NbrC.active || NbrC.mem.empty()) continue;
                
                // Find customers in neighbor that are close to OUR centroid
                std::vector<std::pair<double, int>> steal_cands;
                for (int m : NbrC.mem) {
                    const auto& mc = st.ctx->cust[m];
                    double dist_to_us = std::hypot(mc.x - cx, mc.y - cy);
                    
                    // Boundary customers of neighbor (far from their center)
                    double nbr_cx = NbrC.sumx / std::max(1, NbrC.n);
                    double nbr_cy = NbrC.sumy / std::max(1, NbrC.n);
                    double dist_to_them = std::hypot(mc.x - nbr_cx, mc.y - nbr_cy);
                    
                    // Good steal: close to us, far from them
                    if (dist_to_us < dist_to_them && mc.w <= remaining_cap) {
                        steal_cands.push_back({dist_to_us, m});
                    }
                }
                
                std::sort(steal_cands.begin(), steal_cands.end()); // closest to us first
                
                int stolen = 0;
                for (auto& [d, m] : steal_cands) {
                    if (stolen >= 3) break; // Limit steals per neighbor
                    if (!st.customer_allowed_in_cluster(m, cid)) continue;
                    
                    // Check if we can take it
                    const auto& mc = st.ctx->cust[m];
                    if (C.load_w + mc.w > V.cap_w + 1e-9) continue;
                    
                    // Remove from neighbor, add to us
                    int old_cluster = st.cust_cluster[m];
                    if (old_cluster != nbr_cid) continue;
                    
                    st.remove_customer_from_cluster(m, nbr_cid);
                    double delta;
                    if (st.try_insert(m, cid, iter, &delta)) {
                        touched.insert(cid);
                        touched.insert(nbr_cid);
                        stolen++;
                        remaining_cap -= mc.w;
                    } else {
                        // Rollback - put back in neighbor
                        st.try_insert(m, nbr_cid, iter, &delta);
                    }
                }
            }
        }
    }

    static int grow_one_cluster(State& st, int cid, std::mt19937& rng, int iter, int batch) {
        auto& C = st.clusters[cid];
        if (C.mem.empty()) return 0;

        double cx = C.sumx / std::max(1, C.n);
        double cy = C.sumy / std::max(1, C.n);


        int inserted = 0;
        for (int rep=0; rep<batch && !st.unassigned.empty(); ++rep) {
            struct Move { double delta; int ui; };
            std::vector<Move> best_moves;
            // Reserve to avoid reallocations
            best_moves.reserve(std::min((size_t)st.unassigned.size(), (size_t)200));

            for (int ui : st.unassigned) {
                double delta = 0;
                if (st.check_insert_feasibility(ui, cid, iter, &delta)) {
                    best_moves.push_back({delta, ui});
                }
            }
            
            if (best_moves.empty()) break;
            
            // Stochastic selection from top K. K=3 (moderate randomness)
            std::sort(best_moves.begin(), best_moves.end(), [](auto& a, auto& b){ return a.delta < b.delta; });
            
            int K = std::min(3, (int)best_moves.size()); 
            std::uniform_int_distribution<int> uni(0, K-1);
            int idx = uni(rng);
            
            int chosen_ui = best_moves[idx].ui;
            
            double dummy;
            if (st.try_insert(chosen_ui, cid, iter, &dummy)) {
                inserted++;
                // update centroid for next iteration
                cx = st.clusters[cid].sumx / std::max(1, st.clusters[cid].n);
                cy = st.clusters[cid].sumy / std::max(1, st.clusters[cid].n);
            } else {
                break; 
            }
        }
        return inserted;
    }

    static bool try_ejection(State& st, int ui, std::mt19937& rng, int iter,
                             std::unordered_set<int>& touched) {
        // candidate clusters: nearest by centroid among same depot
        std::vector<std::pair<double,int>> cand;
        const auto& u = st.ctx->cust[ui];
        for (int cid=0; cid<(int)st.clusters.size(); ++cid) {
            const auto& C = st.clusters[cid];
            if (!C.active || C.mem.empty()) continue;
            if (st.ctx->p.enforce_territory && !u.territory.empty()) {
                if (st.ctx->veh[C.vid].depot_id != u.territory) continue;
            }
            double cx = C.sumx / std::max(1, C.n);
            double cy = C.sumy / std::max(1, C.n);
            double d = std::hypot(u.x - cx, u.y - cy);
            cand.push_back({d, cid});
        }
        std::sort(cand.begin(), cand.end());
        if ((int)cand.size() > 8) cand.resize(8);

        for (auto [d, cid] : cand) {
            if (!st.customer_allowed_in_cluster(ui, cid)) continue;
            auto& C = st.clusters[cid];
            if (C.boundary.empty()) st.refresh_boundary(cid);
            std::vector<int> eject = C.boundary;
            if (eject.empty()) eject = C.mem;
            // try few ejection candidates
            int tries = std::min(10, (int)eject.size());
            std::shuffle(eject.begin(), eject.end(), rng);
            
            for (int t=0;t<tries;++t) {
                int v = eject[t];
                
                // BACKUP
                double sO = st.obj, sF = st.fixed, sT = st.travel, sP = st.pen;
                double vkm = st.veh_km[C.vid], dw = st.depot_w[C.did], dv = st.depot_v[C.did];
                auto memC = C.mem; int ctrC = C.center; double trC = C.trip_km;

                st.remove_customer_from_cluster(v, cid);
                double delta = 0;
                if (st.try_insert(ui, cid, iter, &delta)) {
                    // accept if assigned
                    st.tabu_add(v, cid, iter);
                    touched.insert(cid);
                    return true;
                }
                
                // ROLLBACK
                st.unassigned.erase(v); st.cust_cluster[v] = cid;
                C.mem = memC; C.center = ctrC; C.trip_km = trC;
                st.obj = sO; st.fixed = sF; st.travel = sT; st.pen = sP;
                st.veh_km[C.vid] = vkm; st.depot_w[C.did] = dw; st.depot_v[C.did] = dv;
            }
        }
        return false;
    }

    static bool open_trip(State& st, std::mt19937& rng, int iter,
                          std::unordered_set<int>& touched) {
        if (st.unassigned.empty()) return false;
        
        // Try more potential seeds (increased from 10 to 20)
        struct S { double d; int ui; };
        std::vector<S> seeds;
        for (int ui : st.unassigned) {
            const auto& u = st.ctx->cust[ui];
            std::string dep_id = u.territory.empty() ? st.ctx->dep.front().id : u.territory;
            double d = st.ctx->inst.get_dist_km(dep_id, u.id);
            seeds.push_back({d, ui});
        }
        // Mix: some farthest, some heaviest
        std::sort(seeds.begin(), seeds.end(), [](auto& a, auto& b){ return a.d > b.d; });
        
        // Increased seed limit for more aggressive trip opening
        int seed_limit = std::min(50, (int)seeds.size());
        
        for (int i=0; i<seed_limit; ++i) {
            int seed = seeds[i].ui;
            const auto& u = st.ctx->cust[seed];

            std::vector<int> idle;
            std::vector<std::pair<double,int>> partially_used; // {remaining_km, vid}
            for (int vid=0; vid<(int)st.ctx->veh.size(); ++vid) {
                if (st.ctx->p.enforce_territory && !u.territory.empty()) {
                    if (st.ctx->veh[vid].depot_id != u.territory) continue;
                }
                if (st.ctx->p.enforce_class_req) {
                    if (type_rank(st.ctx->veh[vid].type) < type_rank(u.class_req)) continue;
                }
                double max_d = st.ctx->veh[vid].max_dist;
                double used = st.veh_km[vid];
                double remaining = max_d - used;
                
                if (used < 1e-7) {
                    idle.push_back(vid);
                } else if (remaining > 5.0) { // At least 5km remaining for a trip
                    partially_used.push_back({remaining, vid});
                }
            }
            
            // Sort partially used by MOST remaining capacity first
            std::sort(partially_used.begin(), partially_used.end(), std::greater<>());
            
            std::shuffle(idle.begin(), idle.end(), rng);
            
            // Priority: idle first, then vehicles with most remaining capacity
            std::vector<int> v_order = idle;
            for (auto& [rem, vid] : partially_used) {
                v_order.push_back(vid);
            }

            for (int vid : v_order) {
                int did = st.ctx->didx.at(st.ctx->veh[vid].depot_id);
                Cluster C; C.vid = vid; C.did = did; C.active = true;
                st.clusters.push_back(std::move(C));
                int cid = (int)st.clusters.size()-1;

                double delta;
                if (st.try_insert(seed, cid, iter, &delta)) {
                    touched.insert(cid);
                    // grow aggressively until cap
                    while (!st.unassigned.empty()) {
                        int added = grow_one_cluster(st, cid, rng, iter, st.ctx->p.batch_M);
                        if (added == 0) break;
                    }
                    return true;
                }
                st.clusters.pop_back(); // rollback
            }
        }
        return false;
    }

    // ---------- VND (around touched) ----------
    static void vnd(State& st, std::mt19937& rng, int iter, std::unordered_set<int>& touched) {
        if (touched.empty()) return;
        
        // Scope: touched + their current neighbors
        std::unordered_set<int> scope = touched;
        for (int cid : touched) {
            if (cid < 0 || cid >= (int)st.clusters.size()) continue;
            // Only use top 4 neighbors for VND speed
            int take = std::min(4, (int)st.clusters[cid].nbr.size());
            for (int i=0; i<take; ++i) scope.insert(st.clusters[cid].nbr[i]);
        }
        std::vector<int> target_list(scope.begin(), scope.end());

        for (int k=0; k<30; ++k) {
            bool improved = false;
            std::shuffle(target_list.begin(), target_list.end(), rng);
            
            for (int A : target_list) {
                if (A < 0 || A >= (int)st.clusters.size()) continue;
                if (!st.clusters[A].active || st.clusters[A].mem.empty()) continue;
                
                for (int B : st.clusters[A].nbr) {
                    if (B < 0 || B >= (int)st.clusters.size()) continue;
                    if (!st.clusters[B].active || st.clusters[B].mem.empty()) continue;
                    
                    if (try_relocate(st, A, B, iter)) { improved = true; break; }
                    if (try_swap(st, A, B, iter)) { improved = true; break; }
                }
                if (improved) break;
            }
            if (!improved) break;
        }
        for (int cid : scope) { st.refresh_neighbors(cid); st.refresh_boundary(cid); st.clusters[cid].dirty = false; }
    }

    static bool try_relocate(State& st, int A, int B, int iter) {
        if (A < 0 || B < 0 || A >= (int)st.clusters.size() || B >= (int)st.clusters.size()) return false;
        if (st.clusters[A].mem.empty()) return false;
        auto& CA = st.clusters[A];
        if (CA.boundary.empty()) st.refresh_boundary(A);
        std::vector<int> cand = CA.boundary.empty() ? CA.mem : CA.boundary;
        int tries = std::min(10, (int)cand.size());
        for (int t = 0; t < tries; ++t) {
            int ui = cand[t];
            double dRem = 0, dIns = 0;
            if (st.check_remove_delta(ui, A, &dRem)) {
                if (st.check_insert_feasibility(ui, B, iter, &dIns)) {
                    if (dRem + dIns < -1e-9) {
                        st.remove_customer_from_cluster(ui, A);
                        st.try_insert(ui, B, iter);
                        return true;
                    }
                }
            }
        }
        return false;
    }

    static bool try_swap(State& st, int A, int B, int iter) {
        if (A < 0 || B < 0 || A >= (int)st.clusters.size() || B >= (int)st.clusters.size()) return false;
        if (st.clusters[A].mem.empty() || st.clusters[B].mem.empty()) return false;
        auto& CA = st.clusters[A];
        auto& CB = st.clusters[B];
        if (CA.boundary.empty()) st.refresh_boundary(A);
        if (CB.boundary.empty()) st.refresh_boundary(B);
        std::vector<int> a = CA.boundary.empty() ? CA.mem : CA.boundary;
        std::vector<int> b = CB.boundary.empty() ? CB.mem : CB.boundary;
        int ta = std::min(4, (int)a.size());
        int tb = std::min(4, (int)b.size());
        for (int i = 0; i < ta; ++i) {
            for (int j = 0; j < tb; ++j) {
                int ua = a[i], ub = b[j];

                // Pre-filter: if removal deltas are huge, skip
                double dA_rem=0, dB_rem=0;
                if (!st.check_remove_delta(ua, A, &dA_rem)) continue;
                if (!st.check_remove_delta(ub, B, &dB_rem)) continue;
                
                // If it looks very bad (e.g. > 1e4 increase), skip
                if (dA_rem + dB_rem > 1e4) continue;

                double start_o = st.obj;
                auto memA = CA.mem; int ctrA = CA.center; double trA = CA.trip_km;
                auto memB = CB.mem; int ctrB = CB.center; double trB = CB.trip_km;
                double vkmA = st.veh_km[CA.vid], vkmB = st.veh_km[CB.vid];
                double dwA = st.depot_w[CA.did], dwB = st.depot_w[CB.did];
                double dvA = st.depot_v[CA.did], dvB = st.depot_v[CB.did];

                st.remove_customer_from_cluster(ua, A);
                st.remove_customer_from_cluster(ub, B);
                if (st.check_insert_feasibility(ua, B, iter) && st.check_insert_feasibility(ub, A, iter)) {
                    st.try_insert(ua, B, iter);
                    st.try_insert(ub, A, iter);
                    if (st.obj < start_o - 1e-9) return true;
                    st.remove_customer_from_cluster(ub, A);
                    st.remove_customer_from_cluster(ua, B);
                } else {
                    st.unassigned.erase(ua); st.unassigned.erase(ub);
                }
                CA.mem = memA; CA.center = ctrA; CA.trip_km = trA;
                CB.mem = memB; CB.center = ctrB; CB.trip_km = trB;
                for (int x : CA.mem) st.cust_cluster[x] = A;
                for (int x : CB.mem) st.cust_cluster[x] = B;
                st.obj = start_o;
                st.veh_km[CA.vid] = vkmA; st.veh_km[CB.vid] = vkmB;
                st.depot_w[CA.did] = dwA; st.depot_w[CB.did] = dwB;
                st.depot_v[CA.did] = dvA; st.depot_v[CB.did] = dvB;
            }
        }
        return false;
    }
};

} // namespace alns_vnd
