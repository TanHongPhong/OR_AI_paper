#include "config.h"
#include "data_loader.h"
#include "initialization.h"
#include "objectives.h"
#include "results_export.h"
#include "grid_grow_alns.h"  // New engine

#include <iostream>
#include <filesystem>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <map>

// Adapter to run Grid-Grow ALNS
Solution run_grid_grow_optimizer(const Instance& inst, const Solution& init_sol, int iters) {
    // using namespace gg_alns; // CONFLICT! Do not use.
    std::cout << "[GridGrow] Preparing data structures...\n";

    // 1. Convert Data
    std::vector<gg_alns::Customer> customers_vec;
    customers_vec.reserve(inst.customers.size());
    std::unordered_map<std::string, int> cid_map; // ext_id -> int index

    // Important: We need a mapping from int index back to string id for final output
    std::vector<std::string> idx_to_cid;
    idx_to_cid.reserve(inst.customers.size());

    int c_idx = 0;
    for (const auto& kv : inst.customers) {
        gg_alns::Customer c;
        c.id = c_idx;
        c.p = {kv.second.x_km, kv.second.y_km};
        c.demand = kv.second.weight;
        c.vol = kv.second.volume;
        c.class_req = type_rank(kv.second.class_req); // use rank!
        customers_vec.push_back(c);
        
        cid_map[kv.first] = c_idx;
        idx_to_cid.push_back(kv.first);
        c_idx++;
    }

    std::vector<gg_alns::Depot> depots_vec;
    std::unordered_map<std::string, int> did_map;
    std::vector<std::string> idx_to_did;
    int d_idx = 0;
    for (const auto& kv : inst.depots) {
        gg_alns::Depot d;
        d.id = d_idx;
        d.p = {kv.second.x_km, kv.second.y_km};
        d.cap = kv.second.cap_weight_storage; // assuming weight cap primary
        depots_vec.push_back(d);

        did_map[kv.first] = d_idx;
        idx_to_did.push_back(kv.first);
        d_idx++;
    }

    std::vector<gg_alns::Vehicle> vehicles_vec;
    std::unordered_map<std::string, int> vid_map;
    std::vector<std::string> idx_to_vid;
    int v_idx = 0;
    for (const auto& kv : inst.vehicles) {
        gg_alns::Vehicle v;
        v.id = v_idx;
        v.depot_id = did_map.at(kv.second.start_depot);
        v.cap = kv.second.cap_weight;
        v.cap_vol = kv.second.cap_volume;
        v.max_km_total = kv.second.max_dist;
        v.class_cap = type_rank(kv.second.type); // use rank!
        vehicles_vec.push_back(v);

        vid_map[kv.first] = v_idx;
        idx_to_vid.push_back(kv.first);
        v_idx++;
    }

    // 2. Initialize State
    gg_alns::State st;
    // Cell size 200m (0.2km) for broader neighbor search
    st.init_refs(customers_vec, depots_vec, vehicles_vec, 0.05); 

    // 3. Build clusters from initial solution + Backup Clusters
    st.clusters.clear();
    st.cust_cluster.assign(customers_vec.size(), -1); // Reset assignment

    st.hooks.vehicle_customer_ok = [&](const gg_alns::Vehicle& veh, const gg_alns::Customer& cus) {
        return veh.class_cap >= cus.class_req;
    };

    // Soft Hooks returning Penalty
    st.hooks.vehicle_km_penalty = [&](int vehicle_id, double new_vehicle_km) {
        double limit = vehicles_vec[vehicle_id].max_km_total;
        if (new_vehicle_km > limit) {
             return (new_vehicle_km - limit) * 10000.0; // 1e4 per km overflow
        }
        return 0.0;
    };

    st.hooks.depot_load_penalty = [&](int depot_id, double new_load) {
        double limit = depots_vec[depot_id].cap;
        if (!std::isinf(limit) && new_load > limit) {
            return (new_load - limit) * 1000.0; // 1e3 per kg overflow
        }
        return 0.0;
    };
    
    st.hooks.center_ok = [&](int depot_id, int center_ui, double trip_km) {
        if (trip_km >= 1e8) return false; // Sanity check
        return true; 
    };

    // Pass 1: Create clusters from init sol + BACKUP CLUSTERS
    int cid_alloc = 0;
    
    // Existing clusters
    for (const auto& kv : init_sol.routes) {
        std::string veh_id = kv.first;
        if (vid_map.find(veh_id) == vid_map.end()) continue;
        int vid = vid_map[veh_id];
        for (const auto& r : kv.second) {
            gg_alns::Cluster cl;
            cl.id = cid_alloc++;
            cl.vehicle_id = vid;
            cl.depot_id = vehicles_vec[vid].depot_id;
            st.clusters.push_back(cl);
        }
    }
    
    // Backup Clusters (1 empty per vehicle to allow expansion)
    std::cout << "[GridGrow] Creating backup clusters for dynamic trips...\n";
    for (int vid=0; vid < (int)vehicles_vec.size(); ++vid) {
         gg_alns::Cluster cl;
         cl.id = cid_alloc++;
         cl.vehicle_id = vid;
         cl.depot_id = vehicles_vec[vid].depot_id;
         st.clusters.push_back(cl);
    }
    std::cout << "[GridGrow] Total clusters (active + backup): " << st.clusters.size() << "\n";

    // Pass 2: Insert customers into existing clusters
    int obs_cid = 0;
    int fail_finalize = 0;
    for (const auto& kv : init_sol.routes) {
        std::string veh_id = kv.first;
        if (vid_map.find(veh_id) == vid_map.end()) continue;
        for (const auto& r : kv.second) {
            int target_cid = obs_cid; 
            for (const auto& cid_str : r.cluster_customers) {
                if (cid_map.find(cid_str) != cid_map.end()) {
                    int ui = cid_map[cid_str];
                    st.add_to_cluster(ui, target_cid);
                    st.depot_total_load[st.clusters[target_cid].depot_id] += customers_vec[ui].demand;
                }
            }
            if (!st.finalize_cluster(target_cid)) {
                 fail_finalize++;
            }
            obs_cid++;
        }
    }
    std::cout << "[Adap] State populated. Finalize fails: " << fail_finalize << "\n";
    
    // Sync unassigned list and calculate initial costs
    st.unassigned.clear();
    st.unassigned_cnt = 0;
    st.total_cost = 0; 
    
    for(int i=0; i<(int)customers_vec.size(); ++i) {
        if(st.cust_cluster[i] == -1) {
            st.in_unassigned[i] = 1;
            st.unassigned.push_back(i);
            st.unassigned_cnt++;
            st.total_cost += st.penalty_unserved;
        } else {
            st.in_unassigned[i] = 0;
        }
    }
    
    // Add active penalties to cost
    for (const auto& C : st.clusters) {
         st.total_cost += C.trip_km;
         
         const gg_alns::Vehicle& V = vehicles_vec[C.vehicle_id];
         if (C.load > V.cap) st.total_cost += (C.load - V.cap) * st.penalty_cap;
         if (C.load_vol > V.cap_vol) st.total_cost += (C.load_vol - V.cap_vol) * st.penalty_cap;
    }
    for (int did=0; did<(int)depots_vec.size(); ++did) {
        st.total_cost += st.hooks.depot_load_penalty(did, st.depot_total_load[did]);
    }
    for (int vid=0; vid<(int)vehicles_vec.size(); ++vid) {
        st.total_cost += st.hooks.vehicle_km_penalty(vid, st.vehicle_total_km[vid]);
    }
    
    std::cout << "[Adap] Initial State Cost (inc penalties): " << st.total_cost << "\n";

    // 5. Run ALNS
    std::cout << "[GridGrow] Starting optimization for " << iters << " iterations...\n";
    gg_alns::GridGrowALNS alns;
    alns.cfg.cell_size_km = 0.05;
    alns.cfg.grow_budget = 64;
    alns.cfg.vnd_iters = 100; 
    alns.cfg.peel_frac = 0.20;
    alns.cfg.region_cells = 8;
    alns.cfg.destroy_whole_prob = 0.05;
    // --- NEW: territory-free centers (cohesion by center adjacency) ---
    alns.cfg.center_adj_radius_km = 0.6;   // tune (km)
    alns.cfg.w_center_adj = 25.0;          // tune
    alns.cfg.w_center_cc  = 200.0;         // tune
    // --- NEW: empty-cluster seeding around depot center patch ---
    alns.cfg.seed_ring_max = 25;
    alns.cfg.seed_need_cands = 80;
    alns.cfg.seed_global_sample = 400;
    alns.cfg.w_patch = 0.35;
    
    double best_cost = alns.run(st, iters);

    std::cout << "[GridGrow] Optimization finished. Best ALNS Cost: " << best_cost << "\n";

    // 6. Convert back to Solution
    Solution sol_out;
    sol_out.routes = init_sol.routes; // Keep structure, clear content
    for (auto& kv : sol_out.routes) kv.second.clear();
    sol_out.unassigned_customers.clear();

    // Fill routes
    for (const auto& cl : st.clusters) {
        if (cl.members.empty()) continue;
        
        Route r;
        r.vehicle_id = idx_to_vid[cl.vehicle_id];
        r.cluster_id = std::to_string(cl.id); // arbitrary unique ID
        r.centroid_id = (cl.center_ui >= 0) ? idx_to_cid[cl.center_ui] : "";
        
        std::string did_str = idx_to_did[cl.depot_id];
        r.stops = {did_str, r.centroid_id, did_str};
        
        r.distance = cl.trip_km;
        r.load_w = cl.load;
        r.load_v = cl.load_vol;
        
        for (int ui : cl.members) {
            r.cluster_customers.push_back(idx_to_cid[ui]);
        }
        
        if (sol_out.routes.find(r.vehicle_id) == sol_out.routes.end()) {
             sol_out.routes[r.vehicle_id] = std::vector<Route>();
        }
        sol_out.routes[r.vehicle_id].push_back(r);
    }
    
    // Fill unassigned
    for(int i=0; i<(int)customers_vec.size(); ++i) {
        if (st.cust_cluster[i] == -1) {
            sol_out.unassigned_customers.insert(idx_to_cid[i]);
        }
    }

    // Recalculate full objective (costs + penalties)
    sol_out.calculate_objective(inst); 
    
    return sol_out;
}

int main(int argc, char** argv) {
    std::string data_dir = (argc >= 2) ? argv[1] : std::string("data");
    std::string out_base = (argc >= 3) ? argv[2] : std::string("outputs");
    int iters = (argc >= 4) ? std::max(0, std::atoi(argv[3])) : 0;

    std::cout << "========================================\n";
    std::cout << "  CLUSTER-TRIP OPTIMIZER (GRID-GROW ENGINE)\n";
    std::cout << "========================================\n\n";

    std::cout << "[MAIN] Loading instance from: " << data_dir << std::endl;
    Instance inst = DataLoader::load_instance(data_dir);
    std::cout << "[MAIN] Loaded: " << inst.customers.size() << " customers, "
              << inst.depots.size() << " depots, "
              << inst.vehicles.size() << " vehicles" << std::endl << std::endl;

    // timestamp output dir
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);
    std::ostringstream oss;
    oss << out_base << "/run_" << std::put_time(&tm, "%Y%m%d_%H%M%S");
    std::string out_dir = oss.str();
    std::filesystem::create_directories(out_dir);
    std::cout << "[MAIN] Output directory: " << out_dir << std::endl;

    std::cout << "[MAIN] Creating initial solution..." << std::endl;
    Solution init = Initializer::create_initial_solution(inst);
    Objectives::evaluate(inst, init);
    std::cout << "[MAIN] Initial objective = " << init.objective << std::endl;
    std::cout << "       fixed_cost=" << init.fixed_cost
              << " travel_cost=" << init.travel_cost
              << " unserved=" << init.unassigned_customers.size() << std::endl;

    export_solution_summaries(inst, init, out_dir, "initial");

    if (iters > 0) {
        try {
            Solution best = run_grid_grow_optimizer(inst, init, iters);

            std::cout << "[MAIN] Final Best objective = " << best.objective << std::endl;
            std::cout << "       fixed_cost=" << best.fixed_cost
                      << " travel_cost=" << best.travel_cost
                      << " unserved=" << best.unassigned_customers.size() << std::endl;

            export_solution_summaries(inst, best, out_dir, "best");
            std::cout << "[MAIN] Exported best_* files." << std::endl;
            
        } catch (const std::exception& e) {
            std::cerr << "[CRITICAL] Exception caught in solve: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "[CRITICAL] Unknown exception caught in solve." << std::endl;
        }
    } else {
        std::cout << "[MAIN] Skip optimizer (iters=0).\n";
    }

    return 0;
}
