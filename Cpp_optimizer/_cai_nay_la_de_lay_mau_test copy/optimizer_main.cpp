// =============================================================
// optimizer_main.cpp - Grid-Grow ALNS Optimizer with Periodic Export
//
// Usage:
//   ./optimizer [grid_dir] [source_dir] [out_dir] [seed] [iters] [export_interval]
//
// Defaults:
//   grid_dir        = grid_out
//   source_dir      = grid_dir
//   out_dir         = grid_dir
//   seed            = current time
//   iters           = 10000
//   export_interval = 500
// =============================================================

#include "data_loader.h"
#include "initialization.h"
#include "objectives.h"
#include "grid_grow_alns.h"
#include "results_export.h"

#include <iostream>
#include <ctime>
#include <chrono>
#include <sstream>

// Helper to export a solution snapshot
void export_snapshot(gridopt::Instance& inst, const gridopt::Solution& sol, 
                     const std::string& base_dir, int iter_num) {
    // Create subfolder for this iteration: iter_00000, iter_00500, etc.
    std::ostringstream oss;
    oss << "iter_" << std::setfill('0') << std::setw(5) << iter_num;
    std::string iter_dir = gridopt::join_path(base_dir, oss.str());
    gridopt::ensure_dir(iter_dir);
    
    // Sync and export
    gridopt::GridGrowALNS::sync_solution_to_id_map(inst, sol);
    
    gridopt::ExportConfig snap_cfg;
    snap_cfg.out_dir = iter_dir;
    snap_cfg.assignment_filename = "alns_assignment.csv";
    snap_cfg.routes_filename = "alns_routes.csv";
    snap_cfg.summary_filename = "alns_summary.csv";
    snap_cfg.add_timestamp = false;
    
    gridopt::export_all(inst, sol, snap_cfg);
    std::cout << "      [SNAPSHOT] Exported iter " << iter_num << " to " << iter_dir << "\n";
}

int main(int argc, char** argv) {
    try {
        // ===== Parse arguments =====
        const std::string grid_dir   = (argc >= 2) ? argv[1] : "grid_out";
        const std::string source_dir = (argc >= 3) ? argv[2] : grid_dir;
        const std::string out_dir    = (argc >= 4) ? argv[3] : grid_dir;
        const int seed               = (argc >= 5) ? std::stoi(argv[4]) : (int)std::time(nullptr);
        const int total_iters        = (argc >= 6) ? std::stoi(argv[5]) : 5000;
        const int export_interval    = (argc >= 7) ? std::stoi(argv[6]) : 500;

        std::cout << "========================================\n";
        std::cout << "    Grid-Grow ALNS Optimizer (Tracking)\n";
        std::cout << "========================================\n";
        std::cout << "Grid dir:        " << grid_dir << "\n";
        std::cout << "Source dir:      " << source_dir << "\n";
        std::cout << "Out dir:         " << out_dir << "\n";
        std::cout << "Seed:            " << seed << "\n";
        std::cout << "Total Iters:     " << total_iters << "\n";
        std::cout << "Export Interval: " << export_interval << "\n";
        std::cout << "========================================\n\n";

        // ===== Load instance =====
        std::cout << "[1/4] Loading instance...\n";
        auto t0 = std::chrono::steady_clock::now();
        gridopt::Instance inst = DataLoader::load_instance(grid_dir, source_dir);
        auto t1 = std::chrono::steady_clock::now();
        double load_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::cout << "      Done in " << std::fixed << std::setprecision(0) << load_ms << " ms\n\n";

        // ===== Create unique run folder =====
        std::string run_label = "run_" + gridopt::timestamp_suffix();
        std::string final_out_dir = gridopt::join_path(out_dir, run_label);
        gridopt::ensure_dir(final_out_dir);
        std::cout << "Output folder: " << final_out_dir << "\n\n";

        // ===== Create initial solution =====
        std::cout << "[2/4] Creating initial solution...\n";
        t0 = std::chrono::steady_clock::now();
        gridopt::Solution current_sol = gridopt::Initializer::create_initial_solution(inst, seed);
        gridopt::Objectives::evaluate(inst, current_sol);
        t1 = std::chrono::steady_clock::now();
        double init_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::cout << "      Done in " << std::fixed << std::setprecision(0) << init_ms << " ms\n";
        gridopt::print_solution_summary(current_sol, "      Init");
        std::cout << "\n";

        // Export initial solution (iter 0)
        export_snapshot(inst, current_sol, final_out_dir, 0);

        // ===== Run ALNS optimization in segments =====
        std::cout << "[3/4] Running ALNS optimization (" << total_iters << " iterations)...\n";
        t0 = std::chrono::steady_clock::now();
        
        gridopt::GridGrowALNS optimizer(inst, seed);
        
        int completed_iters = 0;
        while (completed_iters < total_iters) {
            // Calculate segment size
            int segment_size = std::min(export_interval, total_iters - completed_iters);
            
            gridopt::ALNSParams params;
            params.iterations = segment_size;
            params.verbose = (completed_iters == 0); // Only verbose on first segment
            
            // Run segment
            current_sol = optimizer.run(current_sol, params);
            completed_iters += segment_size;
            
            // Export snapshot
            export_snapshot(inst, current_sol, final_out_dir, completed_iters);
            
            // Progress update
            size_t served = 0;
            for (const auto& kv : current_sol.routes) {
                for (const auto& rt : kv.second) served += rt.member_cells_linear.size();
            }
            size_t total = served + current_sol.unassigned_cells.size();
            std::cout << "      [PROGRESS] iter=" << completed_iters << "/" << total_iters
                      << " served=" << served << "/" << total
                      << " (" << std::fixed << std::setprecision(1) << (100.0 * served / total) << "%)"
                      << " obj=" << std::setprecision(2) << current_sol.objective << "\n";
        }
        
        t1 = std::chrono::steady_clock::now();
        double opt_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::cout << "      Done in " << std::fixed << std::setprecision(0) << opt_ms << " ms\n";
        gridopt::print_solution_summary(current_sol, "      Final");
        std::cout << "\n";

        // ===== Print penalty summary =====
        gridopt::Objectives::print_penalty_summary(inst, current_sol);

        // ===== Sync solution to id_map for export =====
        gridopt::GridGrowALNS::sync_solution_to_id_map(inst, current_sol);

        // ===== Export final results =====
        std::cout << "[4/4] Exporting final results...\n";

        gridopt::ExportConfig cfg;
        cfg.out_dir = final_out_dir;
        cfg.assignment_filename = "alns_assignment.csv";
        cfg.routes_filename = "alns_routes.csv";
        cfg.summary_filename = "alns_summary.csv";
        cfg.add_timestamp = false;
        
        gridopt::export_all(inst, current_sol, cfg);
        
        // ===== Final summary =====
        std::cout << "\n========================================\n";
        std::cout << "              RESULTS\n";
        std::cout << "========================================\n";
        
        size_t final_served = 0;
        for (const auto& kv : current_sol.routes) {
            for (const auto& rt : kv.second) final_served += rt.member_cells_linear.size();
        }
        
        size_t total_customers = final_served + current_sol.unassigned_cells.size();
        
        std::cout << "Served:     " << final_served << "/" << total_customers 
                  << " (" << std::fixed << std::setprecision(1) 
                  << (100.0 * final_served / total_customers) << "%)\n";
        std::cout << "Objective:  " << std::fixed << std::setprecision(2) << current_sol.objective << "\n";
        std::cout << "Snapshots:  " << (total_iters / export_interval + 1) << " exported\n";
        std::cout << "Output dir: " << final_out_dir << "\n";
        
        // ===== Vehicle Utilization Report =====
        std::cout << "\n--------------------------------------------------------------\n";
        std::cout << "VEHICLE UTILIZATION REPORT\n";
        std::cout << "--------------------------------------------------------------\n";
        std::cout << "VehicleID | Type            | TripKM    | MaxKM     | Usage%  | LoadW%  | LoadV%\n";
        std::cout << "----------|-----------------|-----------|-----------|---------|---------|-------\n";

        for (const auto& kv : current_sol.routes) {
            const std::string& vid = kv.first;
            auto itV = inst.vehicles.find(vid);
            if (itV == inst.vehicles.end()) continue;
            
            double used_km = 0.0;
            double used_w = 0.0;
            double used_v = 0.0;
            for (const auto& r : kv.second) {
                used_km += r.distance_km;
                used_w += r.load_w;
                used_v += r.load_v;
            }
            
            double max_km = itV->second.max_dist > 1e-9 ? itV->second.max_dist : 0.0;
            double pct_km = (max_km > 0) ? (used_km / max_km * 100.0) : 0.0;
            
            double max_w = itV->second.cap_weight > 1e-9 ? itV->second.cap_weight : 0.0;
            double pct_w = (max_w > 0) ? (used_w / max_w * 100.0) : 0.0;
            
            double max_v = itV->second.cap_volume > 1e-9 ? itV->second.cap_volume : 0.0;
            double pct_v = (max_v > 0) ? (used_v / max_v * 100.0) : 0.0;
            
            std::cout << std::left << std::setw(10) << vid 
                      << "| " << std::setw(16) << itV->second.vehicle_type
                      << "| " << std::setw(10) << std::fixed << std::setprecision(2) << used_km
                      << "| " << std::setw(10) << max_km
                      << "| " << std::setw(8) << std::fixed << std::setprecision(1) << pct_km << "%"
                      << "| " << std::setw(8) << std::fixed << std::setprecision(1) << pct_w << "%"
                      << "| " << std::fixed << std::setprecision(1) << pct_v << "%\n";
        }

        std::cout << "========================================\n";
        std::cout << "\n[OK] Optimization complete.\n";
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "\n[ERROR] " << e.what() << "\n";
        return 1;
    }
}
