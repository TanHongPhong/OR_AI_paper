// =============================================================
// optimizer_main.cpp - Grid-Grow ALNS Optimizer
//
// Usage:
//   ./optimizer [grid_dir] [source_dir] [out_dir] [seed] [iters]
//
// Defaults:
//   grid_dir   = grid_out
//   source_dir = grid_dir
//   out_dir    = grid_dir
//   seed       = current time
//   iters      = 1000
// =============================================================

#include "data_loader.h"
#include "initialization.h"
#include "objectives.h"
#include "grid_grow_alns.h"
#include "results_export.h"

#include <iostream>
#include <ctime>
#include <chrono>

int main(int argc, char** argv) {
    try {
        // ===== Parse arguments =====
        const std::string grid_dir   = (argc >= 2) ? argv[1] : "grid_out";
        const std::string source_dir = (argc >= 3) ? argv[2] : grid_dir;
        const std::string out_dir    = (argc >= 4) ? argv[3] : grid_dir;
        const int seed               = (argc >= 5) ? std::stoi(argv[4]) : (int)std::time(nullptr);
        const int iters              = (argc >= 6) ? std::stoi(argv[5]) : 1000;

        std::cout << "========================================\n";
        std::cout << "    Grid-Grow ALNS Optimizer\n";
        std::cout << "========================================\n";
        std::cout << "Grid dir:    " << grid_dir << "\n";
        std::cout << "Source dir:  " << source_dir << "\n";
        std::cout << "Out dir:     " << out_dir << "\n";
        std::cout << "Seed:        " << seed << "\n";
        std::cout << "Iterations:  " << iters << "\n";
        std::cout << "========================================\n\n";

        // ===== Load instance =====
        std::cout << "[1/4] Loading instance...\n";
        auto t0 = std::chrono::steady_clock::now();
        gridopt::Instance inst = DataLoader::load_instance(grid_dir, source_dir);
        auto t1 = std::chrono::steady_clock::now();
        double load_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::cout << "      Done in " << std::fixed << std::setprecision(0) << load_ms << " ms\n\n";

        // ===== Create initial solution =====
        std::cout << "[2/4] Creating initial solution...\n";
        t0 = std::chrono::steady_clock::now();
        gridopt::Solution init_sol = gridopt::Initializer::create_initial_solution(inst, seed);
        gridopt::Objectives::evaluate(inst, init_sol);
        t1 = std::chrono::steady_clock::now();
        double init_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::cout << "      Done in " << std::fixed << std::setprecision(0) << init_ms << " ms\n";
        gridopt::print_solution_summary(init_sol, "      Init");
        std::cout << "\n";

        // ===== Run ALNS optimization =====
        std::cout << "[3/4] Running ALNS optimization (" << iters << " iterations)...\n";
        t0 = std::chrono::steady_clock::now();
        
        gridopt::GridGrowALNS optimizer(inst, seed);
        gridopt::ALNSParams params;
        params.iterations = iters;
        
        gridopt::Solution best_sol = optimizer.run(init_sol, params);
        
        t1 = std::chrono::steady_clock::now();
        double opt_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::cout << "      Done in " << std::fixed << std::setprecision(0) << opt_ms << " ms\n";
        gridopt::print_solution_summary(best_sol, "      Best");
        std::cout << "\n";

        // ===== Print penalty summary =====
        gridopt::Objectives::print_penalty_summary(inst, best_sol);

        // ===== Sync solution to id_map for export =====
        gridopt::GridGrowALNS::sync_solution_to_id_map(inst, best_sol);

        // ===== Export results =====
        std::cout << "[4/4] Exporting results...\n";
        gridopt::ExportConfig cfg;
        cfg.out_dir = out_dir;
        cfg.assignment_filename = "alns_assignment.csv";
        cfg.routes_filename = "alns_routes.csv";
        cfg.summary_filename = "alns_summary.csv";
        cfg.add_timestamp = false;
        
        gridopt::export_all(inst, best_sol, cfg);
        
        // ===== Final summary =====
        std::cout << "\n========================================\n";
        std::cout << "              RESULTS\n";
        std::cout << "========================================\n";
        
        size_t init_served = 0, best_served = 0;
        for (const auto& kv : init_sol.routes) {
            for (const auto& rt : kv.second) init_served += rt.member_cells_linear.size();
        }
        for (const auto& kv : best_sol.routes) {
            for (const auto& rt : kv.second) best_served += rt.member_cells_linear.size();
        }
        
        size_t total_customers = best_served + best_sol.unassigned_cells.size();
        
        std::cout << "Initial:    " << init_served << "/" << total_customers 
                  << " served (" << std::fixed << std::setprecision(1) 
                  << (100.0 * init_served / total_customers) << "%)\n";
        std::cout << "Optimized:  " << best_served << "/" << total_customers 
                  << " served (" << std::fixed << std::setprecision(1) 
                  << (100.0 * best_served / total_customers) << "%)\n";
        std::cout << "Improvement: " << (int)best_served - (int)init_served << " more served\n";
        std::cout << "Objective:   " << std::fixed << std::setprecision(2) << init_sol.objective 
                  << " -> " << best_sol.objective << "\n";
        std::cout << "========================================\n";
        std::cout << "\n[OK] Optimization complete.\n";
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "\n[ERROR] " << e.what() << "\n";
        return 1;
    }
}
