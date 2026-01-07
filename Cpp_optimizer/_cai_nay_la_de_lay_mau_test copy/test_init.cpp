// test_init.cpp
#include "data_loader.h"
#include "initialization.h"
#include "results_export.h" // Added for export
#include "config.h"
#include <iostream>
#include <iomanip>

// Helper to sync solution IDs to id_map (simplified version of GridGrowALNS::sync_solution_to_id_map)
void sync_solution_to_id_map(gridopt::Instance& inst, const gridopt::Solution& sol) {
    // Reset first
    for (auto &row : inst.id_map.rows) {
        row.cluster = -1;
        row.vehicle_id.clear();
        row.depot_id.clear();
    }

    int cluster_id_counter = 1;
    for (const auto& kv : sol.routes) {
        const std::string& vid = kv.first;
        std::string did;
        auto itV = inst.vehicles.find(vid);
        if (itV != inst.vehicles.end()) did = itV->second.start_depot;

        for (const auto& r : kv.second) {
            int cid = cluster_id_counter++;
            for (int cell : r.member_cells_linear) {
                auto it = inst.id_map.cell_to_customer_indices.find(cell);
                if (it == inst.id_map.cell_to_customer_indices.end()) continue;
                for (int cust_idx : it->second) {
                    if (cust_idx < 0 || cust_idx >= (int)inst.id_map.rows.size()) continue;
                    auto &row = inst.id_map.rows[(size_t)cust_idx];
                    row.cluster = cid;
                    row.vehicle_id = vid;
                    row.depot_id = did;
                }
            }
        }
    }
}

int main(int argc, char** argv) {
    try {
        const std::string grid_dir   = (argc >= 2) ? argv[1] : ".";
        const std::string source_dir = (argc >= 3) ? argv[2] : grid_dir;
        const std::string out_dir    = (argc >= 4) ? argv[3] : "init_out"; // New arg for output
        int seed = 1234567;

        std::cout << "Loading instance from: " << grid_dir << "\n";
        gridopt::Instance inst = DataLoader::load_instance(grid_dir, source_dir);
        std::cout << "Instance loaded. Depots: " << inst.depots.size() 
                  << ", Vehicles: " << inst.vehicles.size() << "\n";

        std::cout << "Running Initializer...\n";
        gridopt::Solution sol = gridopt::Initializer::create_initial_solution(inst, seed);

        // Print Summary
        size_t total_served = 0;
        
        std::cout << "\n--------------------------------------------------------------\n";
        std::cout << "VEHICLE UTILIZATION REPORT\n";
        std::cout << "--------------------------------------------------------------\n";
        std::cout << "VehicleID | Type            | TripKM    | MaxKM     | Usage%\n";
        std::cout << "----------|-----------------|-----------|-----------|-------\n";

        for (const auto& kv : sol.routes) {
            const std::string& vid = kv.first;
            auto itV = inst.vehicles.find(vid);
            if (itV == inst.vehicles.end()) continue;
            
            double used_km = 0.0;
            size_t n_cust = 0;
            // Trip-based calculation? sol.routes stores vector<Route>.
            // Need to sum trip distances.
            // Using existing logic for distance check:
            // Assuming sol.routes is the final list of routes.
            // Actually, we need to iterate over sol.routes[vid] which is vector<Route>
            // BUT wait, sol.routes[vid] contains Trips. Each Trip has distance_km.
            
            for (const auto& r : kv.second) {
                total_served += r.member_cells_linear.size();
                n_cust += r.member_cells_linear.size();
                
                // Need accurate trip distance including depot return?
                // Route struct has distance_km. Let's rely on it.
                // However, we should double check if it includes return-to-depot or we need to recalc.
                // Initializer seems to set distance_km.
                used_km += r.distance_km;
            }
            
            double max_km = itV->second.max_dist > 1e-9 ? itV->second.max_dist : 0.0;
            double pct = (max_km > 0) ? (used_km / max_km * 100.0) : 0.0;
            
            std::cout << std::left << std::setw(10) << vid 
                      << "| " << std::setw(16) << itV->second.vehicle_type
                      << "| " << std::setw(10) << std::fixed << std::setprecision(2) << used_km
                      << "| " << std::setw(10) << max_km
                      << "| " << std::fixed << std::setprecision(1) << pct << "%\n";
        }
        size_t unassigned = sol.unassigned_cells.size();
        size_t total = total_served + unassigned;

        std::cout << "--------------------------------\n";
        std::cout << "Initialization Result:\n";
        std::cout << "  Served:     " << total_served << "\n";
        std::cout << "  Unassigned: " << unassigned << "\n";
        std::cout << "  Total:      " << total << "\n";
        std::cout << "  Service %:  " << (total > 0 ? 100.0 * total_served / total : 0.0) << "%\n";
        std::cout << "--------------------------------\n";

        // Export Results
        std::cout << "Exporting results to: " << out_dir << "\n";
        
        // Sync to id_map for correct CSV export
        sync_solution_to_id_map(inst, sol);

        gridopt::ExportConfig cfg;
        cfg.out_dir = out_dir;
        cfg.assignment_filename = "init_assignment.csv"; // Distinct names
        cfg.routes_filename = "init_routes.csv";
        cfg.summary_filename = "init_summary.csv";
        cfg.add_timestamp = false;

        gridopt::export_all(inst, sol, cfg);

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
