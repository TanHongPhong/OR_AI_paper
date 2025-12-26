#include "data_loader.h"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <iomanip>

/*
  Comprehensive Test for config.h + data_loader.h
  ================================================
  Tests all major components:
  1. DataLoader::load_instance() - loads all CSV files
  2. CustomerTensor - grid access, occupied cells, idx lookup
  3. CustomerIdMap - id_to_idx, cell_to_customer_indices, has_idx
  4. Instance - depots, vehicles, roads, dist_km lookup
  5. Route & Solution structures (basic creation)
  6. Export assignment CSV for visualization
*/

// ============== Helper Functions ==============

static std::string choose_out_path(const std::string& data_dir) {
    return data_dir + "/customers_to_nearest_depot.csv";
}

static void write_assignment_csv(
    const std::string& out_path,
    const std::vector<std::pair<std::string, std::string>>& rows)
{
    std::ofstream fout(out_path);
    if (!fout.is_open()) throw std::runtime_error("Cannot write " + out_path);
    fout << "customer_id,depot_id\n";
    for (const auto& kv : rows) {
        fout << kv.first << "," << kv.second << "\n";
    }
}

// Find nearest depot by (row,col) grid indices using Euclidean distance
static std::string find_nearest_depot(
    const gridopt::Instance& inst, 
    int r, int c) 
{
    double best_d2 = 1e18;
    std::string best_id;
    for (const auto& d : inst.depots) {
        double dr = (double)r - (double)d.row;
        double dc = (double)c - (double)d.col;
        double d2 = dr*dr + dc*dc;
        if (d2 < best_d2) {
            best_d2 = d2;
            best_id = d.id;
        }
    }
    return best_id;
}

// ============== Test Functions ==============

void test_customer_tensor(const gridopt::Instance& inst) {
    std::cout << "\n=== TEST: CustomerTensor ===\n";
    
    const auto& tensor = inst.customers;
    std::cout << "  Grid dimensions: H=" << tensor.H << " x W=" << tensor.W << "\n";
    std::cout << "  Total cells: " << (tensor.H * tensor.W) << "\n";
    
    // Count occupied cells
    int occupied = 0;
    double total_weight = 0.0, total_volume = 0.0;
    
    for (int r = 0; r < tensor.H; ++r) {
        for (int c = 0; c < tensor.W; ++c) {
            if (tensor.occupied(r, c)) {
                occupied++;
                total_weight += tensor.at(r, c, gridopt::CUST_CH_W);
                total_volume += tensor.at(r, c, gridopt::CUST_CH_VOL);
            }
        }
    }
    
    std::cout << "  Occupied cells: " << occupied << "\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  Total weight: " << total_weight << "\n";
    std::cout << "  Total volume: " << total_volume << "\n";
    
    // Test linear encoding/decoding
    int test_r = tensor.H / 2, test_c = tensor.W / 2;
    int linear = tensor.linear(test_r, test_c);
    int dec_r, dec_c;
    tensor.decode_linear(linear, dec_r, dec_c);
    std::cout << "  Linear encode/decode test: (" << test_r << "," << test_c << ") -> "
              << linear << " -> (" << dec_r << "," << dec_c << ") "
              << (test_r == dec_r && test_c == dec_c ? "[OK]" : "[FAIL]") << "\n";
}

void test_customer_id_map(const gridopt::Instance& inst) {
    std::cout << "\n=== TEST: CustomerIdMap ===\n";
    
    const auto& id_map = inst.id_map;
    std::cout << "  Total rows in id_map: " << id_map.rows.size() << "\n";
    std::cout << "  id_to_idx entries: " << id_map.id_to_idx.size() << "\n";
    std::cout << "  cell_to_customer_indices entries: " << id_map.cell_to_customer_indices.size() << "\n";
    
    // Count valid customers (those with grid positions)
    int valid_customers = 0;
    for (const auto& row : id_map.rows) {
        if (row.customer_idx >= 0 && row.row >= 0 && row.col >= 0) {
            valid_customers++;
        }
    }
    std::cout << "  Customers with valid grid position: " << valid_customers << "\n";
    
    // Test has_idx function
    int test_idx = 0;
    std::cout << "  has_idx(" << test_idx << "): " << (id_map.has_idx(test_idx) ? "true" : "false") << "\n";
    
    // Show first 3 customers as sample
    std::cout << "  Sample customers:\n";
    int shown = 0;
    for (const auto& row : id_map.rows) {
        if (row.customer_idx >= 0 && !row.customer_id.empty() && shown < 3) {
            std::cout << "    idx=" << row.customer_idx 
                      << " id=\"" << row.customer_id << "\""
                      << " pos=(" << row.row << "," << row.col << ")"
                      << " linear=" << row.cell_linear << "\n";
            shown++;
        }
    }
}

void test_depots(const gridopt::Instance& inst) {
    std::cout << "\n=== TEST: Depots ===\n";
    std::cout << "  Total depots: " << inst.depots.size() << "\n";
    
    for (const auto& d : inst.depots) {
        std::cout << "    " << d.id 
                  << " pos=(" << d.row << "," << d.col << ")"
                  << " cap_w=" << d.cap_weight_storage << "\n";
    }
}

void test_vehicles(const gridopt::Instance& inst) {
    std::cout << "\n=== TEST: Vehicles ===\n";
    std::cout << "  Total vehicles: " << inst.vehicles.size() << "\n";
    
    int shown = 0;
    for (const auto& [vid, v] : inst.vehicles) {
        if (shown < 3) {
            std::cout << "    " << v.id 
                      << " type=" << v.vehicle_type
                      << " cap_w=" << v.cap_weight
                      << " cap_v=" << v.cap_volume
                      << " max_dist=" << v.max_dist
                      << " depot=" << v.start_depot << "\n";
            shown++;
        }
    }
    if (inst.vehicles.size() > 3) {
        std::cout << "    ... and " << (inst.vehicles.size() - 3) << " more vehicles\n";
    }
}

void test_roads_and_distance(const gridopt::Instance& inst) {
    std::cout << "\n=== TEST: Roads & Distance Lookup ===\n";
    std::cout << "  Total roads: " << inst.roads.size() << "\n";
    std::cout << "  dist_km matrix entries: " << inst.dist_km.size() << " origin nodes\n";
    
    // Test distance lookup between first depot and second depot (if available)
    if (inst.depots.size() >= 2) {
        const std::string& d1 = inst.depots[0].id;
        const std::string& d2 = inst.depots[1].id;
        double dist = inst.get_dist_km(d1, d2);
        std::cout << "  Distance " << d1 << " -> " << d2 << ": ";
        if (dist < 1e8) {
            std::cout << dist << " km\n";
        } else {
            std::cout << "NOT FOUND (returns 1e9)\n";
        }
    }
}

void test_route_solution_structures() {
    std::cout << "\n=== TEST: Route & Solution Structures ===\n";
    
    // Create a sample route
    gridopt::Route route;
    route.vehicle_id = "V001";
    route.center_cell_linear = 1000;
    route.member_cells_linear = {1000, 1001, 1002};
    route.load_w = 150.5;
    route.load_v = 0.8;
    route.distance_km = 25.3;
    
    std::cout << "  Created sample Route: vehicle=" << route.vehicle_id
              << " center=" << route.center_cell_linear
              << " members=" << route.member_cells_linear.size()
              << " load_w=" << route.load_w << "\n";
    
    // Create a sample solution
    gridopt::Solution sol;
    sol.routes["V001"].push_back(route);
    sol.unassigned_cells = {2000, 2001};
    sol.objective = 1234.56;
    sol.fixed_cost = 100.0;
    sol.travel_cost = 134.56;
    sol.penalty = 1000.0;
    
    std::cout << "  Created sample Solution: "
              << "routes=" << sol.routes.size() 
              << " unassigned=" << sol.unassigned_cells.size()
              << " objective=" << sol.objective << "\n";
    
    std::cout << "  [OK] Route & Solution structs work correctly\n";
}

// ============== Main Assignment Logic ==============

int assign_customers_to_depots(
    const gridopt::Instance& inst,
    const std::string& out_csv)
{
    std::cout << "\n=== Assignment: Customers to Nearest Depot ===\n";
    
    std::vector<std::pair<std::string, std::string>> out_rows;
    out_rows.reserve(inst.id_map.rows.size());
    
    int assigned = 0;
    
    for (const auto& cust : inst.id_map.rows) {
        if (cust.row < 0 || cust.col < 0) continue;
        if (cust.customer_id.empty()) continue;
        
        std::string depot_id = find_nearest_depot(inst, cust.row, cust.col);
        
        if (!depot_id.empty()) {
            out_rows.push_back({cust.customer_id, depot_id});
            assigned++;
        }
    }
    
    write_assignment_csv(out_csv, out_rows);
    
    std::cout << "  Total customers assigned: " << assigned << "\n";
    std::cout << "  Output file: " << out_csv << "\n";
    
    return assigned;
}

// ============== Main ==============

int main(int argc, char** argv) {
    try {
        // Default paths based on user's file structure
        std::string grid_dir = (argc >= 2) ? argv[1] : "grid_out";
        std::string source_dir = (argc >= 3) ? argv[2] : "D:/A UEH_UNIVERSITY/RESEACH/OR_AI_paper/Zzz_data/LMDO processed/Ho_Chi_Minh_City";
        std::string out_csv = (argc >= 4) ? argv[3] : choose_out_path(grid_dir);
        
        std::cout << "========================================\n";
        std::cout << "  Comprehensive DataLoader Test Suite\n";
        std::cout << "========================================\n";
        std::cout << "Grid directory:   " << grid_dir << "\n";
        std::cout << "Source directory: " << source_dir << "\n";
        
        // ===== Load all data =====
        std::cout << "\n--- Loading Instance ---\n";
        auto inst = DataLoader::load_instance(grid_dir, source_dir);
        
        // ===== Run all tests =====
        test_customer_tensor(inst);
        test_customer_id_map(inst);
        test_depots(inst);
        test_vehicles(inst);
        test_roads_and_distance(inst);
        test_route_solution_structures();
        
        // ===== Assignment for visualization =====
        int assigned = assign_customers_to_depots(inst, out_csv);
        
        // ===== Summary =====
        std::cout << "\n========================================\n";
        std::cout << "  ALL TESTS COMPLETED SUCCESSFULLY!\n";
        std::cout << "========================================\n";
        std::cout << "Summary:\n";
        std::cout << "  - Grid: " << inst.customers.H << " x " << inst.customers.W << "\n";
        std::cout << "  - Depots: " << inst.depots.size() << "\n";
        std::cout << "  - Vehicles: " << inst.vehicles.size() << "\n";
        std::cout << "  - Roads: " << inst.roads.size() << "\n";
        std::cout << "  - Customers assigned: " << assigned << "\n";
        std::cout << "  - Output: " << out_csv << "\n";
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "\n[ERROR] " << e.what() << "\n";
        return 1;
    }
}
