#include "data_loader.h"
#include "initialization.h"   // grid-tensor initializer
#include "objectives.h"       // objective evaluation

#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <unordered_set>
#include <filesystem>
#include <ctime>

namespace fs = std::filesystem;

static std::string join_path(const std::string& dir, const std::string& name) {
    if (dir.empty()) return name;
    char last = dir.back();
    if (last == '/' || last == '\\') return dir + name;
    return dir + "/" + name;
}

static void ensure_dir(const std::string& dir) {
    if (dir.empty()) return;
    fs::create_directories(fs::path(dir));
}

static void write_assignments_csv(const gridopt::Instance& inst, const std::string& out_path) {
    std::ofstream fout(out_path);
    if (!fout.is_open()) throw std::runtime_error("Cannot write " + out_path);

    fout << "customer_idx,customer_id,row,col,cell_linear,weight,volume,priority,cluster,vehicle_id,depot_id\n";

    for (const auto& r : inst.id_map.rows) {
        if (r.customer_idx < 0) continue;
        if (r.customer_id.empty()) continue;
        if (r.row < 0 || r.col < 0) continue;
        if (!inst.customers.in_bounds(r.row, r.col)) continue;

        double w = 0.0, v = 0.0, p = 0.0;
        if (inst.customers.occupied(r.row, r.col)) {
            w = (double)inst.customers.at(r.row, r.col, gridopt::CUST_CH_W);
            v = (double)inst.customers.at(r.row, r.col, gridopt::CUST_CH_VOL);
            p = (double)inst.customers.at(r.row, r.col, gridopt::CUST_CH_PRI);
        }

        fout << r.customer_idx << ","
             << r.customer_id << ","
             << r.row << ","
             << r.col << ","
             << r.cell_linear << ","
             << std::fixed << std::setprecision(6) << w << ","
             << std::fixed << std::setprecision(6) << v << ","
             << std::fixed << std::setprecision(6) << p << ","
             << r.cluster << ","
             << r.vehicle_id << ","
             << r.depot_id << "\n";
    }
}

static void write_routes_csv(const gridopt::Instance& inst, const gridopt::Solution& sol, const std::string& out_path) {
    std::ofstream fout(out_path);
    if (!fout.is_open()) throw std::runtime_error("Cannot write " + out_path);

    fout << "vehicle_id,trip_idx,depot_id,center_cell_linear,center_row,center_col,center_customer_id,trip_km,member_cells,load_w,load_v\n";

    for (const auto& kv : sol.routes) {
        const std::string& vid = kv.first;
        auto itV = inst.vehicles.find(vid);
        std::string depot_id;
        if (itV != inst.vehicles.end()) depot_id = itV->second.start_depot;

        const auto& vec = kv.second;
        for (size_t t = 0; t < vec.size(); ++t) {
            const auto& rt = vec[t];

            int cr = -1, cc = -1;
            inst.customers.decode_linear(rt.center_cell_linear, cr, cc);

            std::string center_cid;
            gridopt::rep_customer_id_from_cell(inst, rt.center_cell_linear, center_cid);

            double trip_km = rt.distance_km;
            if (!center_cid.empty() && !depot_id.empty()) {
                double d = inst.get_dist_km(depot_id, center_cid);
                if (d < 1e8) trip_km = 2.0 * d;
            }

            fout << vid << ","
                 << (t + 1) << ","
                 << depot_id << ","
                 << rt.center_cell_linear << ","
                 << cr << ","
                 << cc << ","
                 << center_cid << ","
                 << std::fixed << std::setprecision(6) << trip_km << ","
                 << rt.member_cells_linear.size() << ","
                 << std::fixed << std::setprecision(6) << rt.load_w << ","
                 << std::fixed << std::setprecision(6) << rt.load_v << "\n";
        }
    }
}

static void print_solution_summary(const gridopt::Instance& inst, const gridopt::Solution& sol) {
    std::unordered_set<int> assigned_cells;
    size_t trip_count = 0;
    for (const auto& kv : sol.routes) {
        for (const auto& rt : kv.second) {
            ++trip_count;
            for (int cell : rt.member_cells_linear) assigned_cells.insert(cell);
        }
    }

    std::cout << "\n================ INIT SOLUTION SUMMARY ================\n";
    std::cout << "Vehicles with routes: " << sol.routes.size() << "\n";
    std::cout << "Trips (clusters):     " << trip_count << "\n";
    std::cout << "Assigned cells:       " << assigned_cells.size() << "\n";
    std::cout << "Unassigned cells:     " << sol.unassigned_cells.size() << "\n";

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Objective:            " << sol.objective << "\n";
    std::cout << "  fixed_cost:         " << sol.fixed_cost << "\n";
    std::cout << "  travel_cost:        " << sol.travel_cost << "\n";
    std::cout << "  penalty:            " << sol.penalty << "\n";

    // show a few example trips
    int shown = 0;
    for (const auto& kv : sol.routes) {
        const std::string& vid = kv.first;
        for (const auto& rt : kv.second) {
            if (shown >= 5) break;
            int cr=-1, cc=-1;
            inst.customers.decode_linear(rt.center_cell_linear, cr, cc);
            std::string center_cid;
            gridopt::rep_customer_id_from_cell(inst, rt.center_cell_linear, center_cid);
            std::cout << "  - vid=" << vid
                      << " center_cell=" << rt.center_cell_linear
                      << " (" << cr << "," << cc << ")"
                      << " center_cid=" << center_cid
                      << " members=" << rt.member_cells_linear.size()
                      << " load_w=" << rt.load_w
                      << " trip_km=" << rt.distance_km
                      << "\n";
            ++shown;
        }
        if (shown >= 5) break;
    }
    std::cout << "=======================================================\n";
}

int main(int argc, char** argv) {
    try {
        // Args:
        //   argv[1] = grid_dir   (default: grid_out)
        //   argv[2] = source_dir (default: grid_dir)
        //   argv[3] = out_dir    (default: grid_dir)
        //   argv[4] = seed       (default: current time - different each run)
        const std::string grid_dir   = (argc >= 2) ? argv[1] : "grid_out";
        const std::string source_dir = (argc >= 3) ? argv[2] : grid_dir;
        const std::string out_dir    = (argc >= 4) ? argv[3] : grid_dir;
        const int seed               = (argc >= 5) ? std::stoi(argv[4]) : (int)std::time(nullptr);

        ensure_dir(out_dir);

        std::cout << "========================================\n";
        std::cout << "  INIT TEST (grid-tensor seed-growing)\n";
        std::cout << "========================================\n";
        std::cout << "Grid dir:    " << grid_dir << "\n";
        std::cout << "Source dir:  " << source_dir << "\n";
        std::cout << "Out dir:     " << out_dir << "\n";
        std::cout << "Seed:        " << seed << "\n";

        std::cout << "\n--- Loading instance ---\n";
        gridopt::Instance inst = DataLoader::load_instance(grid_dir, source_dir);

        std::cout << "\n--- Building initial solution ---\n";
        gridopt::Solution sol = gridopt::Initializer::create_initial_solution(inst, seed);

        std::cout << "\n--- Evaluating objective (roads-only distance) ---\n";
        gridopt::Objectives::evaluate(inst, sol);

        print_solution_summary(inst, sol);

        const std::string out_assign = join_path(out_dir, "init_assignment.csv");
        const std::string out_routes = join_path(out_dir, "init_routes.csv");

        std::cout << "\n--- Writing outputs ---\n";
        write_assignments_csv(inst, out_assign);
        write_routes_csv(inst, sol, out_routes);

        std::cout << "Wrote: " << out_assign << "\n";
        std::cout << "Wrote: " << out_routes << "\n";
        std::cout << "\n[OK] Init test complete.\n";
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "\n[ERROR] " << e.what() << "\n";
        return 1;
    }
}
