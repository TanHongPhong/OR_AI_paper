#include "data_loader_grid.h"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

/*
  Test:
  - Load dense grid customers + depots (row/col)
  - Assign each CUSTOMER (original id_idx) to nearest depot by squared distance in (row,col)
  - Export CSV with 2 columns: customer_id_idx,depot_id
*/

static std::string choose_out_path(const std::string& grid_dir) {
    return gridio::join_path(grid_dir, "customers_to_nearest_depot.csv");
}

static void write_assignment_csv(
    const std::string& out_path,
    const std::vector<std::pair<std::string, std::string>>& rows)
{
    std::ofstream fout(out_path);
    if (!fout.is_open()) throw std::runtime_error("Cannot write " + out_path);
    fout << "customer_id_idx,depot_id\n";
    for (const auto& kv : rows) {
        fout << kv.first << "," << kv.second << "\n";
    }
}

// nearest depot by (row,col) grid indices
static std::string depot_nearest_for_cell(const gridio::GridData& gd, int r, int c) {
    long long best_d2 = (1LL << 62);
    std::string best_id;

    for (const auto& d : gd.depots) {
        long long dr = (long long)r - (long long)d.row;
        long long dc = (long long)c - (long long)d.col;
        long long d2 = dr*dr + dc*dc;
        if (d2 < best_d2) {
            best_d2 = d2;
            best_id = d.depot_id;
        }
    }
    return best_id;
}

int main(int argc, char** argv) {
    try {
        std::string grid_dir = (argc >= 2) ? argv[1] : ".";
        std::string out_csv = (argc >= 3) ? argv[2] : choose_out_path(grid_dir);

        auto gd = gridio::load_grid_data(grid_dir);

        std::vector<std::pair<std::string, std::string>> out_rows;
        out_rows.reserve(4096);

        // For each occupied cell -> nearest depot -> output for each Customer_ID_idx token in that cell
        for (const auto& rc : gd.occupied) {
            int r = rc.first, c = rc.second;
            const auto& cell = gd.at(r, c);
            std::string depot_id = depot_nearest_for_cell(gd, r, c);

            // Prefer customer_id_idx tokens; if empty, fall back to customer_id; if empty, use "r_c"
            std::string key_str = !cell.customer_id_idx.empty() ? cell.customer_id_idx
                                : (!cell.customer_id.empty() ? cell.customer_id
                                : (std::to_string(r) + "_" + std::to_string(c)));

            auto tokens = gridio::split_space_tokens(key_str);
            if (tokens.empty()) {
                out_rows.push_back({key_str, depot_id});
            } else {
                for (const auto& t : tokens) out_rows.push_back({t, depot_id});
            }
        }

        write_assignment_csv(out_csv, out_rows);

        std::cout << "[OK] Loaded grid:\n";
        std::cout << "  - height=" << gd.meta.height << " width=" << gd.meta.width
                  << " cell_km=" << gd.meta.cell_km << "\n";
        std::cout << "  - depots=" << gd.depots.size() << "\n";
        std::cout << "  - occupied_cells=" << gd.occupied.size() << "\n";
        std::cout << "[OK] Wrote: " << out_csv << "\n";
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << "\n";
        return 1;
    }
}
