#pragma once

#include "config.h"

#include <fstream>
#include <iostream>
#include <filesystem>
#include <sstream>
#include <cctype>

// =============================================================
//  DataLoader (GRID-TENSOR edition)
//
//  Loads (new grid inputs):
//    - customers_sparse_grid.csv
//    - depots_grid_min.csv
//    - customer_id_map.csv
//  plus keeps old inputs unchanged:
//    - vehicles__Sheet1.csv or vehicles.csv
//    - roads__Sheet1.csv or roads.csv
//
//  After loading:
//    - inst.customers is dense tensor [H,W,4]
//    - inst.depots / inst.vehicles / inst.roads are tables
//    - inst.id_map is loaded + linked to grid cell positions
// =============================================================

class DataLoader {
public:
    // Load with separate directories for grid data and source data (vehicles/roads)
    static gridopt::Instance load_instance(
        const std::string& grid_dir,      // customer_id_map.csv, depots_grid_min.csv, customers_sparse_grid.csv
        const std::string& source_dir)    // vehicles.csv, roads.csv
    {
        gridopt::Instance inst;

        // Grid data (from grid_dir)
        load_customer_id_map(inst, grid_dir);
        load_depots_grid(inst, grid_dir);
        load_customers_tensor(inst, grid_dir);

        // Source data (from source_dir)
        load_vehicles(inst, source_dir);
        load_roads(inst, source_dir);
        inst.build_dist_matrix();

        return inst;
    }

    // Convenience: single directory for all files
    static gridopt::Instance load_instance(const std::string& data_dir) {
        return load_instance(data_dir, data_dir);
    }

private:
    // ---------------- helpers ----------------
    static inline void strip_utf8_bom(std::string& s) {
        if (s.size() >= 3 && (unsigned char)s[0] == 0xEF && (unsigned char)s[1] == 0xBB && (unsigned char)s[2] == 0xBF) {
            s.erase(0, 3);
        }
    }

    static inline std::string trim(const std::string& s) {
        size_t a = 0, b = s.size();
        while (a < b && std::isspace((unsigned char)s[a])) ++a;
        while (b > a && std::isspace((unsigned char)s[b - 1])) --b;
        return s.substr(a, b - a);
    }

    static inline std::string lower(std::string s) {
        for (auto &ch : s) ch = (char)std::tolower((unsigned char)ch);
        return s;
    }

    static inline std::vector<std::string> split_simple(const std::string& line, char sep) {
        std::vector<std::string> out;
        std::string cur;
        cur.reserve(line.size());
        for (char ch : line) {
            if (ch == sep) {
                out.push_back(cur);
                cur.clear();
            } else {
                cur.push_back(ch);
            }
        }
        out.push_back(cur);
        for (auto &x : out) x = trim(x);
        return out;
    }

    static inline std::vector<std::string> split_ws(const std::string& s) {
        std::vector<std::string> out;
        std::string cur;
        for (char ch : s) {
            if (std::isspace((unsigned char)ch)) {
                if (!cur.empty()) {
                    out.push_back(cur);
                    cur.clear();
                }
            } else {
                cur.push_back(ch);
            }
        }
        if (!cur.empty()) out.push_back(cur);
        return out;
    }

    // Simple path join: data_dir + "/" + filename
    static std::string join_path(const std::string& dir, const std::string& filename) {
        if (dir.empty()) return filename;
        char last = dir.back();
        if (last == '/' || last == '\\') return dir + filename;
        return dir + "/" + filename;
    }

    // Check if file exists at path
    static bool file_exists(const std::string& path) {
        return std::filesystem::exists(path);
    }

    static int col_index(const std::vector<std::string>& header, const std::string& name) {
        std::string target = lower(name);
        for (int i = 0; i < (int)header.size(); ++i) {
            std::string h = lower(header[i]);
            strip_utf8_bom(h);
            if (trim(h) == target) return i;
        }
        return -1;
    }

    static int to_int(const std::string& s, int fallback = 0) {
        try {
            return std::stoi(trim(s));
        } catch (...) {
            return fallback;
        }
    }

    static double to_double(const std::string& s, double fallback = 0.0) {
        try {
            return std::stod(trim(s));
        } catch (...) {
            return fallback;
        }
    }

    // ---------------- loaders ----------------

    static void load_customer_id_map(gridopt::Instance& inst, const std::string& dir) {
        // Expect file at: data_dir/customer_id_map.csv
        std::string path = join_path(dir, "customer_id_map.csv");
        if (!file_exists(path)) throw std::runtime_error("Cannot find: " + path);

        std::ifstream fin(path);
        if (!fin.is_open()) throw std::runtime_error("Cannot open " + path);

        std::string line;
        if (!std::getline(fin, line)) throw std::runtime_error("Empty customer_id_map.csv");
        strip_utf8_bom(line);
        auto header = split_simple(line, ',');

        int idx_i = col_index(header, "customer_id_idx");
        int id_i  = col_index(header, "customer_id");
        if (idx_i < 0) idx_i = 0;
        if (id_i < 0)  id_i  = 1;

        inst.id_map.rows.clear();
        inst.id_map.id_to_idx.clear();
        inst.id_map.cell_to_customer_indices.clear();

        while (std::getline(fin, line)) {
            if (line.empty()) continue;
            auto row = split_simple(line, ',');
            if ((int)row.size() <= std::max(idx_i, id_i)) continue;

            int idx = to_int(row[idx_i], -1);
            if (idx < 0) continue;
            std::string cid = trim(row[id_i]);

            inst.id_map.ensure_size(idx + 1);
            auto &r = inst.id_map.rows[(size_t)idx];
            r.customer_idx = idx;
            r.customer_id = cid;
            // other fields stay default until we link to grid

            if (!cid.empty()) inst.id_map.id_to_idx[cid] = idx;
        }

        if (inst.id_map.rows.empty()) {
            throw std::runtime_error("No rows loaded from customer_id_map.csv");
        }

        std::cout << "[DataLoader] Loaded customer_id_map rows: " << inst.id_map.rows.size() << " (" << path << ")\n";
    }

    static void load_depots_grid(gridopt::Instance& inst, const std::string& dir) {
        // Expect file at: data_dir/depots_grid_min.csv
        std::string path = join_path(dir, "depots_grid_min.csv");
        if (!file_exists(path)) throw std::runtime_error("Cannot find: " + path);

        std::ifstream fin(path);
        if (!fin.is_open()) throw std::runtime_error("Cannot open " + path);

        std::string line;
        if (!std::getline(fin, line)) throw std::runtime_error("Empty depots_grid_min.csv");
        strip_utf8_bom(line);
        auto header = split_simple(line, ',');

        // case-insensitive (col_index lowercases header + target)
        int id_i  = col_index(header, "depot_id");
        if (id_i < 0) id_i = 0;

        int row_i = col_index(header, "row");
        int col_i = col_index(header, "col");
        int cap_i = col_index(header, "capacity_storage");
        int capv_i = col_index(header, "capacity_volume"); // optional

        if (row_i < 0 || col_i < 0) throw std::runtime_error("depots_grid_min.csv must have row,col");

        inst.depots.clear();
        while (std::getline(fin, line)) {
            if (line.empty()) continue;
            auto row = split_simple(line, ',');
            if ((int)row.size() <= std::max({id_i, row_i, col_i, cap_i, capv_i})) continue;

            gridopt::Depot d;
            d.id = trim(row[id_i]);
            d.row = to_int(row[row_i], 0);
            d.col = to_int(row[col_i], 0);
            if (cap_i >= 0) d.cap_weight_storage = to_double(row[cap_i], 0.0);

            inst.depots.push_back(d);
        }

        if (inst.depots.empty()) throw std::runtime_error("No depots loaded from depots_grid_min.csv");
        std::cout << "[DataLoader] Loaded depots: " << inst.depots.size() << " (" << path << ")\n";
    }

    static void load_customers_tensor(gridopt::Instance& inst, const std::string& dir) {
        // Expect file at: data_dir/customers_sparse_grid.csv
        std::string path = join_path(dir, "customers_sparse_grid.csv");
        if (!file_exists(path)) throw std::runtime_error("Cannot find: " + path);

        struct TempCell {
            int r = 0;
            int c = 0;
            float w = 0.0f;
            float v = 0.0f;
            float p = 0.0f;
            std::string idx_list; // space separated customer idx tokens
        };

        std::ifstream fin(path);
        if (!fin.is_open()) throw std::runtime_error("Cannot open " + path);

        std::string line;
        if (!std::getline(fin, line)) throw std::runtime_error("Empty customers_sparse_grid.csv");
        strip_utf8_bom(line);
        auto header = split_simple(line, ',');

        int r_i = col_index(header, "row");
        int c_i = col_index(header, "col");
        int w_i = col_index(header, "weight_sum");
        int v_i = col_index(header, "volume_sum");
        int p_i = col_index(header, "priority_max");
        int idx_i = col_index(header, "customer_id_idx");

        if (r_i < 0 || c_i < 0) throw std::runtime_error("customers_sparse_grid.csv must have row,col");
        if (w_i < 0) w_i = col_index(header, "weight");
        if (v_i < 0) v_i = col_index(header, "volume");
        if (p_i < 0) p_i = col_index(header, "priority");

        if (w_i < 0 || v_i < 0 || p_i < 0) {
            throw std::runtime_error("customers_sparse_grid.csv must have weight_sum/volume_sum/priority_max (or weight/volume/priority)");
        }

        std::vector<TempCell> cells;
        cells.reserve(2048);

        int max_r = -1, max_c = -1;

        while (std::getline(fin, line)) {
            if (line.empty()) continue;
            auto row = split_simple(line, ',');
            if ((int)row.size() <= std::max({r_i, c_i, w_i, v_i, p_i, idx_i})) continue;

            TempCell t;
            t.r = to_int(row[r_i], 0);
            t.c = to_int(row[c_i], 0);
            t.w = (float)to_double(row[w_i], 0.0);
            t.v = (float)to_double(row[v_i], 0.0);
            t.p = (float)to_double(row[p_i], 0.0);
            if (idx_i >= 0 && idx_i < (int)row.size()) t.idx_list = trim(row[idx_i]);

            cells.push_back(t);
            max_r = std::max(max_r, t.r);
            max_c = std::max(max_c, t.c);
        }

        if (cells.empty()) throw std::runtime_error("No sparse cells loaded from customers_sparse_grid.csv");

        // Allocate dense tensor
        const int H = max_r + 1;
        const int W = max_c + 1;
        inst.customers.resize(H, W);

        // Fill tensor + link id_map rows to cell positions
        for (const auto& t : cells) {
            if (!inst.customers.in_bounds(t.r, t.c)) continue;

            inst.customers.at(t.r, t.c, gridopt::CUST_CH_W) = t.w;
            inst.customers.at(t.r, t.c, gridopt::CUST_CH_VOL) = t.v;
            inst.customers.at(t.r, t.c, gridopt::CUST_CH_PRI) = t.p;

            // Link each customer_idx token in that cell to (row,col,cell_linear).
            // Tensor idx channel stores a *representative customer_idx* (min idx in that cell)
            // so it stays compatible with customer_id_map.csv and remains in a safe numeric range.
            const int cell_linear = inst.customers.linear(t.r, t.c);
            int rep_idx = -1;
            if (!t.idx_list.empty()) {
                for (const auto& tok : split_ws(t.idx_list)) {
                    int cust_idx = to_int(tok, -1);
                    if (cust_idx < 0) continue;
                    if (rep_idx < 0 || cust_idx < rep_idx) rep_idx = cust_idx;

                    inst.id_map.ensure_size(cust_idx + 1);
                    auto &mr = inst.id_map.rows[(size_t)cust_idx];
                    if (mr.customer_idx != cust_idx) mr.customer_idx = cust_idx;
                    mr.row = t.r;
                    mr.col = t.c;
                    mr.cell_linear = cell_linear;
                    inst.id_map.cell_to_customer_indices[cell_linear].push_back(cust_idx);
                }
            }

            inst.customers.at(t.r, t.c, gridopt::CUST_CH_IDX) = (float)rep_idx;
        }

        std::cout << "[DataLoader] Loaded customer tensor: H=" << H << " W=" << W
                  << " (occupied sparse cells=" << cells.size() << ") (" << path << ")\n";
        std::cout << "[DataLoader] Linked cells in id_map: " << inst.id_map.cell_to_customer_indices.size() << "\n";
    }

    // ---------------- vehicles / roads (optional) ----------------

    static void load_vehicles(gridopt::Instance& inst, const std::string& dir) {
        // Expect file at: source_dir/vehicles__Sheet1.csv (optional)
        std::string path = join_path(dir, "vehicles__Sheet1.csv");
        if (!file_exists(path)) {
            std::cout << "[DataLoader] No vehicles__Sheet1.csv found at: " << path << "\n";
            return;
        }

        std::ifstream fin(path);
        if (!fin.is_open()) throw std::runtime_error("Cannot open " + path);

        std::string line;
        if (!std::getline(fin, line)) throw std::runtime_error("Empty vehicles file");
        strip_utf8_bom(line);
        auto header = split_simple(line, ',');

        int id_i  = col_index(header, "vehicle_id"); if (id_i < 0) id_i = 0;
        int type_i = col_index(header, "vehicle_type");
        int w_i   = col_index(header, "capacity_weight");
        int v_i   = col_index(header, "capacity_volume");
        int fc_i  = col_index(header, "fixed_cost");
        int vc_i  = col_index(header, "variable_cost");
        int md_i  = col_index(header, "max_distance");
        int sd_i  = col_index(header, "start_depot_id");
        int ed_i  = col_index(header, "end_depot_id");

        inst.vehicles.clear();
        while (std::getline(fin, line)) {
            if (line.empty()) continue;
            auto row = split_simple(line, ',');
            if ((int)row.size() <= std::max({id_i, type_i, w_i, v_i, fc_i, vc_i, md_i, sd_i, ed_i})) continue;

            gridopt::Vehicle v;
            v.id = trim(row[id_i]);
            if (type_i >= 0 && type_i < (int)row.size()) v.vehicle_type = trim(row[type_i]);
            v.type = gridopt::parse_vehicle_type(v.vehicle_type);

            if (w_i >= 0 && w_i < (int)row.size()) v.cap_weight = to_double(row[w_i], 0.0);
            if (v_i >= 0 && v_i < (int)row.size()) v.cap_volume = to_double(row[v_i], 0.0);
            if (fc_i >= 0 && fc_i < (int)row.size()) v.fixed_cost = to_double(row[fc_i], 0.0);
            if (vc_i >= 0 && vc_i < (int)row.size()) v.var_cost = to_double(row[vc_i], 0.0);
            if (md_i >= 0 && md_i < (int)row.size()) v.max_dist = to_double(row[md_i], 0.0);

            if (sd_i >= 0 && sd_i < (int)row.size()) v.start_depot = trim(row[sd_i]);
            if (ed_i >= 0 && ed_i < (int)row.size()) v.end_depot = trim(row[ed_i]);
            if (v.end_depot.empty()) v.end_depot = v.start_depot;

            inst.vehicles[v.id] = v;
        }

        if (inst.vehicles.empty()) {
            std::cout << "[DataLoader] Warning: No vehicles parsed from " << path << "\n";
        } else {
            std::cout << "[DataLoader] Loaded vehicles: " << inst.vehicles.size() << " (" << path << ")\n";
            
            // Fleet count diagnostic
            std::unordered_map<std::string, int> fleet_count;
            for (const auto& kv : inst.vehicles) {
                fleet_count[kv.second.start_depot]++;
            }
            std::cout << "[DataLoader] Fleet by depot:\n";
            for (const auto& fc : fleet_count) {
                std::cout << "  " << fc.first << ": " << fc.second << " vehicles\n";
            }
        }
    }

    static void load_roads(gridopt::Instance& inst, const std::string& dir) {
        // Expect file at: source_dir/roads__Sheet1.csv (optional)
        std::string path = join_path(dir, "roads__Sheet1.csv");
        if (!file_exists(path)) {
            std::cout << "[DataLoader] No roads__Sheet1.csv found at: " << path << "\n";
            return;
        }

        std::ifstream fin(path);
        if (!fin.is_open()) throw std::runtime_error("Cannot open " + path);

        std::string line;
        if (!std::getline(fin, line)) return;
        strip_utf8_bom(line);
        auto header = split_simple(line, ',');

        int from_i = col_index(header, "origin_node_id"); if (from_i < 0) from_i = col_index(header, "from");
        int to_i   = col_index(header, "destination_node_id"); if (to_i < 0) to_i = col_index(header, "to");
        int dist_i = col_index(header, "distance_km");
        int time_i = col_index(header, "travel_time_min");
        int traf_i = col_index(header, "traffic_level");
        int rest_i = col_index(header, "road_restrictions");
        int vel_i  = col_index(header, "velocity");
        if (from_i < 0) from_i = 0;
        if (to_i < 0) to_i = 1;

        inst.roads.clear();
        int count = 0;
        while (std::getline(fin, line)) {
            if (line.empty()) continue;
            auto row = split_simple(line, ',');
            if ((int)row.size() <= std::max({from_i, to_i, dist_i})) continue;

            gridopt::Road r;
            r.from = trim(row[from_i]);
            r.to = trim(row[to_i]);
            if (dist_i >= 0 && dist_i < (int)row.size()) r.distance_km = to_double(row[dist_i], 0.0);
            if (time_i >= 0 && time_i < (int)row.size()) r.travel_time_min = to_double(row[time_i], 0.0);
            if (traf_i >= 0 && traf_i < (int)row.size()) r.traffic_level = trim(row[traf_i]);
            if (rest_i >= 0 && rest_i < (int)row.size()) r.restrictions = trim(row[rest_i]);
            if (vel_i >= 0 && vel_i < (int)row.size()) r.velocity = to_double(row[vel_i], 0.0);

            inst.roads.push_back(r);
            count++;
        }

        std::cout << "[DataLoader] Loaded roads: " << count << " (" << path << ")\n";
    }
};
