#pragma once
/*
  Dense Grid DataLoader for region-growing optimizer

  Reads Python outputs:
    - customers_sparse_grid.csv
        row,col,weight_sum,volume_sum,priority_max,count,customer_id,customer_id_idx
    - depots_grid_min.csv
        Depot_ID,row,col,Capacity_Storage
    - (optional) grid_meta.txt
        ... width=..., height=..., cell_km=...

  Customer grid is stored as a dense 2D matrix (height x width) where each cell contains:
    weight_sum / volume_sum / priority_max / count / customer_id / customer_id_idx

  Notes:
  - row/col are GRID INDICES (matrix coordinates), not lat/lon.
  - Vehicles & roads are intentionally not handled here (keep your existing loader for those).
*/

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace gridio {

inline void strip_utf8_bom(std::string& s) {
    const unsigned char bom[] = {0xEF, 0xBB, 0xBF};
    if (s.size() >= 3 &&
        (unsigned char)s[0] == bom[0] &&
        (unsigned char)s[1] == bom[1] &&
        (unsigned char)s[2] == bom[2]) {
        s.erase(0, 3);
    }
}

inline std::string lower(std::string s) {
    for (char& ch : s) ch = (char)std::tolower((unsigned char)ch);
    return s;
}

inline std::string trim(std::string s) {
    auto issp = [](unsigned char c) { return std::isspace(c) != 0; };
    while (!s.empty() && issp((unsigned char)s.front())) s.erase(s.begin());
    while (!s.empty() && issp((unsigned char)s.back())) s.pop_back();
    return s;
}

// CSV row parser that supports quoted fields (no embedded newlines).
inline std::vector<std::string> parse_csv_row(const std::string& line) {
    std::vector<std::string> out;
    std::string cur;
    bool in_quotes = false;

    for (size_t i = 0; i < line.size(); ++i) {
        char c = line[i];
        if (in_quotes) {
            if (c == '"') {
                // escaped quote
                if (i + 1 < line.size() && line[i + 1] == '"') {
                    cur.push_back('"');
                    ++i;
                } else {
                    in_quotes = false;
                }
            } else {
                cur.push_back(c);
            }
        } else {
            if (c == '"') {
                in_quotes = true;
            } else if (c == ',') {
                out.push_back(trim(cur));
                cur.clear();
            } else {
                cur.push_back(c);
            }
        }
    }
    out.push_back(trim(cur));
    return out;
}

inline int col_index(const std::vector<std::string>& header, const std::string& name) {
    std::string target = lower(name);
    for (int i = 0; i < (int)header.size(); ++i) {
        std::string h = header[i];
        strip_utf8_bom(h);
        h = lower(trim(h));
        if (h == target) return i;
    }
    return -1;
}

inline std::vector<std::string> split_space_tokens(const std::string& s) {
    std::vector<std::string> tok;
    std::stringstream ss(s);
    std::string t;
    while (ss >> t) tok.push_back(t);
    return tok;
}

struct GridMeta {
    double cell_km = 0.01; // default 10m
    int width = 0;         // max col + 1
    int height = 0;        // max row + 1
};

struct GridCell {
    // (row,col) is implicit by index in dense matrix
    double weight_sum = 0.0;
    double volume_sum = 0.0;
    int priority_max = 0;
    int count = 0;
    std::string customer_id;      // space-separated if collision
    std::string customer_id_idx;  // space-separated if collision
    bool occupied = false;
};

struct GridDepot {
    std::string depot_id;
    int row = 0;
    int col = 0;
    double capacity_storage = 0.0;
};

struct GridData {
    GridMeta meta;
    std::vector<GridCell> grid;                 // size = height * width
    std::vector<std::pair<int,int>> occupied;   // occupied (row,col) cells
    std::vector<GridDepot> depots;

    inline int idx(int row, int col) const { return row * meta.width + col; }

    inline bool in_bounds(int row, int col) const {
        return row >= 0 && col >= 0 && row < meta.height && col < meta.width;
    }

    inline const GridCell& at(int row, int col) const { return grid[idx(row, col)]; }
    inline GridCell& at(int row, int col) { return grid[idx(row, col)]; }
};

inline bool file_exists(const std::string& p) {
    return std::filesystem::exists(std::filesystem::path(p));
}

inline std::string join_path(const std::string& a, const std::string& b) {
    return (std::filesystem::path(a) / std::filesystem::path(b)).string();
}

inline GridMeta load_meta_if_any(const std::string& grid_dir) {
    GridMeta meta;
    std::string meta_path = join_path(grid_dir, "grid_meta.txt");
    if (!file_exists(meta_path)) return meta;

    std::ifstream fin(meta_path);
    if (!fin.is_open()) return meta;

    std::string line;
    while (std::getline(fin, line)) {
        strip_utf8_bom(line);
        line = trim(line);
        if (line.empty()) continue;
        auto pos = line.find('=');
        if (pos == std::string::npos) continue;

        std::string k = lower(trim(line.substr(0, pos)));
        std::string v = trim(line.substr(pos + 1));

        try {
            if (k == "cell_km") meta.cell_km = std::stod(v);
            else if (k == "width") meta.width = std::stoi(v);
            else if (k == "height") meta.height = std::stoi(v);
        } catch (...) {
            // ignore
        }
    }
    return meta;
}

inline void ensure_grid_alloc(GridData& gd) {
    if (gd.meta.width <= 0 || gd.meta.height <= 0) {
        throw std::runtime_error("[GridDataLoader] meta width/height not set");
    }
    gd.grid.assign((size_t)gd.meta.width * (size_t)gd.meta.height, GridCell{});
    gd.occupied.clear();
}

inline void load_depots_min(GridData& gd, const std::string& grid_dir) {
    std::string path = join_path(grid_dir, "depots_grid_min.csv");
    if (!file_exists(path)) {
        throw std::runtime_error("[GridDataLoader] Cannot find depots_grid_min.csv in " + grid_dir);
    }

    std::ifstream fin(path);
    if (!fin.is_open()) throw std::runtime_error("[GridDataLoader] Cannot open " + path);

    std::string line;
    if (!std::getline(fin, line)) throw std::runtime_error("[GridDataLoader] Empty " + path);
    strip_utf8_bom(line);
    auto header = parse_csv_row(line);

    int id_i = col_index(header, "depot_id"); if (id_i < 0) id_i = 0;
    int r_i  = col_index(header, "row");
    int c_i  = col_index(header, "col");
    int cap_i = col_index(header, "capacity_storage");

    gd.depots.clear();
    int max_r = -1, max_c = -1;

    while (std::getline(fin, line)) {
        if (line.empty()) continue;
        auto row = parse_csv_row(line);
        if ((int)row.size() <= std::max({id_i, r_i, c_i, cap_i})) continue;

        GridDepot d;
        d.depot_id = trim(row[id_i]);
        if (r_i >= 0) d.row = std::stoi(row[r_i]);
        if (c_i >= 0) d.col = std::stoi(row[c_i]);
        if (cap_i >= 0 && cap_i < (int)row.size() && !row[cap_i].empty()) d.capacity_storage = std::stod(row[cap_i]);

        gd.depots.push_back(d);
        max_r = std::max(max_r, d.row);
        max_c = std::max(max_c, d.col);
    }

    // If meta missing, infer size at least covering depots.
    if (gd.meta.height <= 0) gd.meta.height = max_r + 1;
    if (gd.meta.width <= 0) gd.meta.width = max_c + 1;

    if (gd.depots.empty()) {
        throw std::runtime_error("[GridDataLoader] No depots loaded from " + path);
    }
}

inline void load_customers_sparse_into_dense(GridData& gd, const std::string& grid_dir) {
    std::string path = join_path(grid_dir, "customers_sparse_grid.csv");
    if (!file_exists(path)) {
        throw std::runtime_error("[GridDataLoader] Cannot find customers_sparse_grid.csv in " + grid_dir);
    }

    struct TempRow {
        int r, c;
        double w, v;
        int p, cnt;
        std::string id, idx;
    };
    std::vector<TempRow> tmp;
    tmp.reserve(2048);

    std::ifstream fin(path);
    if (!fin.is_open()) throw std::runtime_error("[GridDataLoader] Cannot open " + path);

    std::string line;
    if (!std::getline(fin, line)) throw std::runtime_error("[GridDataLoader] Empty " + path);
    strip_utf8_bom(line);
    auto header = parse_csv_row(line);

    int r_i = col_index(header, "row"); if (r_i < 0) r_i = 0;
    int c_i = col_index(header, "col"); if (c_i < 0) c_i = 1;
    int w_i = col_index(header, "weight_sum");
    int v_i = col_index(header, "volume_sum");
    int p_i = col_index(header, "priority_max");
    int cnt_i = col_index(header, "count");
    int id_i = col_index(header, "customer_id");
    int idx_i = col_index(header, "customer_id_idx");

    int max_r = -1, max_c = -1;

    while (std::getline(fin, line)) {
        if (line.empty()) continue;
        auto row = parse_csv_row(line);
        if ((int)row.size() <= std::max({r_i, c_i})) continue;

        TempRow t;
        t.r = std::stoi(row[r_i]);
        t.c = std::stoi(row[c_i]);
        t.w = (w_i >= 0 && w_i < (int)row.size() && !row[w_i].empty()) ? std::stod(row[w_i]) : 0.0;
        t.v = (v_i >= 0 && v_i < (int)row.size() && !row[v_i].empty()) ? std::stod(row[v_i]) : 0.0;
        t.p = (p_i >= 0 && p_i < (int)row.size() && !row[p_i].empty()) ? (int)std::lround(std::stod(row[p_i])) : 0;
        t.cnt = (cnt_i >= 0 && cnt_i < (int)row.size() && !row[cnt_i].empty()) ? (int)std::lround(std::stod(row[cnt_i])) : 0;
        t.id = (id_i >= 0 && id_i < (int)row.size()) ? row[id_i] : "";
        t.idx = (idx_i >= 0 && idx_i < (int)row.size()) ? row[idx_i] : "";

        tmp.push_back(std::move(t));
        max_r = std::max(max_r, tmp.back().r);
        max_c = std::max(max_c, tmp.back().c);
    }

    // If meta missing OR too small, expand to cover customers.
    if (gd.meta.height <= 0) gd.meta.height = max_r + 1;
    else gd.meta.height = std::max(gd.meta.height, max_r + 1);

    if (gd.meta.width <= 0) gd.meta.width = max_c + 1;
    else gd.meta.width = std::max(gd.meta.width, max_c + 1);

    // Allocate dense grid now (meta known).
    ensure_grid_alloc(gd);

    // Fill dense cells.
    for (const auto& t : tmp) {
        if (!gd.in_bounds(t.r, t.c)) continue;

        GridCell& cell = gd.at(t.r, t.c);
        if (!cell.occupied) {
            gd.occupied.push_back({t.r, t.c});
            cell.occupied = true;
        }
        cell.weight_sum = t.w;
        cell.volume_sum = t.v;
        cell.priority_max = t.p;
        cell.count = t.cnt;
        cell.customer_id = t.id;
        cell.customer_id_idx = t.idx;
    }

    if (gd.occupied.empty()) {
        throw std::runtime_error("[GridDataLoader] No occupied customer cells loaded from " + path);
    }
}

inline GridData load_grid_data(const std::string& grid_dir) {
    GridData gd;
    gd.meta = load_meta_if_any(grid_dir);
    load_depots_min(gd, grid_dir);
    load_customers_sparse_into_dense(gd, grid_dir);
    return gd;
}

} // namespace gridio
