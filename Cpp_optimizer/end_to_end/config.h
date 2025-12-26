#pragma once

// ==========================================
//  Cluster-trip last-mile (GRID-TENSOR edition)
//
//  RAM data layout (as requested):
//   1) CustomerTensor: dense tensor [H, W, 4]
//        ch0 = weight
//        ch1 = volume
//        ch2 = priority
//        ch3 = idx (stored as float, but represents integer)
//   2) Depot / Vehicle / Road: table structures loaded from the original CSVs
//   3) CustomerIdMap: mapping table (like customer_id_map.csv) extended with
//        cluster / vehicle / depot fields during init/optimizer.
//
//  Notes:
//   - CustomerTensor is a *true numeric tensor*.
//   - IDs (strings) live in CustomerIdMap / Depot / Vehicle / Road, not inside the tensor.
// ==========================================

#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <set>
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace gridopt {

using std::string;
using std::vector;
using std::map;
using std::set;

// -------- Customer tensor channels --------
static constexpr int CUST_CH_W   = 0;
static constexpr int CUST_CH_VOL = 1;
static constexpr int CUST_CH_PRI = 2;
static constexpr int CUST_CH_IDX = 3;
static constexpr int CUST_CH     = 4;

// ==========================================
// CustomerTensor: dense [H, W, 4] float tensor
// ==========================================
struct CustomerTensor {
    int H = 0;
    int W = 0;
    // Layout: ((r*W + c)*4 + k)
    vector<float> data;

    CustomerTensor() = default;
    CustomerTensor(int h, int w) { resize(h, w); }

    void resize(int h, int w) {
        if (h < 0 || w < 0) throw std::runtime_error("CustomerTensor::resize: negative dims");
        H = h; W = w;
        data.assign((size_t)H * (size_t)W * (size_t)CUST_CH, 0.0f);
        // idx channel default = -1 to mark empty
        for (int r = 0; r < H; ++r) {
            for (int c = 0; c < W; ++c) {
                at(r, c, CUST_CH_IDX) = -1.0f;
            }
        }
    }

    inline bool in_bounds(int r, int c) const {
        return (r >= 0 && r < H && c >= 0 && c < W);
    }

    inline size_t off(int r, int c, int k) const {
        return ((size_t)r * (size_t)W + (size_t)c) * (size_t)CUST_CH + (size_t)k;
    }

    inline float& at(int r, int c, int k) {
        return data[off(r, c, k)];
    }

    inline const float& at(int r, int c, int k) const {
        return data[off(r, c, k)];
    }

    inline bool occupied(int r, int c) const {
        return at(r, c, CUST_CH_IDX) >= 0.0f;
    }

    inline int idx_int(int r, int c) const {
        // idx stored as float but represents integer
        return (int)at(r, c, CUST_CH_IDX);
    }

    inline int linear(int r, int c) const {
        return r * W + c;
    }

    inline void decode_linear(int linear_idx, int &r, int &c) const {
        r = linear_idx / W;
        c = linear_idx % W;
    }
};

// ==========================================
// Depots / Vehicles / Roads (tables)
// ==========================================
struct Depot {
    string id;
    int row = 0;
    int col = 0;
    double cap_weight_storage = 0.0;
};

enum class VehicleType { VAN, EV_VAN, MOTORBIKE, BIKE, CARGO_TRIKE, UNKNOWN };

inline VehicleType parse_vehicle_type(string s) {
    for (auto &ch : s) ch = (char)std::tolower((unsigned char)ch);
    if (s.find("ev") != string::npos && s.find("van") != string::npos) return VehicleType::EV_VAN;
    if (s == "van" || s.find("van") != string::npos) return VehicleType::VAN;
    if (s.find("motor") != string::npos) return VehicleType::MOTORBIKE;
    if (s.find("bike") != string::npos) return VehicleType::BIKE;
    if (s.find("trike") != string::npos) return VehicleType::CARGO_TRIKE;
    return VehicleType::UNKNOWN;
}

struct Vehicle {
    string id;
    VehicleType type = VehicleType::UNKNOWN;
    string vehicle_type;   // raw string

    double cap_weight = 0.0;
    double cap_volume = 0.0;
    double fixed_cost = 0.0;
    double var_cost = 0.0;
    double max_dist = 0.0; // TOTAL km budget per day

    string start_depot;
    string end_depot; // should equal start_depot
};

struct Road {
    string from;
    string to;
    double distance_km = 0.0;

    // optional fields (not always used by objective)
    double travel_time_min = 0.0;
    string traffic_level;
    string restrictions;
    double velocity = 0.0;
};

// ==========================================
// CustomerIdMap (extended during init/optimizer)
// ==========================================
struct CustomerIdMapRow {
    int customer_idx = -1;     // key in customer_id_map.csv
    string customer_id;        // original customer id

    // Where this customer landed on the grid
    int row = -1;
    int col = -1;
    int cell_linear = -1;      // row*W + col

    // Filled/updated by init/optimizer
    int cluster = -1;
    string vehicle_id;
    string depot_id;
};

struct CustomerIdMap {
    // For convenience we keep a vector indexed by customer_idx when possible.
    // If idx is sparse, we still store rows[idx] (after ensure_size).
    vector<CustomerIdMapRow> rows;

    // lookups
    std::unordered_map<string, int> id_to_idx;
    std::unordered_map<int, vector<int>> cell_to_customer_indices; // cell_linear -> list of customer_idx

    inline void ensure_size(int n) {
        if ((int)rows.size() < n) rows.resize((size_t)n);
    }

    inline bool has_idx(int idx) const {
        return (idx >= 0 && idx < (int)rows.size() && rows[(size_t)idx].customer_idx == idx);
    }
};

// ==========================================
// Instance: all data needed by init + optimizer
// ==========================================
struct Instance {
    // Grid customers
    CustomerTensor customers;

    // Tables
    vector<Depot> depots;
    map<string, Vehicle> vehicles;
    vector<Road> roads;

    // Roads distance lookup
    map<string, map<string, double>> dist_km;
    map<string, map<string, string>> edge_traffic;
    map<string, map<string, string>> edge_restrictions;

    // customer idx mapping
    CustomerIdMap id_map;

    // Build dist/attribute matrices from roads
    void build_dist_matrix() {
        dist_km.clear();
        edge_traffic.clear();
        edge_restrictions.clear();
        for (const auto& r : roads) {
            dist_km[r.from][r.to] = r.distance_km;
            edge_traffic[r.from][r.to] = r.traffic_level;
            edge_restrictions[r.from][r.to] = r.restrictions;
        }
    }

    // Distance between 2 nodes using the road table.
    // (No fallback: if missing returns big number.)
    double get_dist_km(const string& from, const string& to) const {
        if (from == to) return 0.0;
        auto it1 = dist_km.find(from);
        if (it1 != dist_km.end()) {
            auto it2 = it1->second.find(to);
            if (it2 != it1->second.end()) return it2->second;
        }
        return 1e9;
    }
};

// ==========================================
// Optional: route/solution schema (kept minimal)
// ==========================================
struct Route {
    string vehicle_id;
    int center_cell_linear = -1;      // center on grid
    vector<int> member_cells_linear;  // members on grid

    double load_w = 0.0;
    double load_v = 0.0;

    // objective may use road distance depot->center (computed elsewhere)
    double distance_km = 0.0;
};

struct Solution {
    map<string, vector<Route>> routes;  // routes[vehicle_id]
    set<int> unassigned_cells;          // cell_linear ids not assigned

    double objective = 1e18;
    double fixed_cost = 0.0;
    double travel_cost = 0.0;
    double penalty = 0.0;
};

} // namespace gridopt
