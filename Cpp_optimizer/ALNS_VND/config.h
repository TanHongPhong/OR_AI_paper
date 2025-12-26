#pragma once

#define _USE_MATH_DEFINES
#include <string>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <iostream>
#include <cmath>
#include <algorithm>

// ===============================
//  MODEL: Cluster-trip last-mile
//  - Each vehicle k belongs to exactly one depot (K_d)
//  - Each trip/cluster t of vehicle k: depot -> center(customer) -> depot
//  - Total km of a vehicle is sum of all trips' round-trip distances
//  - No time windows / service time in objective
// ===============================

using std::string;
using std::vector;
using std::map;
using std::set;

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

inline int type_rank(VehicleType t) {
    // smaller = smaller vehicle
    switch (t) {
        case VehicleType::BIKE: return 0;
        case VehicleType::MOTORBIKE:
        case VehicleType::CARGO_TRIKE: return 1;
        case VehicleType::EV_VAN: return 2;
        case VehicleType::VAN: return 3;
        default: return 99;
    }
}

inline double spread_limit_km(VehicleType t) {
    // RELAXED: Doubled limits for better coverage of scattered customers
    //  Van, EV Van: < 10km (was 5km)
    //  Motorbike (+ Cargo Trike): < 4km (was 2km)
    //  Bike: < 2km (was 1km)
    switch (t) {
        case VehicleType::VAN:
        case VehicleType::EV_VAN: return 10.0;  // DOUBLED
        case VehicleType::MOTORBIKE:
        case VehicleType::CARGO_TRIKE: return 4.0;  // DOUBLED
        case VehicleType::BIKE: return 2.0;  // DOUBLED
        default: return 4.0;
    }
}

struct Customer {
    string id;
    double lat = 0.0;
    double lon = 0.0;

    // planar km coords for fast neighbor queries/visualize
    double x_km = 0.0;
    double y_km = 0.0;

    double weight = 0.0;
    double volume = 0.0;

    // preprocessing tags
    string territory_depot;          // which depot territory
    VehicleType class_req = VehicleType::UNKNOWN; // quartile class
    
    // Grid coordinates for 8-neighbor
    int row = 0;
    int col = 0;
};

struct Depot {
    string id;
    double lat = 0.0;
    double lon = 0.0;
    double x_km = 0.0;
    double y_km = 0.0;

    // storage capacity (required)
    double cap_weight_storage = 0.0;
    double cap_volume_storage = 0.0; // optional, 0 => ignore
};

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
    // We ignore time in objective; keep optional fields to leverage parameters.
    double travel_time_min = 0.0;
    string traffic_level;       // Low/Medium/High
    string restrictions;        // e.g., "No Heavy Trucks"
    double velocity = 0.0;
};

struct Instance {
    map<string, Customer> customers;
    map<string, Depot> depots;
    map<string, Vehicle> vehicles;
    vector<Road> roads;

    // matrix[from][to] = distance_km
    map<string, map<string, double>> dist_km;
    // optional edge attributes
    map<string, map<string, string>> edge_traffic;
    map<string, map<string, string>> edge_restrictions;

    // traffic multiplier mapping (tune later)
    double traffic_mult_low = 1.0;
    double traffic_mult_medium = 1.05;
    double traffic_mult_high = 1.12;

    // reference latitude for projection
    double ref_lat = 0.0;

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

    inline string edge_traffic_level(const string& from, const string& to) const {
        auto it1 = edge_traffic.find(from);
        if (it1 != edge_traffic.end()) {
            auto it2 = it1->second.find(to);
            if (it2 != it1->second.end()) return it2->second;
        }
        return "";
    }

    inline string edge_restriction_text(const string& from, const string& to) const {
        auto it1 = edge_restrictions.find(from);
        if (it1 != edge_restrictions.end()) {
            auto it2 = it1->second.find(to);
            if (it2 != it1->second.end()) return it2->second;
        }
        return "";
    }

    inline bool edge_allowed(const string& from, const string& to, VehicleType t) const {
        // If no restrictions -> allowed
        string r = edge_restriction_text(from, to);
        if (r.empty()) return true;
        string rr = r;
        for (auto &ch : rr) ch = (char)std::tolower((unsigned char)ch);
        // Simple mapping: "No Heavy Trucks" disallows Van/EV Van
        if (rr.find("no heavy") != string::npos || rr.find("heavy") != string::npos) {
            if (t == VehicleType::VAN || t == VehicleType::EV_VAN) return false;
        }
        return true;
    }

    inline double traffic_multiplier(const string& level) const {
        string s = level;
        for (auto &ch : s) ch = (char)std::tolower((unsigned char)ch);
        if (s.find("high") != string::npos) return traffic_mult_high;
        if (s.find("medium") != string::npos) return traffic_mult_medium;
        if (s.find("low") != string::npos) return traffic_mult_low;
        return 1.0;
    }

    // Fallback distance if arc missing: haversine on lat/lon (km)
    static double haversine_km(double lat1, double lon1, double lat2, double lon2) {
        const double R = 6371.0;
        auto deg2rad = [](double d) { return d * M_PI / 180.0; };
        double dLat = deg2rad(lat2 - lat1);
        double dLon = deg2rad(lon2 - lon1);
        double a = std::sin(dLat/2)*std::sin(dLat/2) +
                   std::cos(deg2rad(lat1))*std::cos(deg2rad(lat2)) *
                   std::sin(dLon/2)*std::sin(dLon/2);
        double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1-a));
        return R * c;
    }

    // Raw distance in km between nodes. Uses road matrix if available.
    // If missing, falls back to haversine if both nodes have lat/lon.
    double get_dist_km(const string& from, const string& to) const {
        if (from == to) return 0.0;
        auto it1 = dist_km.find(from);
        if (it1 != dist_km.end()) {
            auto it2 = it1->second.find(to);
            if (it2 != it1->second.end()) return it2->second;
        }

        // fallback using coordinates
        auto get_coord = [&](const string& id, double &lat, double &lon)->bool {
            auto itC = customers.find(id);
            if (itC != customers.end()) { lat = itC->second.lat; lon = itC->second.lon; return true; }
            auto itD = depots.find(id);
            if (itD != depots.end()) { lat = itD->second.lat; lon = itD->second.lon; return true; }
            return false;
        };
        double la1, lo1, la2, lo2;
        if (get_coord(from, la1, lo1) && get_coord(to, la2, lo2)) {
            return haversine_km(la1, lo1, la2, lo2);
        }
        return 1e9;
    }
};

struct Route {
    string vehicle_id;
    vector<string> stops; // [depot, centroid, depot]

    string cluster_id;
    string centroid_id;              // customer representative center
    vector<string> cluster_customers;

    double load_w = 0.0;
    double load_v = 0.0;
    double distance = 0.0; // round-trip distance (km)
};

struct Solution {
    // routes[vehicle_id] = list of trips/clusters
    map<string, vector<Route>> routes;
    double objective = 1e15;

    set<string> unassigned_customers;

    // breakdown
    double total_cost = 0.0;
    double fixed_cost = 0.0;
    double travel_cost = 0.0;
    double penalty = 0.0;

    double penalty_unserved = 0.0;
    double penalty_maxdist = 0.0;
    double penalty_veh_cap = 0.0;
    double penalty_depot_cap = 0.0;
    double penalty_cluster_spread = 0.0;
    double penalty_kd = 0.0;

    double calculate_objective(const Instance& inst);
};
