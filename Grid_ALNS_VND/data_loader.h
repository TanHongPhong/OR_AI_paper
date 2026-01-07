#pragma once

#define _USE_MATH_DEFINES
#include "config.h"
#include "utils.h"

#include <fstream>
#include <iostream>
#include <filesystem>
#include <cmath>

// Loader for 4 input tables:
// - customers__Sheet1.csv
// - depots__Sheet1.csv
// - vehicles__Sheet1.csv
// - roads__Sheet1.csv
// Also supports legacy filenames: customers.csv, depots.csv, vehicles.csv, roads.csv

class DataLoader {
public:
    static Instance load_instance(const std::string& data_dir) {
        Instance inst;
        load_depots(inst, data_dir);
        load_customers(inst, data_dir);
        compute_projection(inst);        // fill x_km/y_km for customers/depots
        load_vehicles(inst, data_dir);
        load_roads(inst, data_dir);
        inst.build_dist_matrix();
        return inst;
    }

private:
    static std::string pick_file(const std::string& dir, const std::initializer_list<std::string>& names) {
        for (const auto& n : names) {
            std::string p = dir + "/" + n;
            if (std::filesystem::exists(p)) return p;
        }
        return "";
    }

    static int col_index(const std::vector<std::string>& header, const std::string& name) {
        std::string target = lower(name);
        for (int i = 0; i < (int)header.size(); ++i) {
            std::string h = lower(header[i]);
            strip_utf8_bom(h);
            if (h == target) return i;
        }
        return -1;
    }

    static void load_customers(Instance& inst, const std::string& dir) {
        std::string path = pick_file(dir, {"customers__Sheet1.csv", "customers.csv"});
        if (path.empty()) throw std::runtime_error("Cannot find customers file");

        std::ifstream fin(path);
        if (!fin.is_open()) throw std::runtime_error("Cannot open " + path);

        std::string line;
        if (!std::getline(fin, line)) throw std::runtime_error("Empty customers file");
        strip_utf8_bom(line);
        auto header = split(line, ',');

        int id_i  = col_index(header, "customer_id");
        int lat_i = col_index(header, "latitude");
        int lon_i = col_index(header, "longitude");
        int w_i   = col_index(header, "order_weight");
        int v_i   = col_index(header, "order_volume");

        // allow fallback positions if header not found (old format)
        if (id_i < 0) id_i = 0;

        inst.customers.clear();
        while (std::getline(fin, line)) {
            if (line.empty()) continue;
            auto row = split(line, ',');
            if ((int)row.size() <= std::max({id_i, lat_i, lon_i, w_i, v_i})) continue;

            Customer c;
            c.id = row[id_i];
            if (lat_i >= 0) c.lat = std::stod(row[lat_i]);
            if (lon_i >= 0) c.lon = std::stod(row[lon_i]);
            if (w_i >= 0) c.weight = std::stod(row[w_i]);
            if (v_i >= 0) c.volume = std::stod(row[v_i]);
            inst.customers[c.id] = c;
        }

        if (inst.customers.empty()) {
            throw std::runtime_error("No customers loaded from " + path);
        }
    }

    static void load_depots(Instance& inst, const std::string& dir) {
        std::string path = pick_file(dir, {"depots__Sheet1.csv", "depots.csv"});
        if (path.empty()) throw std::runtime_error("Cannot find depots file");

        std::ifstream fin(path);
        if (!fin.is_open()) throw std::runtime_error("Cannot open " + path);

        std::string line;
        if (!std::getline(fin, line)) throw std::runtime_error("Empty depots file");
        strip_utf8_bom(line);
        auto header = split(line, ',');

        int id_i  = col_index(header, "depot_id"); if (id_i < 0) id_i = 0;
        int lat_i = col_index(header, "latitude");
        int lon_i = col_index(header, "longitude");
        int cap_i = col_index(header, "capacity_storage");

        inst.depots.clear();
        while (std::getline(fin, line)) {
            if (line.empty()) continue;
            auto row = split(line, ',');
            if ((int)row.size() <= std::max({id_i, lat_i, lon_i, cap_i})) continue;

            Depot d;
            d.id = row[id_i];
            if (lat_i >= 0) d.lat = std::stod(row[lat_i]);
            if (lon_i >= 0) d.lon = std::stod(row[lon_i]);
            if (cap_i >= 0) d.cap_weight_storage = std::stod(row[cap_i]);
            inst.depots[d.id] = d;
        }

        if (inst.depots.empty()) throw std::runtime_error("No depots loaded from " + path);
    }

    static void load_vehicles(Instance& inst, const std::string& dir) {
        std::string path = pick_file(dir, {"vehicles__Sheet1.csv", "vehicles.csv"});
        if (path.empty()) throw std::runtime_error("Cannot find vehicles file");

        std::ifstream fin(path);
        if (!fin.is_open()) throw std::runtime_error("Cannot open " + path);

        std::string line;
        if (!std::getline(fin, line)) throw std::runtime_error("Empty vehicles file");
        strip_utf8_bom(line);
        auto header = split(line, ',');

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
            auto row = split(line, ',');
            if ((int)row.size() <= std::max({id_i, type_i, w_i, v_i, fc_i, vc_i, md_i, sd_i, ed_i})) continue;

            Vehicle v;
            v.id = row[id_i];
            if (type_i >= 0) v.vehicle_type = row[type_i];
            v.type = parse_vehicle_type(v.vehicle_type);
            if (w_i >= 0) v.cap_weight = std::stod(row[w_i]);
            if (v_i >= 0) v.cap_volume = std::stod(row[v_i]);
            if (fc_i >= 0) v.fixed_cost = std::stod(row[fc_i]);
            if (vc_i >= 0) v.var_cost = std::stod(row[vc_i]);
            if (md_i >= 0) v.max_dist = std::stod(row[md_i]);
            if (sd_i >= 0) v.start_depot = row[sd_i];
            if (ed_i >= 0) v.end_depot = row[ed_i];
            if (v.end_depot.empty()) v.end_depot = v.start_depot;
            // clean trailing spaces
            v.start_depot = trim(v.start_depot);
            v.end_depot = trim(v.end_depot);

            inst.vehicles[v.id] = v;
        }

        if (inst.vehicles.empty()) throw std::runtime_error("No vehicles loaded from " + path);
    }

    static void load_roads(Instance& inst, const std::string& dir) {
        std::string path = pick_file(dir, {"roads__Sheet1.csv", "roads.csv"});
        if (path.empty()) {
            // roads optional; distances will fall back to haversine
            std::cout << "[DataLoader] No roads file found, using haversine\n";
            return;
        }

        std::cout << "[DataLoader] Loading roads from: " << path << "\n";
        std::ifstream fin(path);
        if (!fin.is_open()) throw std::runtime_error("Cannot open " + path);

        std::string line;
        if (!std::getline(fin, line)) return;
        strip_utf8_bom(line);
        auto header = split(line, ',');

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
            auto row = split(line, ',');
            if ((int)row.size() <= std::max({from_i, to_i, dist_i})) continue;

            Road r;
            r.from = row[from_i];
            r.to = row[to_i];
            if (dist_i >= 0) r.distance_km = std::stod(row[dist_i]);
            if (time_i >= 0 && time_i < (int)row.size()) r.travel_time_min = std::stod(row[time_i]);
            if (traf_i >= 0 && traf_i < (int)row.size()) r.traffic_level = row[traf_i];
            if (rest_i >= 0 && rest_i < (int)row.size()) r.restrictions = row[rest_i];
            if (vel_i >= 0 && vel_i < (int)row.size()) r.velocity = std::stod(row[vel_i]);
            inst.roads.push_back(r);
            count++;
        }
        std::cout << "[DataLoader] Loaded " << count << " road segments\n";
    }

    static void compute_projection(Instance& inst) {
        // equirectangular projection to km coordinates for neighbor queries + visualization
        // x = lon*cos(ref_lat)*111.32 ; y = lat*110.57 (approx)
        double sum_lat = 0.0;
        int cnt = 0;
        for (const auto& kv : inst.customers) {
            sum_lat += kv.second.lat;
            cnt++;
        }
        for (const auto& kv : inst.depots) {
            sum_lat += kv.second.lat;
            cnt++;
        }
        inst.ref_lat = (cnt > 0) ? (sum_lat / cnt) : 0.0;
        double cos_ref = std::cos(inst.ref_lat * M_PI / 180.0);
        auto lon2km = [&](double lon) { return lon * 111.32 * cos_ref; };
        auto lat2km = [&](double lat) { return lat * 110.57; };

        // Find min lat/lon for grid conversion
        double min_lat = 1e18, min_lon = 1e18;
        for (const auto& kv : inst.customers) {
            min_lat = std::min(min_lat, kv.second.lat);
            min_lon = std::min(min_lon, kv.second.lon);
        }
        for (const auto& kv : inst.depots) {
            min_lat = std::min(min_lat, kv.second.lat);
            min_lon = std::min(min_lon, kv.second.lon);
        }

        // Grid cell size in km (100m cells)
        const double GRID_CELL_KM = 0.1;

        for (auto& kv : inst.customers) {
            kv.second.x_km = lon2km(kv.second.lon);
            kv.second.y_km = lat2km(kv.second.lat);
            
            // Convert to grid coordinates
            double rel_y_km = lat2km(kv.second.lat) - lat2km(min_lat);
            double rel_x_km = lon2km(kv.second.lon) - lon2km(min_lon);
            kv.second.row = (int)(rel_y_km / GRID_CELL_KM);
            kv.second.col = (int)(rel_x_km / GRID_CELL_KM);
        }
        for (auto& kv : inst.depots) {
            kv.second.x_km = lon2km(kv.second.lon);
            kv.second.y_km = lat2km(kv.second.lat);
        }
    }
};
