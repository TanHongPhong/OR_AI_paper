#pragma once

#include "config.h"
#include "utils.h"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <random>

// Export solution summaries + simple visualization (PPM).
// Designed to be reusable for both initial solution and optimizer output.

namespace export_detail {

inline std::string join_ids(const std::vector<std::string>& v, const std::string& sep = ";") {
    std::ostringstream oss;
    for (size_t i = 0; i < v.size(); ++i) {
        if (i) oss << sep;
        oss << v[i];
    }
    return oss.str();
}

struct RGB { unsigned char r,g,b; };

inline RGB rand_color(std::mt19937& rng) {
    std::uniform_int_distribution<int> dist(60, 235);
    return {(unsigned char)dist(rng), (unsigned char)dist(rng), (unsigned char)dist(rng)};
}

inline void write_ppm(const std::string& path, int W, int H, const std::vector<RGB>& pix) {
    std::ofstream f(path, std::ios::binary);
    if (!f.is_open()) return;
    f << "P6\n" << W << " " << H << "\n255\n";
    f.write(reinterpret_cast<const char*>(pix.data()), (std::streamsize)pix.size()*3);
}

inline void draw_dot(std::vector<RGB>& pix, int W, int H, int x, int y, int radius, RGB c) {
    for (int dy=-radius; dy<=radius; ++dy) {
        for (int dx=-radius; dx<=radius; ++dx) {
            if (dx*dx + dy*dy > radius*radius) continue;
            int xx = x + dx;
            int yy = y + dy;
            if (xx < 0 || xx >= W || yy < 0 || yy >= H) continue;
            pix[yy*W + xx] = c;
        }
    }
}

} // namespace export_detail

inline void export_solution_summaries(const Instance& inst,
                                      const Solution& sol,
                                      const std::string& out_dir,
                                      const std::string& prefix = "solution")
{
    using namespace export_detail;

    // ===== customer assignments =====
    {
        std::ofstream f(out_dir + "/" + prefix + "_customer_assignments.csv");
        f << "Customer_ID,Depot_ID,Vehicle_ID,Trip_Index,Center_ID\n";
        
        // Track assigned customers to avoid duplicates
        std::unordered_set<std::string> assigned_customers;
        
        for (const auto& kv : sol.routes) {
            const std::string& vid = kv.first;
            auto itV = inst.vehicles.find(vid);
            if (itV == inst.vehicles.end()) continue;
            const std::string did = itV->second.start_depot;
            for (int t = 0; t < (int)kv.second.size(); ++t) {
                const auto& r = kv.second[t];
                for (const auto& cid : r.cluster_customers) {
                    f << cid << "," << did << "," << vid << "," << (t+1) << "," << r.centroid_id << "\n";
                    assigned_customers.insert(cid);  // Track this customer
                }
            }
        }
        
        // Only export customers NOT in assigned set
        for (const auto& cid : sol.unassigned_customers) {
            if (assigned_customers.count(cid) == 0) {  // Not already exported
                f << cid << ",,,,-\n";
            }
        }
    }

    // ===== routes summary =====
    {
        std::ofstream f(out_dir + "/" + prefix + "_routes_summary.csv");
        f << "Depot_ID,Vehicle_ID,Trip_Index,Vehicle_Type,Center_ID,Num_Customers,Load_Weight,Load_Volume,Trip_Distance_km,Spread_km,Customer_List\n";
        for (const auto& kv : sol.routes) {
            const std::string& vid = kv.first;
            auto itV = inst.vehicles.find(vid);
            if (itV == inst.vehicles.end()) continue;
            const auto& veh = itV->second;
            const std::string did = veh.start_depot;

            for (int t = 0; t < (int)kv.second.size(); ++t) {
                const auto& r = kv.second[t];
                double w=0.0,v=0.0;
                for (const auto& cid : r.cluster_customers) {
                    const auto& c = inst.customers.at(cid);
                    w += c.weight; v += c.volume;
                }
                double spread = 0.0;
                if (!r.centroid_id.empty()) {
                    const auto& cen = inst.customers.at(r.centroid_id);
                    for (const auto& cid : r.cluster_customers) {
                        const auto& c = inst.customers.at(cid);
                        spread = std::max(spread, Instance::haversine_km(cen.lat, cen.lon, c.lat, c.lon));
                    }
                }
                f << did << "," << vid << "," << (t+1) << "," << veh.vehicle_type << "," << r.centroid_id << ","
                  << r.cluster_customers.size() << "," << std::fixed << std::setprecision(6)
                  << w << "," << v << "," << r.distance << "," << spread << "," << "\"" << join_ids(r.cluster_customers) << "\"\n";
            }
        }
    }

    // ===== vehicles summary =====
    {
        std::ofstream f(out_dir + "/" + prefix + "_vehicles_summary.csv");
        f << "Depot_ID,Vehicle_ID,Vehicle_Type,Num_Trips,Total_Distance_km,Max_Distance_km,Fixed_Cost,Travel_Cost\n";
        for (const auto& kv : inst.vehicles) {
            const auto& veh = kv.second;
            double km_sum = 0.0;
            int trips = 0;
            auto itR = sol.routes.find(veh.id);
            if (itR != sol.routes.end()) {
                trips = (int)itR->second.size();
                for (const auto& r : itR->second) km_sum += r.distance;
            }
            double fc = trips * veh.fixed_cost;
            double tc = km_sum * veh.var_cost;
            f << veh.start_depot << "," << veh.id << "," << veh.vehicle_type << "," << trips << ","
              << std::fixed << std::setprecision(6)
              << km_sum << "," << veh.max_dist << "," << fc << "," << tc << "\n";
        }
    }

    // ===== objective summary =====
    {
        std::ofstream f(out_dir + "/" + prefix + "_objective_summary.csv");
        f << "Objective,Total_Cost,Fixed_Cost,Travel_Cost,Penalty_Unserved,Penalty_MaxDist,Penalty_VehCap,Penalty_DepotCap,Penalty_Spread,Penalty_Kd,Num_Unserved\n";
        f << std::fixed << std::setprecision(6)
          << sol.objective << "," << sol.total_cost << "," << sol.fixed_cost << "," << sol.travel_cost << ","
          << sol.penalty_unserved << "," << sol.penalty_maxdist << "," << sol.penalty_veh_cap << "," << sol.penalty_depot_cap << ","
          << sol.penalty_cluster_spread << "," << sol.penalty_kd << "," << sol.unassigned_customers.size() << "\n";
    }

    // ===== visualization (PPM) =====
    {
        const int W = 1080, H = 1080;
        std::vector<RGB> pix(W*H, RGB{255,255,255});

        // bounding box from customers & depots
        double minx=1e18, maxx=-1e18, miny=1e18, maxy=-1e18;
        for (const auto& kv : inst.customers) {
            minx = std::min(minx, kv.second.x_km);
            maxx = std::max(maxx, kv.second.x_km);
            miny = std::min(miny, kv.second.y_km);
            maxy = std::max(maxy, kv.second.y_km);
        }
        for (const auto& kv : inst.depots) {
            minx = std::min(minx, kv.second.x_km);
            maxx = std::max(maxx, kv.second.x_km);
            miny = std::min(miny, kv.second.y_km);
            maxy = std::max(maxy, kv.second.y_km);
        }
        double dx = std::max(1e-9, maxx-minx);
        double dy = std::max(1e-9, maxy-miny);

        auto map_xy = [&](double x, double y)->std::pair<int,int> {
            int px = (int)std::round((x - minx) / dx * (W-1));
            int py = (int)std::round((maxy - y) / dy * (H-1));
            return {px, py};
        };

        // assign a color per (vehicle,trip)
        std::mt19937 rng(123);
        std::unordered_map<std::string, RGB> cluster_color;
        for (const auto& kv : sol.routes) {
            for (int t=0; t<(int)kv.second.size(); ++t) {
                std::string key = kv.first + "#" + std::to_string(t+1);
                cluster_color[key] = rand_color(rng);
            }
        }

        // map customer -> (vehicle,trip)
        std::unordered_map<std::string, std::string> cust_cluster;
        for (const auto& kv : sol.routes) {
            for (int t=0; t<(int)kv.second.size(); ++t) {
                std::string key = kv.first + "#" + std::to_string(t+1);
                for (const auto& cid : kv.second[t].cluster_customers) cust_cluster[cid] = key;
            }
        }

        // draw customers
        for (const auto& kv : inst.customers) {
            const auto& c = kv.second;
            auto [px,py] = map_xy(c.x_km, c.y_km);
            auto it = cust_cluster.find(c.id);
            if (it == cust_cluster.end()) {
                draw_dot(pix, W, H, px, py, 2, RGB{0,0,0});
            } else {
                draw_dot(pix, W, H, px, py, 2, cluster_color[it->second]);
            }
        }

        // draw depots
        for (const auto& kv : inst.depots) {
            const auto& d = kv.second;
            auto [px,py] = map_xy(d.x_km, d.y_km);
            draw_dot(pix, W, H, px, py, 6, RGB{220,0,0});
        }

        write_ppm(out_dir + "/" + prefix + "_vis.ppm", W, H, pix);
    }
}
