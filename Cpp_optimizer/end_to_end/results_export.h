#pragma once

// =============================================================
// results_export.h - Export utilities for optimization results
//
// This file provides structured export functions for:
// - Assignment CSV (customer -> cluster/vehicle/depot mapping)
// - Routes CSV (trip details with distance, load, members)
// - Summary stats (trips, served, unserved, objective)
// =============================================================

#include "config.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <filesystem>
#include <chrono>
#include <sstream>

namespace fs = std::filesystem;

namespace gridopt {

// ============== Path utilities ==============

inline std::string join_path(const std::string& dir, const std::string& name) {
    if (dir.empty()) return name;
    char last = dir.back();
    if (last == '/' || last == '\\') return dir + name;
    return dir + "/" + name;
}

inline void ensure_dir(const std::string& dir) {
    if (dir.empty()) return;
    fs::create_directories(fs::path(dir));
}

inline std::string timestamp_suffix() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now;
#ifdef _WIN32
    localtime_s(&tm_now, &time_t_now);
#else
    localtime_r(&time_t_now, &tm_now);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm_now, "%Y%m%d_%H%M%S");
    return oss.str();
}

// ============== Export Configuration ==============

struct ExportConfig {
    std::string out_dir = "grid_out";
    std::string assignment_filename = "assignment.csv";
    std::string routes_filename = "routes.csv";
    std::string summary_filename = "summary.csv";
    bool add_timestamp = false;
    int precision = 6;
    
    std::string assignment_path() const {
        return join_path(out_dir, add_timestamp ? 
            (timestamp_suffix() + "_" + assignment_filename) : assignment_filename);
    }
    std::string routes_path() const {
        return join_path(out_dir, add_timestamp ? 
            (timestamp_suffix() + "_" + routes_filename) : routes_filename);
    }
    std::string summary_path() const {
        return join_path(out_dir, add_timestamp ? 
            (timestamp_suffix() + "_" + summary_filename) : summary_filename);
    }
};

// ============== Export Functions ==============

inline void export_assignments(const Instance& inst, const ExportConfig& cfg) {
    ensure_dir(cfg.out_dir);
    const std::string path = cfg.assignment_path();
    
    std::ofstream fout(path);
    if (!fout.is_open()) throw std::runtime_error("Cannot write " + path);
    
    fout << "customer_idx,customer_id,row,col,cell_linear,weight,volume,priority,cluster,vehicle_id,depot_id\n";
    
    for (const auto& r : inst.id_map.rows) {
        if (r.customer_idx < 0) continue;
        if (r.customer_id.empty()) continue;
        if (r.row < 0 || r.col < 0) continue;
        if (!inst.customers.in_bounds(r.row, r.col)) continue;

        double w = 0.0, v = 0.0, p = 0.0;
        if (inst.customers.occupied(r.row, r.col)) {
            w = (double)inst.customers.at(r.row, r.col, CUST_CH_W);
            v = (double)inst.customers.at(r.row, r.col, CUST_CH_VOL);
            p = (double)inst.customers.at(r.row, r.col, CUST_CH_PRI);
        }

        fout << r.customer_idx << "," << r.customer_id << "," << r.row << "," << r.col << "," << r.cell_linear
             << "," << std::fixed << std::setprecision(cfg.precision) << w
             << "," << std::fixed << std::setprecision(cfg.precision) << v
             << "," << std::fixed << std::setprecision(cfg.precision) << p
             << "," << r.cluster << "," << r.vehicle_id << "," << r.depot_id << "\n";
    }
    
    std::cout << "[Export] Wrote assignments: " << path << "\n";
}

inline void export_routes(const Instance& inst, const Solution& sol, const ExportConfig& cfg) {
    ensure_dir(cfg.out_dir);
    const std::string path = cfg.routes_path();
    
    std::ofstream fout(path);
    if (!fout.is_open()) throw std::runtime_error("Cannot write " + path);
    
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
            rep_customer_id_from_cell(inst, rt.center_cell_linear, center_cid);
            
            double trip_km = rt.distance_km;
            if (!center_cid.empty() && !depot_id.empty()) {
                double d = inst.get_dist_km(depot_id, center_cid);
                if (d < 1e8) trip_km = 2.0 * d;
            }
            
            std::string members;
            for (size_t i = 0; i < rt.member_cells_linear.size(); ++i) {
                members += std::to_string(rt.member_cells_linear[i]);
                if (i + 1 < rt.member_cells_linear.size()) members += " ";
            }
            
            fout << vid << "," << (t + 1) << "," << depot_id << "," << rt.center_cell_linear
                 << "," << cr << "," << cc << "," << center_cid
                 << "," << std::fixed << std::setprecision(cfg.precision) << trip_km
                 << "," << members
                 << "," << std::fixed << std::setprecision(cfg.precision) << rt.load_w
                 << "," << std::fixed << std::setprecision(cfg.precision) << rt.load_v
                 << "\n";
        }
    }
    
    std::cout << "[Export] Wrote routes: " << path << "\n";
}

inline void export_summary(const Instance& inst, const Solution& sol, const ExportConfig& cfg) {
    ensure_dir(cfg.out_dir);
    const std::string path = cfg.summary_path();
    
    std::ofstream fout(path);
    if (!fout.is_open()) throw std::runtime_error("Cannot write " + path);
    
    size_t trip_count = 0;
    size_t assigned = 0;
    double total_km = 0.0;
    
    for (const auto& kv : sol.routes) {
        for (const auto& rt : kv.second) {
            ++trip_count;
            assigned += rt.member_cells_linear.size();
            total_km += rt.distance_km;
        }
    }
    
    fout << "metric,value\n";
    fout << "trips," << trip_count << "\n";
    fout << "assigned_cells," << assigned << "\n";
    fout << "unassigned_cells," << sol.unassigned_cells.size() << "\n";
    fout << "serve_rate," << std::fixed << std::setprecision(4) 
         << (assigned > 0 ? (double)assigned / (assigned + sol.unassigned_cells.size()) : 0.0) << "\n";
    fout << "total_km," << std::fixed << std::setprecision(cfg.precision) << total_km << "\n";
    fout << "objective," << std::fixed << std::setprecision(cfg.precision) << sol.objective << "\n";
    fout << "fixed_cost," << std::fixed << std::setprecision(cfg.precision) << sol.fixed_cost << "\n";
    fout << "travel_cost," << std::fixed << std::setprecision(cfg.precision) << sol.travel_cost << "\n";
    fout << "penalty," << std::fixed << std::setprecision(cfg.precision) << sol.penalty << "\n";
    
    std::cout << "[Export] Wrote summary: " << path << "\n";
}

inline void export_all(const Instance& inst, const Solution& sol, const ExportConfig& cfg) {
    export_assignments(inst, cfg);
    export_routes(inst, sol, cfg);
    export_summary(inst, sol, cfg);
}

// ============== Print Summary ==============

inline void print_solution_summary(const Solution& sol, const std::string& label = "Solution") {
    size_t trip_count = 0;
    size_t assigned = 0;
    for (const auto& kv : sol.routes) {
        for (const auto& rt : kv.second) {
            ++trip_count;
            assigned += rt.member_cells_linear.size();
        }
    }
    
    double serve_rate = (assigned > 0) ? 
        (double)assigned / (assigned + sol.unassigned_cells.size()) * 100.0 : 0.0;
    
    std::cout << label << ": "
              << "trips=" << trip_count
              << " served=" << assigned
              << " unserved=" << sol.unassigned_cells.size()
              << " (" << std::fixed << std::setprecision(1) << serve_rate << "%)"
              << " obj=" << std::fixed << std::setprecision(2) << sol.objective
              << " (fixed=" << sol.fixed_cost
              << ", travel=" << sol.travel_cost
              << ", pen=" << sol.penalty << ")\n";
}

} // namespace gridopt
