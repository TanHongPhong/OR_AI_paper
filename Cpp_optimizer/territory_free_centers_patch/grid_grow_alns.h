#pragma once
/*
grid_grow_alns.h  (header-only, C++17)
========================================================
"Grid-grow ALNS" skeleton for your cluster-trip model.
- Customers live on a sparse grid (cell -> customers).
- Repair is flood-fill regrow from cluster frontier (like your init grow).
- Destroy focuses on boundary / connected cell regions / center shift / neighbor swap.
- All moves are transactional (remove -> try -> rollback) to avoid "kick customer".

How to integrate (minimal):
1) Provide customers/depots/vehicles with x_km,y_km, demand.
2) Build State from an initial assignment (your init).
3) Call gg_alns::GridGrowALNS().run(state, iters).
4) Export/visualize using your existing exporter.

You can keep your current objective/constraints by wiring the Hooks below.
========================================================
*/

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include <iostream>
#include <iomanip>

namespace gg_alns {

// ===============================
// Basic geometry / keys
// ===============================
struct Pt {
  double x = 0.0;
  double y = 0.0;
};

inline double dist_km(const Pt& a, const Pt& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx*dx + dy*dy);
}

using CellKey = std::int64_t;

// Pack row/col (signed 32-bit) into one 64-bit key.
inline CellKey pack_cell(int row, int col) {
  return ( (std::int64_t)row << 32 ) ^ (std::uint32_t)col;
}
inline int cell_row(CellKey k) { return (int)(k >> 32); }
inline int cell_col(CellKey k) { return (int)(k & 0xFFFFFFFFu); }

inline void nbr8_cells(CellKey c, CellKey out[8]) {
  const int r = cell_row(c), cl = cell_col(c);
  int idx = 0;
  for (int dr = -1; dr <= 1; ++dr) {
    for (int dc = -1; dc <= 1; ++dc) {
      if (dr == 0 && dc == 0) continue;
      out[idx++] = pack_cell(r + dr, cl + dc);
    }
  }
}

// ===============================
// Problem entities (lightweight)
// ===============================
struct Customer {
  int id = -1;          // external ID
  Pt  p;                // km coords
  double demand = 0.0;  // weight
  double vol = 0.0;     // volume
  int class_req = 0;    // optional
};

struct Depot {
  int id = -1;
  Pt  p;
  double cap = std::numeric_limits<double>::infinity(); // optional depot cap
};

struct Vehicle {
  int id = -1;
  int depot_id = -1;         // fixed depot
  double cap = 0.0;          // weight capacity
  double cap_vol = 0.0;      // volume capacity
  double max_km_total = std::numeric_limits<double>::infinity(); // total km across its clusters
  int class_cap = 0;         // optional class compatibility flag
};

// ===============================
// Hooks for your constraints
// ===============================
struct Hooks {
  // Check if a customer can be served by this vehicle (class, depot assignment rules, etc.)
  std::function<bool(const Vehicle&, const Customer&)> vehicle_customer_ok =
      [](const Vehicle&, const Customer&) { return true; };

  // Check if a chosen center is allowed (edge allowed, etc.)
  std::function<bool(int depot_id, int center_ui, double trip_km)> center_ok =
      [](int /*depot_id*/, int /*center_ui*/, double /*trip_km*/) { return true; };

  // Optional: depot total load cap (sum over clusters at depot <= cap).
  // Return penalty (>0) if violated, 0.0 if ok.
  std::function<double(int depot_id, double new_depot_load)> depot_load_penalty =
      [](int, double) { return 0.0; };

  // Optional: vehicle total km cap (sum of trip_km of clusters of vehicle <= max_km_total).
  // Return penalty (>0) if violated, 0.0 if ok.
  std::function<double(int vehicle_id, double new_vehicle_km)> vehicle_km_penalty =
      [](int, double) { return 0.0; };
};

// ===============================
// Sparse grid index
// ===============================
struct GridIndex {
  double cell_size_km = 0.02; // default 20m = 0.02km
  std::vector<CellKey> cust_cell; // ui -> cell
  std::unordered_map<CellKey, std::vector<int>> cell2cust; // cell -> ui list

  void build(const std::vector<Customer>& custs, double cell_size_km_) {
    cell_size_km = cell_size_km_;
    cust_cell.assign(custs.size(), 0);
    cell2cust.clear();
    cell2cust.reserve(custs.size() * 2);

    for (int ui = 0; ui < (int)custs.size(); ++ui) {
      const int r = (int)std::floor(custs[ui].p.y / cell_size_km);
      const int c = (int)std::floor(custs[ui].p.x / cell_size_km);
      const CellKey key = pack_cell(r, c);
      cust_cell[ui] = key;
      cell2cust[key].push_back(ui);
    }
  }
};

// ===============================
// Cluster / State
// ===============================
struct Cluster {
  int id = -1;
  int vehicle_id = -1;
  int depot_id = -1;

  std::vector<int> members; // customer indices (ui)

  // stats
  double sumx = 0.0, sumy = 0.0;
  double load = 0.0;
  double load_vol = 0.0;

  // representative (center) used by objective: trip = 2*dist(depot, center)
  int center_ui = -1;
  double trip_km = 0.0;

  // cached boundary
  bool boundary_dirty = true;
  std::vector<int> boundary;
};

struct State {
  const std::vector<Customer>* cust = nullptr;
  const std::vector<Depot>* depots = nullptr;
  const std::vector<Vehicle>* veh = nullptr;

  GridIndex grid;
  Hooks hooks;

  std::vector<Cluster> clusters;   // each cluster is a trip assigned to a vehicle
  std::vector<int> cust_cluster;   // ui -> cluster_id or -1
  std::vector<char> in_unassigned; // ui -> bool
  std::vector<int> unassigned;     // list of ui (lazy delete)

  // depot/vehicle totals
  std::vector<double> depot_total_load;
  std::vector<double> vehicle_total_km;

  // rng
  std::mt19937 rng { 1234567u };

  // objective: sum trip_km + penalty
  double total_cost = 0.0;
  double penalty_unserved = 100000000.0; // 1e8
  double penalty_cap = 1000.0;           // 1e3 per unit overflow (much lighter than unserved)
  int unassigned_cnt = 0;

  // center cohesion config (set from ALNS config)
  double center_adj_radius_km = 0.6;
  double w_center_adj = 0.0;   // 0 disables
  double w_center_cc  = 0.0;   // 0 disables

  // cached per-depot center cohesion penalty (sum already included in total_cost)
  std::vector<double> depot_center_penalty;

  // helper: cluster representative point (center customer). empty clusters have no rep.
  inline bool cluster_has_center(const Cluster& C) const { return C.center_ui >= 0; }
  inline Pt cluster_center_pt(const Cluster& C) const {
    if (C.center_ui >= 0) return customer(C.center_ui).p;
    return depot(C.depot_id).p;
  }

  double compute_depot_center_penalty(int depot_id) const {
    if (w_center_adj <= 0.0 && w_center_cc <= 0.0) return 0.0;

    std::vector<Pt> centers;
    centers.reserve(16);
    for (const auto& C : clusters) {
      if (C.depot_id != depot_id) continue;
      if (C.center_ui < 0) continue;
      centers.push_back(customer(C.center_ui).p);
    }
    const int n = (int)centers.size();
    if (n <= 1) return 0.0;

    double pen = 0.0;

    // isolated-center penalty: each center must be within R of another center
    if (w_center_adj > 0.0) {
      for (int i=0;i<n;i++) {
        double best = std::numeric_limits<double>::infinity();
        for (int j=0;j<n;j++) if (i!=j) {
          best = std::min(best, dist_km(centers[i], centers[j]));
        }
        if (best > center_adj_radius_km) {
          pen += w_center_adj * (best - center_adj_radius_km);
        }
      }
    }

    // connected-components penalty (within R edges)
    if (w_center_cc > 0.0) {
      std::vector<int> parent(n);
      for(int i=0;i<n;i++) parent[i]=i;
      auto findp=[&](int a){ while(parent[a]!=a){ parent[a]=parent[parent[a]]; a=parent[a]; } return a; };
      auto unite=[&](int a,int b){
        a=findp(a); b=findp(b);
        if(a!=b) parent[b]=a;
      };
      for(int i=0;i<n;i++){
        for(int j=i+1;j<n;j++){
          if (dist_km(centers[i], centers[j]) <= center_adj_radius_km) unite(i,j);
        }
      }
      std::unordered_set<int> comps;
      comps.reserve(n*2);
      for(int i=0;i<n;i++) comps.insert(findp(i));
      const int cc = (int)comps.size();
      if (cc > 1) pen += w_center_cc * (cc - 1);
    }

    return pen;
  }


  void init_refs(const std::vector<Customer>& C,
                 const std::vector<Depot>& D,
                 const std::vector<Vehicle>& V,
                 double cell_size_km) {
    cust = &C; depots = &D; veh = &V;
    grid.build(C, cell_size_km);

    cust_cluster.assign(C.size(), -1);
    in_unassigned.assign(C.size(), 1);
    unassigned.clear();
    unassigned.reserve(C.size());
    unassigned_cnt = 0;
    for (int i=0;i<(int)C.size();++i) {
      unassigned.push_back(i);
      unassigned_cnt++;
    }

    depot_total_load.assign(D.size(), 0.0);
    vehicle_total_km.assign(V.size(), 0.0);
    depot_center_penalty.assign(D.size(), 0.0);

    total_cost = (double)unassigned_cnt * penalty_unserved;
    // Note: Hooks might have initial penalties if Init Solution is infeasible, 
    // but we assume Init is roughly checked or penalties will be added incrementally.
    // For exactness, one should loop and sum penalties, but starting from 0 delta is fine as long as deltas are correct.
  }

  inline const Customer& customer(int ui) const { return (*cust)[ui]; }
  inline const Depot& depot(int did) const { return (*depots)[did]; }
  inline const Vehicle& vehicle(int vid) const { return (*veh)[vid]; }

  inline double depot_customer_km(int depot_id, int ui) const {
    return dist_km(depot(depot_id).p, customer(ui).p);
  }

  int rand_int(int lo, int hi) {
    std::uniform_int_distribution<int> dist(lo, hi);
    return dist(rng);
  }
  double rand01() {
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(rng);
  }

  // -------------------------------
  // Center selection (scan-all, robust)
  // -------------------------------
  bool recompute_center(Cluster& C) {
    if (C.members.empty()) {
      C.center_ui = -1;
      C.trip_km = 0.0;
      return true;
    }

    const double cx = C.sumx / (double)C.members.size();
    const double cy = C.sumy / (double)C.members.size();
    Pt centroid{cx, cy};

    int best_ui = -1;
    double best_d = std::numeric_limits<double>::infinity();
    double best_dd = std::numeric_limits<double>::infinity();
    int best_id = std::numeric_limits<int>::max();

    for (int ui : C.members) {
      const double d = dist_km(customer(ui).p, centroid);
      const double dd = depot_customer_km(C.depot_id, ui);
      const int ext_id = customer(ui).id;
      if (d < best_d - 1e-12 ||
          (std::abs(d - best_d) <= 1e-12 && (dd < best_dd - 1e-12 ||
           (std::abs(dd - best_dd) <= 1e-12 && ext_id < best_id)))) {
        best_d = d; best_dd = dd; best_id = ext_id; best_ui = ui;
      }
    }

    const double trip = 2.0 * depot_customer_km(C.depot_id, best_ui);
    if (!hooks.center_ok(C.depot_id, best_ui, trip)) return false;

    C.center_ui = best_ui;
    C.trip_km = trip;
    return true;
  }

  void apply_trip_change(int vid, double old_trip, double new_trip) {
    vehicle_total_km[vid] += (new_trip - old_trip);
    total_cost += (new_trip - old_trip);
  }

  // -------------------------------
  // Assignment primitives
  // -------------------------------
  void remove_from_cluster(int ui, int cid) {
    Cluster& C = clusters[cid];
    const Vehicle& V = vehicle(C.vehicle_id);

    // Calc "Before" penalty
    double p_old = 0.0;
    if (C.load > V.cap) p_old += (C.load - V.cap) * penalty_cap;
    if (C.load_vol > V.cap_vol) p_old += (C.load_vol - V.cap_vol) * penalty_cap;

    auto it = std::find(C.members.begin(), C.members.end(), ui);
    assert(it != C.members.end());
    C.members.erase(it);

    C.sumx -= customer(ui).p.x;
    C.sumy -= customer(ui).p.y;
    C.load -= customer(ui).demand;
    C.load_vol -= customer(ui).vol;

    // Calc "After" penalty
    double p_new = 0.0;
    if (C.load > V.cap) p_new += (C.load - V.cap) * penalty_cap;
    if (C.load_vol > V.cap_vol) p_new += (C.load_vol - V.cap_vol) * penalty_cap;

    total_cost += (p_new - p_old); // Should be negative (reduction)

    cust_cluster[ui] = -1;
    if (!in_unassigned[ui]) {
      in_unassigned[ui] = 1;
      unassigned.push_back(ui);
      unassigned_cnt++;
      total_cost += penalty_unserved;
    }

    C.boundary_dirty = true;
  }

  void add_to_cluster(int ui, int cid) {
    Cluster& C = clusters[cid];
    const Vehicle& V = vehicle(C.vehicle_id);

    // Calc "Before" penalty
    double p_old = 0.0;
    if (C.load > V.cap) p_old += (C.load - V.cap) * penalty_cap;
    if (C.load_vol > V.cap_vol) p_old += (C.load_vol - V.cap_vol) * penalty_cap;

    C.members.push_back(ui);
    C.sumx += customer(ui).p.x;
    C.sumy += customer(ui).p.y;
    C.load += customer(ui).demand;
    C.load_vol += customer(ui).vol;

    // Calc "After" penalty
    double p_new = 0.0;
    if (C.load > V.cap) p_new += (C.load - V.cap) * penalty_cap;
    if (C.load_vol > V.cap_vol) p_new += (C.load_vol - V.cap_vol) * penalty_cap;

    total_cost += (p_new - p_old);

    cust_cluster[ui] = cid;
    if (in_unassigned[ui]) {
      in_unassigned[ui] = 0;
      unassigned_cnt--;
      total_cost -= penalty_unserved;
    }

    C.boundary_dirty = true;
  }

  void compact_unassigned() {
    std::vector<int> tmp;
    tmp.reserve(unassigned.size());
    for (int ui : unassigned) if (in_unassigned[ui]) tmp.push_back(ui);
    unassigned.swap(tmp);
  }

  // Returns penalty change (always >= 0)
  double calc_penalty_add(int ui, int cid) const {
    const Cluster& C = clusters[cid];
    const Vehicle& V = vehicle(C.vehicle_id);
    double p = 0.0;

    // Hard constraints check (still hard?) User said "phạt tràn chuyến".
    // So Capacity/Vol/MaxDist are SOFT.
    // Class constraint ("xe có thể mở chuyến tự do") -> usually implies availability.
    // We keep class HARD for now or standard.
    if (!hooks.vehicle_customer_ok(V, customer(ui))) return 1e12; // Big M for hard constraint

    // Capacity Soft
    double new_load = C.load + customer(ui).demand;
    if (new_load > V.cap) p += (new_load - V.cap) * penalty_cap;

    double new_vol = C.load_vol + customer(ui).vol;
    if (new_vol > V.cap_vol) p += (new_vol - V.cap_vol) * penalty_cap;

    return p;
  }
  
  // Helper to remove penalty of u from C
  double calc_penalty_remove(int ui, int cid) const {
      // In strict delta mode, we need current penalty vs new penalty.
      // Current penalty of C:
      const Cluster& C = clusters[cid];
      const Vehicle& V = vehicle(C.vehicle_id);
      double p_curr = 0.0;
      if (C.load > V.cap) p_curr += (C.load - V.cap) * penalty_cap;
      if (C.load_vol > V.cap_vol) p_curr += (C.load_vol - V.cap_vol) * penalty_cap;

      double rem_load = C.load - customer(ui).demand;
      double rem_vol = C.load_vol - customer(ui).vol;
      double p_new = 0.0;
      if (rem_load > V.cap) p_new += (rem_load - V.cap) * penalty_cap;
      if (rem_vol > V.cap_vol) p_new += (rem_vol - V.cap_vol) * penalty_cap;
      
      return p_curr - p_new; // amount of penalty REDUCED
  }

  bool finalize_cluster(int cid) {
    Cluster& C = clusters[cid];
    const double old_trip = C.trip_km;
    const int vid = C.vehicle_id;
    const int did = C.depot_id;

    const double old_km_pen = hooks.vehicle_km_penalty(vid, vehicle_total_km[vid]);
    const double old_center_pen = (did >= 0 && did < (int)depot_center_penalty.size()) ? depot_center_penalty[did] : 0.0;

    if (!recompute_center(C)) return false;

    // update vehicle trip km + its penalty delta
    apply_trip_change(vid, old_trip, C.trip_km);
    const double new_km_pen = hooks.vehicle_km_penalty(vid, vehicle_total_km[vid]);
    total_cost += (new_km_pen - old_km_pen);

    // update depot center cohesion penalty delta
    if (did >= 0 && did < (int)depot_center_penalty.size()) {
      const double new_center_pen = compute_depot_center_penalty(did);
      depot_center_penalty[did] = new_center_pen;
      total_cost += (new_center_pen - old_center_pen);
    }

    return true;
  }

  bool try_insert(int ui, int cid) {
    if (!in_unassigned[ui]) return false;
    
    // Soft Feasibility check (just hard barrier)
    double pen_increase = calc_penalty_add(ui, cid);
    if (pen_increase >= 1e11) return false; 

    Cluster& C = clusters[cid];
    const double old_trip = C.trip_km;
    double old_depot_pen = hooks.depot_load_penalty(C.depot_id, depot_total_load[C.depot_id]);

    add_to_cluster(ui, cid); // Updates total_cost (capacity + unserved)
    
    depot_total_load[C.depot_id] += customer(ui).demand;
    double new_depot_pen = hooks.depot_load_penalty(C.depot_id, depot_total_load[C.depot_id]);
    total_cost += (new_depot_pen - old_depot_pen);

    if (!finalize_cluster(cid)) {
      // rollback
      total_cost -= (new_depot_pen - old_depot_pen);
      depot_total_load[C.depot_id] -= customer(ui).demand;
      
      remove_from_cluster(ui, cid); // Reverts total_cost
      
      clusters[cid].trip_km = old_trip;
      return false;
    }
    return true;
  }

  bool try_move(int ui, int src, int dst) {
    if (src == dst) return false;
    if (cust_cluster[ui] != src) return false;
    
    double val_add = calc_penalty_add(ui, dst);
    if (val_add >= 1e11) return false;

    Cluster& A = clusters[src];
    const double oldA = A.trip_km;
    
    double old_prop_pen_src = hooks.depot_load_penalty(A.depot_id, depot_total_load[A.depot_id]);

    remove_from_cluster(ui, src); // Updates total_cost
    
    depot_total_load[A.depot_id] -= customer(ui).demand;
    double new_prop_pen_src = hooks.depot_load_penalty(A.depot_id, depot_total_load[A.depot_id]);
    total_cost += (new_prop_pen_src - old_prop_pen_src);

    if (!finalize_cluster(src)) {
      // rollback remove
      total_cost -= (new_prop_pen_src - old_prop_pen_src);
      depot_total_load[A.depot_id] += customer(ui).demand;
      
      add_to_cluster(ui, src); 
      
      clusters[src].trip_km = oldA;
      return false;
    }

    // 2. Insert into Dst (uses try_insert to handle details)
    if (!try_insert(ui, dst)) {
      // rollback back to src (should succeed)
      (void)try_insert(ui, src);
      return false;
    }
    return true;
  }

  bool try_swap(int u, int A_id, int v, int B_id) {
    if (A_id == B_id) return false;
    if (cust_cluster[u] != A_id) return false;
    if (cust_cluster[v] != B_id) return false;

    const Cluster& A = clusters[A_id];
    const Cluster& B = clusters[B_id];
    const Vehicle& VA = vehicle(A.vehicle_id);
    const Vehicle& VB = vehicle(B.vehicle_id);

    const double A_new = A.load - customer(u).demand + customer(v).demand;
    const double B_new = B.load - customer(v).demand + customer(u).demand;
    const double A_new_vol = A.load_vol - customer(u).vol + customer(v).vol;
    const double B_new_vol = B.load_vol - customer(v).vol + customer(u).vol;
    
    if (A_new > VA.cap + 1e-9) return false;
    if (B_new > VB.cap + 1e-9) return false;
    if (A_new_vol > VA.cap_vol + 1e-9) return false;
    if (B_new_vol > VB.cap_vol + 1e-9) return false;
    if (!hooks.vehicle_customer_ok(VA, customer(v))) return false;
    if (!hooks.vehicle_customer_ok(VB, customer(u))) return false;

    // remove both then cross insert; rollback if any fail
    const double old_cost = total_cost;

    int srcA = A_id, srcB = B_id;
    remove_from_cluster(u, srcA);
    depot_total_load[clusters[srcA].depot_id] -= customer(u).demand;
    if (!finalize_cluster(srcA)) { (void)try_insert(u, srcA); return false; }

    remove_from_cluster(v, srcB);
    depot_total_load[clusters[srcB].depot_id] -= customer(v).demand;
    if (!finalize_cluster(srcB)) { (void)try_insert(v, srcB); (void)try_insert(u, srcA); return false; }

    bool ok = try_insert(v, srcA) && try_insert(u, srcB);
    if (!ok) {
      // rollback
      if (in_unassigned[v]) (void)try_insert(v, srcB);
      if (in_unassigned[u]) (void)try_insert(u, srcA);
      total_cost = old_cost; // best-effort; if you need exact, snapshot state instead
      return false;
    }
    return true;
  }

  // -------------------------------
  // Boundary extraction (grid-based)
  // -------------------------------
  void recompute_boundary(int cid) {
    Cluster& C = clusters[cid];
    C.boundary.clear();
    if (C.members.empty()) { C.boundary_dirty = false; return; }

    std::unordered_set<CellKey> cells;
    cells.reserve(C.members.size() * 2);
    for (int ui : C.members) cells.insert(grid.cust_cell[ui]);

    CellKey nb[8];
    for (int ui : C.members) {
      const CellKey cc = grid.cust_cell[ui];
      nbr8_cells(cc, nb);
      bool is_b = false;

      for (int k=0;k<8 && !is_b;k++) {
        const CellKey nk = nb[k];
        // if neighbor cell has anyone not in this cluster => boundary
        auto it = grid.cell2cust.find(nk);
        if (it != grid.cell2cust.end()) {
          for (int w : it->second) {
            if (cust_cluster[w] != cid) { is_b = true; break; }
          }
        } else {
          // empty neighbor also counts as geometric boundary
          is_b = true;
        }
      }
      if (is_b) C.boundary.push_back(ui);
    }

    if (C.boundary.empty()) C.boundary = C.members;
    C.boundary_dirty = false;
  }

  const std::vector<int>& boundary_members(int cid) {
    Cluster& C = clusters[cid];
    if (C.boundary_dirty) recompute_boundary(cid);
    return C.boundary;
  }

  // -------------------------------
  // Validator (catch "kick" bugs early)
  // -------------------------------
  bool validate(const char* /*tag*/ = nullptr) const {
    for (int ui=0; ui<(int)cust_cluster.size(); ++ui) {
      const bool un = in_unassigned[ui] != 0;
      const bool assigned = cust_cluster[ui] != -1;
      if (un == assigned) return false;
    }
    double real_cost = 0.0;
    
    // Sum Cluster costs + Capacity Penalties
    for (const Cluster& C : clusters) {
      for (int ui : C.members) if (cust_cluster[ui] != C.id) return false;
      real_cost += C.trip_km;
      
      const Vehicle& V = vehicle(C.vehicle_id);
      if (C.load > V.cap) real_cost += (C.load - V.cap) * penalty_cap;
      if (C.load_vol > V.cap_vol) real_cost += (C.load_vol - V.cap_vol) * penalty_cap;
    }
    
    // Depot penalties
    for (int i=0; i<(int)depot_total_load.size(); ++i) {
        real_cost += hooks.depot_load_penalty(i, depot_total_load[i]);
    }

    // Vehicle KM penalties
    for (int i=0; i<(int)vehicle_total_km.size(); ++i) {
         real_cost += hooks.vehicle_km_penalty(i, vehicle_total_km[i]);
    }

    // Depot center cohesion penalties
    for (int did=0; did<(int)depot_center_penalty.size(); ++did) {
        real_cost += compute_depot_center_penalty(did);
    }
    
    int real_un = 0;
    for (char c : in_unassigned) if(c) real_un++;
    if (real_un != unassigned_cnt) return false;
    
    real_cost += (double)unassigned_cnt * penalty_unserved;
    
    // Relaxed tolerance for float drift
    if (std::abs(real_cost - total_cost) > 1e-1) {
        // Simple print for debug if needed, but return false triggers assert
        return false;
    }

    return true;
  }
};

// ===============================
// Config
// ===============================
struct GridGrowALNSConfig {
  double cell_size_km = 0.02;  // 20m
  int    grow_budget = 64;     // per affected cluster
  int    vnd_iters = 60;

  double peel_frac = 0.25;     // boundary peel fraction
  int    region_cells = 8;     // connected region (cells) removal
  double destroy_whole_prob = 0.03;

  // Grow scoring weights
  double w_centroid = 1.0;
  double w_center   = 0.7;
  double w_cell     = 0.3;

  // Seeding (when cluster empty)
  int    seed_ring_max = 25;           // ring radius in grid cells around a depot/patch anchor
  int    seed_need_cands = 80;         // stop ring scan once we have this many candidates
  int    seed_global_sample = 400;     // fallback sample from global unassigned if ring finds none
  double w_patch = 0.35;               // pull candidates toward depot's existing center-patch

  // Depot center cohesion penalty (creates "territory" from centers)
  double center_adj_radius_km = 0.6;   // within this distance, centers are considered adjacent
  double w_center_adj = 25.0;          // penalty weight for isolated centers
  double w_center_cc  = 200.0;         // penalty for multiple disconnected components (per depot)

  // SA
  double T0 = 0.05;
  double alpha = 0.995;
};

// ===============================
// Grow (Repair)
// ===============================
struct GrowOps {
  struct Cand { int ui=-1; double score=std::numeric_limits<double>::infinity(); };

  static Cand pick_best_candidate(State& st, int cid, const GridGrowALNSConfig& cfg) {
    const Cluster& C = st.clusters[cid];

    Pt centroid{0,0};
    Pt center_pt{0,0};
    Pt patch_anchor{0,0};

    // Patch anchor: nearest existing center of SAME depot (creates "territory" from centers).
    auto pick_patch_anchor = [&](int depot_id, Pt ref) -> Pt {
      int best_ui = -1;
      double best = std::numeric_limits<double>::infinity();
      for (const auto& CC : st.clusters) {
        if (CC.depot_id != depot_id) continue;
        if (CC.id == cid) continue;
        if (CC.center_ui < 0) continue;
        double d = dist_km(ref, st.customer(CC.center_ui).p);
        if (d < best) { best = d; best_ui = CC.center_ui; }
      }
      if (best_ui >= 0) return st.customer(best_ui).p;
      return st.depot(depot_id).p;
    };

    if (!C.members.empty()) {
      const double cx = C.sumx / (double)C.members.size();
      const double cy = C.sumy / (double)C.members.size();
      centroid = {cx, cy};
      center_pt = (C.center_ui >= 0) ? st.customer(C.center_ui).p : centroid;
      patch_anchor = pick_patch_anchor(C.depot_id, center_pt);
    } else {
      // empty cluster: seed near depot's current patch (or depot if no active patch)
      patch_anchor = pick_patch_anchor(C.depot_id, st.depot(C.depot_id).p);
      centroid = patch_anchor;
      center_pt = patch_anchor;
    }

    std::unordered_set<int> cand;
    cand.reserve(256);

    CellKey nb[8];

    if (C.members.empty()) {
      // --- ring scan around patch anchor cell ---
      const int ar = (int)std::floor(patch_anchor.y / st.grid.cell_size_km);
      const int ac = (int)std::floor(patch_anchor.x / st.grid.cell_size_km);

      auto add_cell = [&](CellKey k) {
        auto it = st.grid.cell2cust.find(k);
        if (it == st.grid.cell2cust.end()) return;
        for (int u : it->second) cand.insert(u);
      };

      for (int R=0; R<=cfg.seed_ring_max; ++R) {
        // ring boundary: max(|dr|,|dc|)==R
        for (int dr=-R; dr<=R; ++dr) {
          for (int dc=-R; dc<=R; ++dc) {
            if (std::max(std::abs(dr), std::abs(dc)) != R) continue;
            add_cell(pack_cell(ar + dr, ac + dc));
          }
        }
        if ((int)cand.size() >= cfg.seed_need_cands) break;
      }

      // fallback: sample global unassigned
      if (cand.empty() && !st.unassigned.empty()) {
        const int lim = std::min((int)st.unassigned.size(), cfg.seed_global_sample);
        for (int t=0; t<lim; ++t) {
          int idx = st.rand_int(0, (int)st.unassigned.size()-1);
          int ui = st.unassigned[idx];
          if (st.in_unassigned[ui]) cand.insert(ui);
        }
      }
    } else {
      // --- frontier expansion from boundary ---
      const auto& boundary = st.boundary_members(cid);
      cand.reserve(std::max((size_t)128, boundary.size() * 12));
      for (int b_ui : boundary) {
        CellKey cc = st.grid.cust_cell[b_ui];
        if (auto it0 = st.grid.cell2cust.find(cc); it0 != st.grid.cell2cust.end())
          for (int u : it0->second) cand.insert(u);

        nbr8_cells(cc, nb);
        for (int k=0;k<8;k++) {
          auto it = st.grid.cell2cust.find(nb[k]);
          if (it == st.grid.cell2cust.end()) continue;
          for (int u : it->second) cand.insert(u);
        }
      }
    }

    Cand best;
    const auto& boundary = st.boundary_members(cid);

    for (int ui : cand) {
      if (!st.in_unassigned[ui]) continue;
      if (st.calc_penalty_add(ui, cid) >= 1e11) continue; // hard filter

      const double d1 = dist_km(st.customer(ui).p, centroid);
      const double d2 = dist_km(st.customer(ui).p, center_pt);
      const double dp = dist_km(st.customer(ui).p, patch_anchor);

      // tiny grid-adjacency bonus if neighbor of boundary (only meaningful if non-empty)
      double cell_bonus = 0.0;
      if (!C.members.empty() && !boundary.empty()) {
        const CellKey cu = st.grid.cust_cell[ui];
        for (int b_ui : boundary) {
          const CellKey cb = st.grid.cust_cell[b_ui];
          const int dr = std::abs(cell_row(cu) - cell_row(cb));
          const int dc = std::abs(cell_col(cu) - cell_col(cb));
          if (dr<=1 && dc<=1) { cell_bonus = -1.0; break; }
        }
      }

      const double score = cfg.w_centroid*d1 + cfg.w_center*d2 + cfg.w_patch*dp + cfg.w_cell*cell_bonus;
      if (score < best.score) best = {ui, score};
    }
    return best;
  }

  static bool regrow_cluster(State& st, int cid, const GridGrowALNSConfig& cfg, int budget) {
    bool changed = false;
    for (int s=0; s<budget; ++s) {
      auto c = pick_best_candidate(st, cid, cfg);
      if (c.ui < 0) break;
      if (st.try_insert(c.ui, cid)) changed = true;
    }
    return changed;
  }
};

// ===============================
// Destroy operators
// ===============================
struct DestroyOps {

  static std::vector<int> boundary_peel(State& st, int cid, double frac) {
    std::vector<int> removed;
    Cluster& C = st.clusters[cid];
    if (C.members.empty()) return removed;

    const auto& bnd = st.boundary_members(cid);
    const int k = std::max(1, (int)std::round(frac * (double)bnd.size()));

    std::vector<int> perm = bnd;
    std::shuffle(perm.begin(), perm.end(), st.rng);

    for (int i=0;i<k && i<(int)perm.size();++i) {
      int ui = perm[i];
      if (st.cust_cluster[ui] != cid) continue;

      const double old_trip = C.trip_km;
      st.remove_from_cluster(ui, cid);
      st.depot_total_load[C.depot_id] -= st.customer(ui).demand;
      if (!st.finalize_cluster(cid)) {
        (void)st.try_insert(ui, cid);
        C.trip_km = old_trip;
      } else {
        removed.push_back(ui);
      }
    }
    return removed;
  }

  static std::vector<int> connected_region_cells(State& st, int cid, int target_cells) {
    std::vector<int> removed;
    Cluster& C = st.clusters[cid];
    if (C.members.empty()) return removed;

    std::unordered_map<CellKey, std::vector<int>> cell_members;
    cell_members.reserve(C.members.size());
    for (int ui : C.members) cell_members[st.grid.cust_cell[ui]].push_back(ui);

    if (cell_members.empty()) return removed;

    // seed: boundary cell if possible
    CellKey seed = st.grid.cust_cell[C.members[ st.rand_int(0, (int)C.members.size()-1) ]];
    if (C.boundary_dirty) st.recompute_boundary(cid);
    if (!C.boundary.empty()) seed = st.grid.cust_cell[C.boundary[ st.rand_int(0, (int)C.boundary.size()-1) ]];

    std::unordered_set<CellKey> vis;
    std::vector<CellKey> q;
    vis.reserve(target_cells*4);
    q.reserve(target_cells*2);

    q.push_back(seed);
    vis.insert(seed);

    CellKey nb[8];
    for (int qi=0; qi<(int)q.size() && (int)vis.size()<target_cells; ++qi) {
      CellKey cur = q[qi];
      nbr8_cells(cur, nb);
      for (int k=0;k<8 && (int)vis.size()<target_cells;k++) {
        CellKey nxt = nb[k];
        if (vis.count(nxt)) continue;
        if (cell_members.find(nxt) == cell_members.end()) continue;
        vis.insert(nxt);
        q.push_back(nxt);
      }
    }

    std::vector<int> to_remove;
    for (CellKey ck : vis) {
      auto it = cell_members.find(ck);
      if (it != cell_members.end()) for (int ui : it->second) to_remove.push_back(ui);
    }
    std::shuffle(to_remove.begin(), to_remove.end(), st.rng);

    for (int ui : to_remove) {
      if (st.cust_cluster[ui] != cid) continue;
      const double old_trip = C.trip_km;
      st.remove_from_cluster(ui, cid);
      st.depot_total_load[C.depot_id] -= st.customer(ui).demand;
      if (!st.finalize_cluster(cid)) {
        (void)st.try_insert(ui, cid);
        C.trip_km = old_trip;
      } else {
        removed.push_back(ui);
      }
    }
    return removed;
  }

  static bool center_shift(State& st, int cid) {
    Cluster& C = st.clusters[cid];
    if ((int)C.members.size() < 2) return false;

    int pick = C.center_ui;
    for (int t=0;t<10 && pick==C.center_ui; ++t)
      pick = C.members[ st.rand_int(0, (int)C.members.size()-1) ];
    if (pick == C.center_ui) return false;

    const double trip = 2.0 * st.depot_customer_km(C.depot_id, pick);
    if (!st.hooks.center_ok(C.depot_id, pick, trip)) return false;

    const double old = C.trip_km;
    C.center_ui = pick;
    C.trip_km = trip;
    st.apply_trip_change(C.vehicle_id, old, trip);
    C.boundary_dirty = true;
    return true;
  }

  static std::pair<int,int> pick_neighbor_clusters(State& st) {
    const int m = (int)st.clusters.size();
    if (m < 2) return {-1,-1};

    int a = st.rand_int(0, m-1);
    if (st.clusters[a].members.empty()) return {-1,-1};

    const auto& bndA = st.boundary_members(a);
    std::unordered_set<CellKey> cellsA;
    cellsA.reserve(bndA.size()*2);
    for (int ui : bndA) cellsA.insert(st.grid.cust_cell[ui]);

    for (int t=0;t<30;++t) {
      int b = st.rand_int(0, m-1);
      if (b==a || st.clusters[b].members.empty()) continue;
      const auto& bndB = st.boundary_members(b);
      for (int v : bndB) {
        CellKey cv = st.grid.cust_cell[v];
        for (CellKey ca : cellsA) {
          const int dr = std::abs(cell_row(cv) - cell_row(ca));
          const int dc = std::abs(cell_col(cv) - cell_col(ca));
          if (dr<=1 && dc<=1) return {a,b};
        }
      }
    }
    int b = (a + 1 + st.rand_int(0, m-2)) % m;
    return {a,b};
  }
};

// ===============================
// Boundary VND
// ===============================
struct VND {
  static bool boundary_relocate(State& st) {
    auto [A,B] = DestroyOps::pick_neighbor_clusters(st);
    if (A<0 || B<0) return false;
    const auto& bndA = st.boundary_members(A);
    if (bndA.empty()) return false;

    int u = bndA[ st.rand_int(0, (int)bndA.size()-1) ];
    const double old = st.total_cost;
    if (st.try_move(u, A, B)) return st.total_cost <= old + 1e-12;
    return false;
  }

  static bool boundary_swap(State& st) {
    auto [A,B] = DestroyOps::pick_neighbor_clusters(st);
    if (A<0 || B<0) return false;

    const auto& bndA = st.boundary_members(A);
    const auto& bndB = st.boundary_members(B);
    if (bndA.empty() || bndB.empty()) return false;

    int u = bndA[ st.rand_int(0, (int)bndA.size()-1) ];
    int v = bndB[ st.rand_int(0, (int)bndB.size()-1) ];
    const double old = st.total_cost;
    if (st.try_swap(u, A, v, B)) return st.total_cost <= old + 1e-12;
    return false;
  }

  static void run(State& st, int iters) {
    for (int i=0;i<iters;i++) {
      if (boundary_relocate(st)) continue;
      if (boundary_swap(st)) continue;
    }
  }
};

// ===============================
// Grid-Grow ALNS engine
// ===============================
class GridGrowALNS {
public:
  GridGrowALNSConfig cfg;

  enum DestroyType { PEEL=0, REGION=1, CENTER_SHIFT=2, NUM=3 };

  struct Weights {
    double w[NUM] = {1,1,1};
    double score[NUM] = {0,0,0};
    int used[NUM] = {0,0,0};
  };

  static int roulette_pick(State& st, const Weights& W) {
    double sum = 0;
    for (int i=0;i<NUM;i++) sum += std::max(1e-9, W.w[i]);
    double r = st.rand01() * sum;
    for (int i=0;i<NUM;i++) {
      r -= std::max(1e-9, W.w[i]);
      if (r <= 0) return i;
    }
    return NUM-1;
  }

  static void update_weights(Weights& W, double rho=0.2) {
    for (int i=0;i<NUM;i++) {
      if (W.used[i] == 0) continue;
      const double avg = W.score[i] / (double)W.used[i];
      W.w[i] = (1.0 - rho)*W.w[i] + rho*avg;
      W.score[i] = 0;
      W.used[i] = 0;
    }
  }

  double run(State& st, int iters) {
    if (st.grid.cust_cell.empty()) st.grid.build(*st.cust, cfg.cell_size_km);

    // sync center cohesion config into State and (re)compute initial penalties
    st.center_adj_radius_km = cfg.center_adj_radius_km;
    st.w_center_adj = cfg.w_center_adj;
    st.w_center_cc  = cfg.w_center_cc;

    if (st.depot_center_penalty.size() != st.depot_total_load.size())
      st.depot_center_penalty.assign(st.depot_total_load.size(), 0.0);

    double old_sum = 0.0, new_sum = 0.0;
    for (double v : st.depot_center_penalty) old_sum += v;
    for (int did=0; did<(int)st.depot_center_penalty.size(); ++did) {
      st.depot_center_penalty[did] = st.compute_depot_center_penalty(did);
      new_sum += st.depot_center_penalty[did];
    }
    st.total_cost += (new_sum - old_sum);

    Weights W;
    double T = cfg.T0;

    State best = st;
    double best_cost = st.total_cost;

    for (int it=0; it<iters; ++it) {
      State backup = st;
      const double base = st.total_cost;

      const int dtype = roulette_pick(st, W);
      W.used[dtype]++;

      int cid = st.rand_int(0, (int)st.clusters.size()-1);
      if (st.clusters[cid].members.empty()) cid = (cid + 1) % (int)st.clusters.size();

      // Destroy
      if (dtype == PEEL) {
        (void)DestroyOps::boundary_peel(st, cid, cfg.peel_frac);
      } else if (dtype == REGION) {
        (void)DestroyOps::connected_region_cells(st, cid, cfg.region_cells);
      } else if (dtype == CENTER_SHIFT) {
        (void)DestroyOps::center_shift(st, cid);
      }

      // Repair (regrow affected cluster strongly, plus a couple random clusters lightly)
      // Repair (regrow affected cluster strongly, plus a couple random clusters lightly)
      GrowOps::regrow_cluster(st, cid, cfg, cfg.grow_budget);
      for (int pass=0; pass<4; ++pass) {
        int c2 = st.rand_int(0, (int)st.clusters.size()-1);
        GrowOps::regrow_cluster(st, c2, cfg, cfg.grow_budget/2);
      }

      // Boundary tightening
      VND::run(st, cfg.vnd_iters);

      st.compact_unassigned();

      const double now = st.total_cost;
      const double delta = now - base;

      bool accept = false;
      if (delta <= 1e-12) accept = true;
      else {
        const double p = std::exp(-delta / std::max(1e-9, T));
        accept = (st.rand01() < p);
      }

      if (!accept) {
        st = std::move(backup);
      } else {
        const double reward = (now < best_cost - 1e-12) ? 5.0 : (delta <= 0 ? 2.0 : 0.5);
        W.score[dtype] += reward;

        if (now < best_cost - 1e-12) {
          best = st;
          best_cost = now;
        }
      }

      T *= cfg.alpha;
      if ((it+1) % 50 == 0) {
          update_weights(W);
          std::cout << "[GridGrow] Iter " << std::setw(5) << (it+1) 
                    << " | Best: " << std::fixed << std::setprecision(2) << best_cost 
                    << " | Curr: " << now 
                    << " | Unserved: " << st.unassigned_cnt 
                    << " | T: " << T << "\n";
      }

      assert(st.validate("ALNS"));
    }

    st = std::move(best);
    return best_cost;
  }
};

} // namespace gg_alns
