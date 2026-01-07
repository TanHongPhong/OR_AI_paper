#!/usr/bin/env python3
"""
Visualize all ALNS iteration snapshots with CONSISTENT vehicle colors.

Each vehicle (and its territory/region) maintains the SAME color across all
iterations, allowing you to track how territories change during optimization.

Usage:
    python visualize_all_snapshots.py --run_dir out/run_YYYYMMDD_HHMMSS --data_dir .
"""

import argparse
import os
import re
import glob
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon
from scipy.spatial import ConvexHull

def split_tokens(s: str):
    if s is None:
        return []
    s = str(s).strip()
    if not s:
        return []
    return re.split(r"\s+", s)

def _is_collinear(p0, p1, p2):
    return (p0[0] == p1[0] == p2[0]) or (p0[1] == p1[1] == p2[1])

def _simplify_loop(loop):
    if len(loop) < 4:
        return loop
    out = []
    n = len(loop)
    for i in range(n):
        p_prev = loop[i - 1]
        p = loop[i]
        p_next = loop[(i + 1) % n]
        if _is_collinear(p_prev, p, p_next):
            continue
        out.append(p)
    return out if len(out) >= 3 else loop

def boundary_loops_from_cells(cells):
    """Trace outer boundaries of union of grid cells."""
    cell_set = set((int(x), int(y)) for x, y in cells)
    if not cell_set:
        return []

    out_edges = {}
    used = set()

    def add_dir(a, b):
        out_edges.setdefault(a, []).append(b)

    for x, y in cell_set:
        tl = (x - 0.5, y - 0.5)
        tr = (x + 0.5, y - 0.5)
        br = (x + 0.5, y + 0.5)
        bl = (x - 0.5, y + 0.5)

        if (x, y - 1) not in cell_set:
            add_dir(tl, tr)
        if (x + 1, y) not in cell_set:
            add_dir(tr, br)
        if (x, y + 1) not in cell_set:
            add_dir(br, bl)
        if (x - 1, y) not in cell_set:
            add_dir(bl, tl)

    def vdir(a, b):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        if dx != 0:
            dx = int(np.sign(dx))
        if dy != 0:
            dy = int(np.sign(dy))
        return (dx, dy)

    right_turn = {(1, 0): (0, 1), (0, 1): (-1, 0), (-1, 0): (0, -1), (0, -1): (1, 0)}
    left_turn  = {(1, 0): (0, -1), (0, 1): (1, 0), (-1, 0): (0, 1), (0, -1): (-1, 0)}
    back_turn  = {(1, 0): (-1, 0), (0, 1): (0, -1), (-1, 0): (1, 0), (0, -1): (0, 1)}

    def pick_next(cur, incoming_dir):
        cands = out_edges.get(cur, [])
        if not cands:
            return None

        dir2ends = {}
        for e in cands:
            if (cur, e) in used:
                continue
            d = vdir(cur, e)
            dir2ends.setdefault(d, []).append(e)

        if not dir2ends:
            return None

        for d in (right_turn[incoming_dir], incoming_dir, left_turn[incoming_dir], back_turn[incoming_dir]):
            if d in dir2ends:
                dir2ends[d].sort()
                return dir2ends[d][0]
        d0 = sorted(dir2ends.keys())[0]
        dir2ends[d0].sort()
        return dir2ends[d0][0]

    loops = []
    all_edges = [(a, b) for a, outs in out_edges.items() for b in outs]

    for a, b in all_edges:
        if (a, b) in used:
            continue

        loop = [a, b]
        used.add((a, b))
        start = a
        prev, cur = a, b
        incoming = vdir(prev, cur)

        guard = 0
        while guard < 200000:
            guard += 1
            nxt = pick_next(cur, incoming)
            if nxt is None:
                break

            used.add((cur, nxt))
            loop.append(nxt)

            if nxt == start:
                loop = loop[:-1]
                loop = _simplify_loop(loop)
                if len(loop) >= 3:
                    loops.append(loop)
                break

            prev, cur = cur, nxt
            incoming = vdir(prev, cur)

    return loops

def load_depots(depots_csv: str):
    df = pd.read_csv(depots_csv)
    did_col = "Depot_ID" if "Depot_ID" in df.columns else "depot_id"
    depots = {}
    for _, r in df.iterrows():
        depots[str(r[did_col])] = {"id": str(r[did_col]), "row": int(r["row"]), "col": int(r["col"])}
    return depots

def build_vehicle_color_map(run_dir: str):
    """
    Scan all snapshots to find all unique vehicle_ids.
    Assign each vehicle a consistent color.
    """
    vehicle_ids = set()
    
    # Find all iter_* directories
    iter_dirs = sorted(glob.glob(os.path.join(run_dir, "iter_*")))
    
    for iter_dir in iter_dirs:
        routes_csv = os.path.join(iter_dir, "alns_routes.csv")
        if os.path.exists(routes_csv):
            df = pd.read_csv(routes_csv)
            if "vehicle_id" in df.columns:
                vehicle_ids.update(df["vehicle_id"].unique())
    
    # Also check root directory
    routes_csv = os.path.join(run_dir, "alns_routes.csv")
    if os.path.exists(routes_csv):
        df = pd.read_csv(routes_csv)
        if "vehicle_id" in df.columns:
            vehicle_ids.update(df["vehicle_id"].unique())
    
    # Sort for consistent ordering
    vehicle_list = sorted(vehicle_ids)
    
    # Create color map using a colormap with good distinguishability
    cmap = plt.cm.get_cmap('tab20', len(vehicle_list) + 1)
    color_map = {vid: cmap(i) for i, vid in enumerate(vehicle_list)}
    
    return color_map

def visualize_snapshot(snapshot_dir, data_dir, vehicle_color_map, out_png, 
                       marker_size=8.0, invert_y=True, show_links=True,
                       show_hull=True, no_cover=False):
    """Visualize a single snapshot with consistent vehicle colors."""
    
    sparse_csv = os.path.join(data_dir, "customers_sparse_grid.csv")
    depots_csv = os.path.join(data_dir, "depots_grid_min.csv")
    meta_txt = os.path.join(data_dir, "grid_meta.txt")
    routes_csv = os.path.join(snapshot_dir, "alns_routes.csv")
    summary_csv = os.path.join(snapshot_dir, "alns_summary.csv")
    
    if not os.path.exists(routes_csv):
        print(f"[SKIP] No routes in {snapshot_dir}")
        return False
    
    sparse_df = pd.read_csv(sparse_csv)
    sparse_df.columns = [c.lower() for c in sparse_df.columns]
    depots = load_depots(depots_csv)
    
    width = 0
    if os.path.exists(meta_txt):
        with open(meta_txt, 'r') as f:
            for line in f:
                if line.startswith("width="):
                    width = int(line.split("=")[1])
                    break
    if width <= 0:
        width = int(sparse_df["col"].max() + 1)
    
    # Load stats
    stats = {}
    if os.path.exists(summary_csv):
        sdf = pd.read_csv(summary_csv)
        if "metric" in sdf.columns and "value" in sdf.columns:
            stats = dict(zip(sdf['metric'], sdf['value']))
    
    # Extract iteration from directory name
    iter_match = re.search(r'iter_(\d+)', os.path.basename(snapshot_dir))
    iter_num = int(iter_match.group(1)) if iter_match else 0
    
    title = f"Iteration {iter_num}: {int(stats.get('trips', 0))} Trips, {int(stats.get('assigned_cells', 0))} Served, {int(stats.get('unassigned_cells', 0))} Unserved"
    
    routes = pd.read_csv(routes_csv).to_dict('records') if os.path.exists(routes_csv) else []
    
    fig, ax = plt.subplots(figsize=(15, 15))
    ax.grid(True, linestyle=':', alpha=0.4, color='lightgrey', zorder=0)
    
    # Background - unassigned customers
    ax.scatter(sparse_df["col"], sparse_df["row"], c='#e0e0e0', s=marker_size, alpha=0.2,
               marker='s', label="Customer (unassigned)", zorder=1)
    
    center_depot_links = []
    
    for rt in routes:
        vehicle_id = rt.get("vehicle_id", "")
        color = vehicle_color_map.get(vehicle_id, (0.5, 0.5, 0.5, 1.0))
        
        tokens = split_tokens(str(rt.get("member_cells", "")))
        pts = []
        for t in tokens:
            try:
                lin = int(t)
                pts.append((lin % width, lin // width))
            except:
                pass
        
        if not pts:
            continue
        
        pts = np.asarray(pts, dtype=float)
        
        # --- Boundary cover (grid based) ---
        if not no_cover:
            loops = boundary_loops_from_cells([(p[0], p[1]) for p in pts])
            for lp in loops:
                xs = [p[0] for p in lp] + [lp[0][0]]
                ys = [p[1] for p in lp] + [lp[0][1]]
                
                # Subtle fill color
                fc = (color[0], color[1], color[2], 0.12)
                ax.add_patch(Polygon(lp, closed=True, facecolor=fc, edgecolor='none', zorder=2.1))
                
                # Subtle but clear outline
                ax.plot(xs, ys, color='white', linewidth=2.8, alpha=0.9, zorder=6)
                ec = (color[0], color[1], color[2], 0.8)
                ax.plot(xs, ys, color=ec, linewidth=1.5, alpha=0.8, zorder=7)
        
        # --- Convex Hull (subtle secondary layer) ---
        if show_hull and len(pts) >= 3:
            try:
                hull = ConvexHull(pts)
                hull_pts = np.vstack([pts[hull.vertices], pts[hull.vertices[0]]])
                # Very subtle fill
                ax.fill(hull_pts[:, 0], hull_pts[:, 1], color=color, alpha=0.06, zorder=2)
                # Subtle outline
                ax.plot(hull_pts[:, 0], hull_pts[:, 1], color=color, linewidth=1.0, alpha=0.4, zorder=2)
            except:
                pass
        
        # Customer markers
        ax.scatter(pts[:, 0], pts[:, 1], color=color, s=marker_size, marker='s',
                   edgecolors='white', linewidths=0.3, alpha=0.95, zorder=4)
        
        # Cluster center
        if "center_col" in rt and "center_row" in rt:
            cx, cy = rt["center_col"], rt["center_row"]
            if pd.notna(cx) and pd.notna(cy) and cx >= 0 and cy >= 0:
                ax.scatter([cx], [cy], color=color, s=80, marker='D',
                           edgecolors='black', linewidths=1.5, zorder=8)
                depot_id = rt.get("depot_id", "")
                if depot_id:
                    center_depot_links.append((cx, cy, depot_id, color))
    
    # Draw center-depot links
    if show_links:
        for cx, cy, depot_id, color in center_depot_links:
            if depot_id in depots:
                d = depots[depot_id]
                dx, dy = d["col"], d["row"]
                ax.plot([cx, dx], [cy, dy], color=color, linewidth=1.2, alpha=0.5,
                        linestyle='--', zorder=3)
    
    # Depots
    for depot_id, d in depots.items():
        ax.scatter(d["col"], d["row"], marker="*", s=400, c='red',
                   edgecolors="black", linewidths=2.0, zorder=15)
        ax.annotate(d["id"], (d["col"], d["row"]), fontsize=10, fontweight='bold',
                    xytext=(8, 8), textcoords='offset points',
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.9, edgecolor='black'))
    
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("col", fontsize=11)
    ax.set_ylabel("row", fontsize=11)
    ax.set_title(title, fontsize=15, fontweight='bold', pad=20)
    if invert_y:
        ax.invert_yaxis()
    
    legend_elements = [
        Line2D([0], [0], marker='s', color='w', label='Customer (by vehicle)', markerfacecolor='blue', markersize=10, markeredgecolor='white'),
        Line2D([0], [0], marker='D', color='w', label='Cluster Center', markerfacecolor='green', markeredgecolor='black', markersize=10),
        Line2D([0], [0], marker='*', color='w', label='Depot', markerfacecolor='red', markeredgecolor='black', markersize=14),
        Line2D([0], [0], linestyle='--', color='grey', label='Center-Depot Link', linewidth=1.5),
        Line2D([0], [0], marker='s', color='w', label='Unassigned', markerfacecolor='lightgrey', markersize=8),
    ]
    ax.legend(handles=legend_elements, loc="upper right", framealpha=0.95, fontsize=11)
    
    plt.tight_layout()
    plt.savefig(out_png, dpi=150)
    plt.close()
    print(f"[OK] Saved: {out_png}")
    return True

def main():
    ap = argparse.ArgumentParser(description="Visualize all ALNS snapshots with consistent vehicle colors")
    ap.add_argument("--run_dir", required=True, help="Directory containing iter_* snapshot folders")
    ap.add_argument("--data_dir", default="", help="Directory containing static grid data (customers, depots). Defaults to run_dir parent if empty.")
    ap.add_argument("--s", type=float, default=8.0, help="Customer marker size")
    ap.add_argument("--show_hull", action="store_true", default=True, help="Draw convex hull boundaries")
    ap.add_argument("--no_cover", action="store_true", help="Disable filled boundary cover per cluster")
    ap.add_argument("--no_hull", dest="show_hull", action="store_false", help="Disable convex hull")
    args = ap.parse_args()
    
    run_dir = args.run_dir
    data_dir = args.data_dir if args.data_dir else os.path.dirname(run_dir)
    if not data_dir:
        data_dir = "."
    
    print(f"Run directory: {run_dir}")
    print(f"Data directory: {data_dir}")
    
    # Build consistent color map for all vehicles
    print("\n[1/3] Building vehicle color map...")
    vehicle_color_map = build_vehicle_color_map(run_dir)
    print(f"      Found {len(vehicle_color_map)} unique vehicles")
    
    # Find all snapshot directories
    print("\n[2/3] Finding snapshots...")
    iter_dirs = sorted(glob.glob(os.path.join(run_dir, "iter_*")))
    print(f"      Found {len(iter_dirs)} snapshot directories")
    
    # Visualize each snapshot
    print("\n[3/3] Generating visualizations...")
    for iter_dir in iter_dirs:
        iter_name = os.path.basename(iter_dir)
        out_png = os.path.join(iter_dir, f"viz_{iter_name}.png")
        visualize_snapshot(iter_dir, data_dir, vehicle_color_map, out_png, args.s, 
                           show_hull=args.show_hull, no_cover=args.no_cover)
    
    # Also visualize final result in root
    if os.path.exists(os.path.join(run_dir, "alns_routes.csv")):
        out_png = os.path.join(run_dir, "viz_final.png")
        visualize_snapshot(run_dir, data_dir, vehicle_color_map, out_png, args.s,
                           show_hull=args.show_hull, no_cover=args.no_cover)
    
    print(f"\n[DONE] All visualizations saved to {run_dir}")

if __name__ == "__main__":
    main()
