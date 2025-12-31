import argparse
import os
import re
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-GUI backend
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


def _edge_key(a, b):
    return (a, b) if a <= b else (b, a)

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
    """Build boundary polygon loops (union outline) from integer cell centers (x,y).

    Returns a list of loops, each loop is a list of (x,y) vertices (floats) on half-grid corners.
    """
    cell_set = set((int(x), int(y)) for x, y in cells)
    if not cell_set:
        return []

    edges = set()

    def add_edge(a, b):
        edges.add(_edge_key(a, b))

    for x, y in cell_set:
        # Unit square around the cell center (x,y)
        tl = (x - 0.5, y - 0.5)
        tr = (x + 0.5, y - 0.5)
        br = (x + 0.5, y + 0.5)
        bl = (x - 0.5, y + 0.5)

        if (x, y - 1) not in cell_set:  # top exposed
            add_edge(tl, tr)
        if (x, y + 1) not in cell_set:  # bottom exposed
            add_edge(bl, br)
        if (x - 1, y) not in cell_set:  # left exposed
            add_edge(tl, bl)
        if (x + 1, y) not in cell_set:  # right exposed
            add_edge(tr, br)

    if not edges:
        return []

    # adjacency
    adj = {}
    for a, b in edges:
        adj.setdefault(a, []).append(b)
        adj.setdefault(b, []).append(a)

    visited = set()
    loops = []

    for a, b in list(edges):
        e0 = _edge_key(a, b)
        if e0 in visited:
            continue

        visited.add(e0)
        loop = [a, b]
        prev, cur = a, b

        guard = 0
        while cur != a and guard < 100000:
            guard += 1
            nbrs = adj.get(cur, [])
            if not nbrs:
                break

            nxt = None
            for cand in nbrs:
                if cand == prev:
                    continue
                ek = _edge_key(cur, cand)
                if ek not in visited:
                    nxt = cand
                    break
            if nxt is None:
                for cand in nbrs:
                    ek = _edge_key(cur, cand)
                    if ek not in visited:
                        nxt = cand
                        break
            if nxt is None:
                break

            visited.add(_edge_key(cur, nxt))
            loop.append(nxt)
            prev, cur = cur, nxt

        if len(loop) >= 4 and loop[-1] == a:
            loop = _simplify_loop(loop[:-1])
            if len(loop) >= 3:
                loops.append(loop)

    return loops

def build_id_to_cell(grid_dir: str):
    id_map_csv = os.path.join(grid_dir, "customer_id_map.csv")
    sparse_csv = os.path.join(grid_dir, "customers_sparse_grid.csv")
    id_map_df = pd.read_csv(id_map_csv)
    sparse_df = pd.read_csv(sparse_csv)
    id_map_df.columns = [c.lower() for c in id_map_df.columns]
    sparse_df.columns = [c.lower() for c in sparse_df.columns]
    idx_to_rc = {}
    for _, r in sparse_df.iterrows():
        idx_tokens = split_tokens(r.get("customer_id_idx", ""))
        for t in idx_tokens:
            try: idx_to_rc[int(t)] = (int(r["row"]), int(r["col"]))
            except: pass
    id_to_rc = {}
    for _, r in id_map_df.iterrows():
        try:
            cid = str(r["customer_id"]).strip()
            idx = int(r["customer_id_idx"])
            if idx in idx_to_rc: id_to_rc[cid] = idx_to_rc[idx]
        except: pass
    return id_to_rc

def load_depots(depots_csv: str):
    df = pd.read_csv(depots_csv)
    did_col = "Depot_ID" if "Depot_ID" in df.columns else "depot_id"
    depots = []
    for _, r in df.iterrows():
        depots.append({"id": str(r[did_col]), "row": int(r["row"]), "col": int(r["col"])})
    return depots

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--grid_out", required=True, help="Folder containing input and output CSVs")
    ap.add_argument("--routes", default="alns_routes.csv", help="Route details (trip-based)")
    ap.add_argument("--summary", default="alns_summary.csv", help="Summary stats")
    ap.add_argument("--out_png", default="viz_clusters_v2.png")
    ap.add_argument("--s", type=float, default=4.0, help="Customer marker size")
    ap.add_argument("--show_hull", action="store_true", help="Draw convex hull boundaries")
    ap.add_argument("--no_cover", action="store_true", help="Disable filled boundary cover per cluster")
    ap.add_argument("--invert_y", action="store_true", default=True, help="Invert Y axis")
    args = ap.parse_args()

    # Paths
    sparse_csv = os.path.join(args.grid_out, "customers_sparse_grid.csv")
    depots_csv = os.path.join(args.grid_out, "depots_grid_min.csv")
    routes_csv = os.path.join(args.grid_out, args.routes)
    summary_csv = os.path.join(args.grid_out, args.summary)
    meta_txt = os.path.join(args.grid_out, "grid_meta.txt")
    out_png = os.path.join(args.grid_out, args.out_png)

    # Load data
    sparse_df = pd.read_csv(sparse_csv)
    sparse_df.columns = [c.lower() for c in sparse_df.columns]
    depots = load_depots(depots_csv)
    
    width = 0
    if os.path.exists(meta_txt):
        with open(meta_txt, 'r') as f:
            for line in f:
                if line.startswith("width="): width = int(line.split("=")[1]); break
    if width <= 0: width = sparse_df["col"].max() + 1

    # Get Summary Stats for Title
    stats = {}
    if os.path.exists(summary_csv):
        sdf = pd.read_csv(summary_csv)
        stats = dict(zip(sdf['metric'], sdf['value']))
    
    title = f"ALNS Solution: {int(stats.get('trips', 0))} Trips, {int(stats.get('assigned_cells', 0))} Served, {int(stats.get('unassigned_cells', 0))} Unserved"

    # Process Routes
    routes = []
    if os.path.exists(routes_csv):
        routes = pd.read_csv(routes_csv).to_dict('records')

    # Color map
    cmap = plt.cm.get_cmap('gist_ncar', len(routes) + 5)

    # Plot
    fig, ax = plt.subplots(figsize=(15, 15))
    ax.grid(True, linestyle=':', alpha=0.4, color='lightgrey', zorder=0)

    # 1. Background (all occupied cells)
    ax.scatter(sparse_df["col"], sparse_df["row"], c='#e0e0e0', s=args.s, alpha=0.2, marker='s', label="Customer (unassigned)", zorder=1)

    # 2. Plot Unassigned specifically if we can distinguish them
    # For now, let's just plot the trips over the background.
    
    # 3. Trips
    centers_x, centers_y = [], []
    for i, rt in enumerate(routes):
        tokens = split_tokens(str(rt["member_cells"]))
        pts = []
        for t in tokens:
            try:
                lin = int(t)
                pts.append((lin % width, lin // width))
            except: pass
        
        if not pts: continue
        pts = np.array(pts)
        color = cmap(i)

        # Cluster cover (outline of union of grid cells)
        if not args.no_cover:
            try:
                loops = boundary_loops_from_cells([(p[0], p[1]) for p in pts])
                for lp in loops:
                    # Fill should be visible even with square markers -> draw ABOVE markers with gentle alpha
                    fc = (color[0], color[1], color[2], 0.14)
                    ec = (color[0], color[1], color[2], 0.85)
                    ax.add_patch(Polygon(lp, closed=True, facecolor=fc, edgecolor=ec, linewidth=1.6, zorder=4))
            except:
                pass

        # Plot markers
        ax.scatter(pts[:, 0], pts[:, 1], color=color, s=args.s, marker='s', edgecolors='none', alpha=0.9, zorder=3)
        
        # Collect center
        centers_x.append(rt["center_col"])
        centers_y.append(rt["center_row"])

        # Hull (optional)
        if args.show_hull and len(pts) >= 3:
            try:
                hull = ConvexHull(pts)
                hull_pts = np.vstack([pts[hull.vertices], pts[hull.vertices[0]]])
                ax.fill(hull_pts[:, 0], hull_pts[:, 1], color=color, alpha=0.1, zorder=2)
                ax.plot(hull_pts[:, 0], hull_pts[:, 1], color=color, linewidth=1, alpha=0.4, zorder=2)
            except: pass
    
    # 4. Route Centers (Black dots)
    ax.scatter(centers_x, centers_y, c='black', s=12, marker='o', edgecolors='white', linewidths=0.5, label="Route Center", zorder=5)

    # 5. Depots
    for d in depots:
        ax.scatter(d["col"], d["row"], marker="*", s=300, c='red', edgecolors="black", linewidths=1.5, label="Depot", zorder=10)
        ax.annotate(d["id"], (d["col"], d["row"]), fontsize=9, fontweight='bold', xytext=(5, 5), textcoords='offset points', bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8, edgecolor='grey'))

    # Aesthetic adjustments
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("col", fontsize=11)
    ax.set_ylabel("row", fontsize=11)
    ax.set_title(title, fontsize=15, fontweight='bold', pad=20)
    if args.invert_y: ax.invert_yaxis()

    # Legend
    legend_elements = [
        Line2D([0], [0], marker='s', color='w', label='Customer (by cluster)', markerfacecolor='blue', markersize=8),
        Line2D([0], [0], marker='o', color='w', label='Route Center', markerfacecolor='black', markeredgecolor='white', markersize=8),
        Line2D([0], [0], marker='*', color='w', label='Depot', markerfacecolor='red', markeredgecolor='black', markersize=12),
        Line2D([0], [0], marker='x', color='w', label='Unassigned', markerfacecolor='grey', markeredgecolor='grey', markersize=8),
    ]
    ax.legend(handles=legend_elements, loc="upper right", framealpha=0.9, fontsize=10)

    plt.tight_layout()
    plt.savefig(out_png, dpi=200)
    print(f"[OK] saved: {out_png}")

if __name__ == "__main__":
    main()
