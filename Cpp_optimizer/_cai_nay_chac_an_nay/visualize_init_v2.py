
import argparse
import os
import re
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
    """
    Trace outer boundaries of union of grid cells.
    """
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

        if (x, y - 1) not in cell_set:   # top exposed
            add_dir(tl, tr)              # E
        if (x + 1, y) not in cell_set:   # right exposed
            add_dir(tr, br)              # S
        if (x, y + 1) not in cell_set:   # bottom exposed
            add_dir(br, bl)              # W
        if (x - 1, y) not in cell_set:   # left exposed
            add_dir(bl, tl)              # N

    def vdir(a, b):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        if dx != 0:
            dx = int(np.sign(dx))
        if dy != 0:
            dy = int(np.sign(dy))
        return (dx, dy)

    # Right-hand rule in y-down coordinates
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
        # fallback
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
    depots = []
    for _, r in df.iterrows():
        depots.append({"id": str(r[did_col]), "row": int(r["row"]), "col": int(r["col"])})
    return depots

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--grid_out", required=True, help="Folder containing INIT output CSVs (assignment/routes)")
    ap.add_argument("--data_dir", default="", help="Folder containing STATIC grid data (sparse grid/depots). Defaults to grid_out if empty.")
    ap.add_argument("--routes", default="init_routes.csv", help="Route details (trip-based)")
    ap.add_argument("--summary", default="init_summary.csv", help="Summary stats")
    ap.add_argument("--out_png", default="viz_init_v2.png")
    ap.add_argument("--s", type=float, default=4.0, help="Customer marker size")
    ap.add_argument("--show_hull", action="store_true", help="Draw convex hull boundaries")
    ap.add_argument("--no_cover", action="store_true", help="Disable filled boundary cover per cluster")
    ap.add_argument("--invert_y", action="store_true", default=True, help="Invert Y axis")
    args = ap.parse_args()

    data_dir = args.data_dir if args.data_dir else args.grid_out

    sparse_csv = os.path.join(data_dir, "customers_sparse_grid.csv")
    depots_csv = os.path.join(data_dir, "depots_grid_min.csv")
    meta_txt = os.path.join(data_dir, "grid_meta.txt")

    routes_csv = os.path.join(args.grid_out, args.routes)
    summary_csv = os.path.join(args.grid_out, args.summary)
    out_png = os.path.join(args.grid_out, args.out_png)

    print(f"Reading data from: {data_dir}")
    print(f"Reading results from: {args.grid_out}")

    sparse_df = pd.read_csv(sparse_csv)
    sparse_df.columns = [c.lower() for c in sparse_df.columns]
    depots = load_depots(depots_csv)

    width = 0
    if os.path.exists(meta_txt):
        with open(meta_txt, 'r') as f:
            for line in f:
                if line.startswith("width="):
                    width = int(line.split("=")[1]); break
    if width <= 0:
        width = int(sparse_df["col"].max() + 1)

    stats = {}
    if os.path.exists(summary_csv):
        sdf = pd.read_csv(summary_csv)
        if "metric" in sdf.columns and "value" in sdf.columns:
            stats = dict(zip(sdf['metric'], sdf['value']))

    title = f"Init Solution: {int(float(stats.get('trips', 0)))} Trips, {int(float(stats.get('assigned_cells', 0)))} Served, {int(float(stats.get('unassigned_cells', 0)))} Unserved"

    routes = []
    if os.path.exists(routes_csv):
        routes = pd.read_csv(routes_csv).to_dict('records')

    cmap = plt.cm.get_cmap('gist_ncar', len(routes) + 5)

    fig, ax = plt.subplots(figsize=(15, 15))
    ax.grid(True, linestyle=':', alpha=0.4, color='lightgrey', zorder=0)

    # Background
    ax.scatter(sparse_df["col"], sparse_df["row"], c='#e0e0e0', s=args.s, alpha=0.2,
               marker='s', label="Customer (unassigned)", zorder=1)

    centers_x, centers_y = [], []
    for i, rt in enumerate(routes):
        tokens = split_tokens(str(rt.get("member_cells", "")))
        pts = []
        for t in tokens:
            try:
                lin = int(t)
                pts.append((lin % width, lin // width))  # (x,y)
            except:
                pass

        if not pts:
            continue

        pts = np.asarray(pts, dtype=float)
        color = cmap(i)

        # --- Boundary cover + explicit outline (this is the NEW part) ---
        if not args.no_cover:
            pts_tuples = [(p[0], p[1]) for p in pts]
            loops = boundary_loops_from_cells(pts_tuples)
            if i < 5: # Debug print for first few clusters
                print(f"  Cluster {i}: {len(pts)} points -> {len(loops)} loops")
            
            for lp in loops:
                xs = [p[0] for p in lp] + [lp[0][0]]
                ys = [p[1] for p in lp] + [lp[0][1]]

                # fill
                fc = (color[0], color[1], color[2], 0.10)
                ax.add_patch(Polygon(lp, closed=True, facecolor=fc, edgecolor='none', zorder=2.1))

                # outline with a white underlay for contrast
                ax.plot(xs, ys, color='white', linewidth=3.6, alpha=0.95, zorder=6)
                ec = (color[0], color[1], color[2], 0.98)
                ax.plot(xs, ys, color=ec, linewidth=2.2, alpha=0.98, zorder=7)

        # Cluster markers (unchanged)
        ax.scatter(pts[:, 0], pts[:, 1], color=color, s=args.s, marker='s',
                   edgecolors='none', alpha=0.9, zorder=3)

        # Centers
        if "center_col" in rt and "center_row" in rt:
            centers_x.append(rt["center_col"])
            centers_y.append(rt["center_row"])

        # Hull optional (unchanged)
        if args.show_hull and len(pts) >= 3:
            try:
                hull = ConvexHull(pts)
                hull_pts = np.vstack([pts[hull.vertices], pts[hull.vertices[0]]])
                ax.fill(hull_pts[:, 0], hull_pts[:, 1], color=color, alpha=0.1, zorder=2)
                ax.plot(hull_pts[:, 0], hull_pts[:, 1], color=color, linewidth=1, alpha=0.4, zorder=2)
            except:
                pass

    # Route centers
    if centers_x and centers_y:
        ax.scatter(centers_x, centers_y, c='black', s=12, marker='o',
                   edgecolors='white', linewidths=0.5, label="Route Center", zorder=5)

    # Depots (unchanged)
    for d in depots:
        ax.scatter(d["col"], d["row"], marker="*", s=300, c='red',
                   edgecolors="black", linewidths=1.5, label="Depot", zorder=10)
        ax.annotate(d["id"], (d["col"], d["row"]), fontsize=9, fontweight='bold',
                    xytext=(5, 5), textcoords='offset points',
                    bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8, edgecolor='grey'))

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("col", fontsize=11)
    ax.set_ylabel("row", fontsize=11)
    ax.set_title(title, fontsize=15, fontweight='bold', pad=20)
    if args.invert_y:
        ax.invert_yaxis()

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
