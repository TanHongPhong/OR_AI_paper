
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

# =============================================================
# CONFIGURATION
# =============================================================
GRID_DATA_DIR   = r"D:\A UEH_UNIVERSITY\RESEACH\OR_AI_paper\Cpp_optimizer\end_to_end\grid_out"
BASE_OUTPUT_DIR = r"D:\A UEH_UNIVERSITY\RESEACH\OR_AI_paper\Cpp_optimizer\_cai_nay_chac_an_nay\output"

def get_latest_run_dir(base_dir):
    runs = [d for d in os.listdir(base_dir) if d.startswith("run_") and os.path.isdir(os.path.join(base_dir, d))]
    if not runs:
        return None
    return os.path.join(base_dir, sorted(runs)[-1])

LATEST_RUN = get_latest_run_dir(BASE_OUTPUT_DIR)
if LATEST_RUN:
    RUN_RESULT_DIR = LATEST_RUN
    print(f"[INFO] Auto-detected latest run: {RUN_RESULT_DIR}")
else:
    # Fallback to shared grid_out if no runs found
    RUN_RESULT_DIR = GRID_DATA_DIR
    print(f"[WARNING] No run_* folders found. Falling back to {GRID_DATA_DIR}.")

OUTPUT_PNG   = os.path.join(BASE_OUTPUT_DIR, "viz_latest.png")
SHOW_HULL    = True    # Enable cluster boundaries
MARKER_SIZE  = 4.0
INVERT_Y     = True

def split_tokens(s: str):
    if s is None: return []
    s = str(s).strip()
    if not s: return []
    return re.split(r"\s+", s)

def _is_collinear(p0, p1, p2):
    return (p0[0] == p1[0] == p2[0]) or (p0[1] == p1[1] == p2[1])

def _simplify_loop(loop):
    if len(loop) < 4: return loop
    out = []
    n = len(loop)
    for i in range(n):
        p_prev = loop[i - 1]
        p = loop[i]
        p_next = loop[(i + 1) % n]
        if _is_collinear(p_prev, p, p_next): continue
        out.append(p)
    return out if len(out) >= 3 else loop

def boundary_loops_from_cells(cells):
    cell_set = set((int(x), int(y)) for x, y in cells)
    if not cell_set: return []
    out_edges = {}
    used = set()
    def add_dir(a, b): out_edges.setdefault(a, []).append(b)
    for x, y in cell_set:
        tl, tr = (x - 0.5, y - 0.5), (x + 0.5, y - 0.5)
        br, bl = (x + 0.5, y + 0.5), (x - 0.5, y + 0.5)
        if (x, y - 1) not in cell_set: add_dir(tl, tr)
        if (x + 1, y) not in cell_set: add_dir(tr, br)
        if (x, y + 1) not in cell_set: add_dir(br, bl)
        if (x - 1, y) not in cell_set: add_dir(bl, tl)

    def vdir(a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        return (int(np.sign(dx)) if dx != 0 else 0, int(np.sign(dy)) if dy != 0 else 0)

    right_turn = {(1, 0): (0, 1), (0, 1): (-1, 0), (-1, 0): (0, -1), (0, -1): (1, 0)}
    left_turn  = {(1, 0): (0, -1), (0, 1): (1, 0), (-1, 0): (0, 1), (0, -1): (-1, 0)}
    back_turn  = {(1, 0): (-1, 0), (0, 1): (0, -1), (-1, 0): (1, 0), (0, -1): (0, 1)}

    def pick_next(cur, incoming_dir):
        cands = out_edges.get(cur, [])
        if not cands: return None
        dir2ends = {}
        for e in cands:
            if (cur, e) in used: continue
            d = vdir(cur, e)
            dir2ends.setdefault(d, []).append(e)
        if not dir2ends: return None
        for d in (right_turn[incoming_dir], incoming_dir, left_turn[incoming_dir], back_turn[incoming_dir]):
            if d in dir2ends: return sorted(dir2ends[d])[0]
        return sorted(dir2ends[sorted(dir2ends.keys())[0]])[0]

    loops = []
    all_edges = [(a, b) for a, outs in out_edges.items() for b in outs]
    for a, b in all_edges:
        if (a, b) in used: continue
        loop, start = [a, b], a
        used.add((a, b))
        prev, cur = a, b
        incoming, guard = vdir(prev, cur), 0
        while guard < 200000:
            guard += 1
            nxt = pick_next(cur, incoming)
            if nxt is None: break
            used.add((cur, nxt))
            loop.append(nxt)
            if nxt == start:
                loop = _simplify_loop(loop[:-1])
                if len(loop) >= 3: loops.append(loop)
                break
            prev, cur, incoming = cur, nxt, vdir(cur, nxt)
    return loops

def load_depots(depots_csv: str):
    df = pd.read_csv(depots_csv)
    did_col = "Depot_ID" if "Depot_ID" in df.columns else "depot_id"
    return [{"id": str(r[did_col]), "row": int(r["row"]), "col": int(r["col"])} for _, r in df.iterrows()]

def main():
    print(f"[INFO] Static Data Dir: {GRID_DATA_DIR}")
    print(f"[INFO] Run Results Dir: {RUN_RESULT_DIR}")
    sparse_csv = os.path.join(GRID_DATA_DIR, "customers_sparse_grid.csv")
    depots_csv = os.path.join(GRID_DATA_DIR, "depots_grid_min.csv")
    routes_csv = os.path.join(RUN_RESULT_DIR, "alns_routes.csv")
    summary_csv = os.path.join(RUN_RESULT_DIR, "alns_summary.csv")
    meta_txt = os.path.join(GRID_DATA_DIR, "grid_meta.txt")

    sparse_df = pd.read_csv(sparse_csv)
    sparse_df.columns = [c.lower() for c in sparse_df.columns]
    depots = load_depots(depots_csv)

    width = 0
    if os.path.exists(meta_txt):
        with open(meta_txt, 'r') as f:
            for line in f:
                if line.startswith("width="): width = int(line.split("=")[1]); break
    if width <= 0: width = int(sparse_df["col"].max() + 1)

    stats = {}
    if os.path.exists(summary_csv):
        sdf = pd.read_csv(summary_csv)
        if "metric" in sdf.columns: stats = dict(zip(sdf['metric'], sdf['value']))

    title = f"ALNS Solution: {int(stats.get('trips', 0))} Trips, {int(stats.get('assigned_cells', 0))} Served"
    routes = pd.read_csv(routes_csv).to_dict('records') if os.path.exists(routes_csv) else []
    cmap = plt.cm.get_cmap('gist_ncar', len(routes) + 5)

    fig, ax = plt.subplots(figsize=(15, 15))
    ax.grid(True, linestyle=':', alpha=0.4, color='lightgrey')
    ax.scatter(sparse_df["col"], sparse_df["row"], c='#e0e0e0', s=MARKER_SIZE, alpha=0.2, marker='s', label="Unassigned")

    centers_x, centers_y = [], []
    for i, rt in enumerate(routes):
        tokens = split_tokens(str(rt.get("member_cells", "")))
        pts = []
        for t in tokens:
            try: lin = int(t); pts.append((lin % width, lin // width))
            except: pass
        if not pts: continue
        pts = np.asarray(pts, dtype=float)
        color = cmap(i)

        # Boundary cover logic
        loops = boundary_loops_from_cells([(p[0], p[1]) for p in pts])
        for lp in loops:
            xs, ys = [p[0] for p in lp] + [lp[0][0]], [p[1] for p in lp] + [lp[0][1]]
            ax.add_patch(Polygon(lp, closed=True, facecolor=(color[0], color[1], color[2], 0.10), edgecolor='none', zorder=2.1))
            ax.plot(xs, ys, color='white', linewidth=3.6, alpha=0.95, zorder=6)
            ax.plot(xs, ys, color=(color[0], color[1], color[2], 0.98), linewidth=2.2, alpha=0.98, zorder=7)

        ax.scatter(pts[:, 0], pts[:, 1], color=color, s=MARKER_SIZE, marker='s', alpha=0.9, zorder=3)
        if "center_col" in rt: centers_x.append(rt["center_col"]); centers_y.append(rt["center_row"])

        if SHOW_HULL and len(pts) >= 3:
            try:
                hull = ConvexHull(pts)
                hull_pts = np.vstack([pts[hull.vertices], pts[hull.vertices[0]]])
                ax.fill(hull_pts[:, 0], hull_pts[:, 1], color=color, alpha=0.1, zorder=2)
                ax.plot(hull_pts[:, 0], hull_pts[:, 1], color=color, linewidth=1, alpha=0.4, zorder=2)
            except: pass

    if centers_x: ax.scatter(centers_x, centers_y, c='black', s=12, marker='o', edgecolors='white', linewidths=0.5, zorder=5)
    for d in depots:
        ax.scatter(d["col"], d["row"], marker="*", s=300, c='red', edgecolors="black", linewidths=1.5, zorder=10)
        ax.annotate(d["id"], (d["col"], d["row"]), fontsize=9, fontweight='bold', xytext=(5, 5), textcoords='offset points', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

    ax.set_aspect("equal")
    ax.set_title(title, fontsize=15, fontweight='bold')
    if INVERT_Y: ax.invert_yaxis()
    plt.tight_layout()
    plt.savefig(OUTPUT_PNG, dpi=200)
    print(f"[OK] saved: {OUTPUT_PNG}")

if __name__ == "__main__":
    main()
