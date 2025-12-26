import argparse
import os
import re
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-GUI backend
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

def split_tokens(s: str):
    if s is None:
        return []
    s = str(s).strip()
    if not s:
        return []
    # tách theo khoảng trắng (nhiều space ok)
    return re.split(r"\s+", s)

def build_id_to_cell(grid_dir: str):
    """
    Build mapping from customer_id -> (row, col).
    Uses customer_id_map.csv to get customer_id, and customers_sparse_grid.csv for coordinates.
    """
    # Load customer_id_map.csv to get customer_id -> customer_id_idx mapping
    id_map_csv = os.path.join(grid_dir, "customer_id_map.csv")
    sparse_csv = os.path.join(grid_dir, "customers_sparse_grid.csv")
    
    id_map_df = pd.read_csv(id_map_csv)
    sparse_df = pd.read_csv(sparse_csv)
    
    # Lowercase all column names for consistency
    id_map_df.columns = [c.lower() for c in id_map_df.columns]
    sparse_df.columns = [c.lower() for c in sparse_df.columns]
    
    # Build idx -> (row, col) from sparse_grid
    # Each row in sparse_grid has customer_id_idx which may contain multiple space-separated indices
    idx_to_rc = {}
    for _, r in sparse_df.iterrows():
        rr = int(r["row"])
        cc = int(r["col"])
        if "customer_id_idx" in r:
            tokens = split_tokens(r["customer_id_idx"])
            for t in tokens:
                try:
                    idx = int(t)
                    idx_to_rc[idx] = (rr, cc)
                except:
                    pass
    
    # Build customer_id -> (row, col) using id_map
    id_to_rc = {}
    for _, r in id_map_df.iterrows():
        cust_id = str(r.get("customer_id", "")).strip()
        try:
            idx = int(r.get("customer_id_idx", -1))
        except:
            continue
        if idx in idx_to_rc and cust_id:
            id_to_rc[cust_id] = idx_to_rc[idx]
    
    return id_to_rc

def load_depots(depots_csv: str):
    df = pd.read_csv(depots_csv)
    # cố gắng đoán cột
    depot_id_col = "Depot_ID" if "Depot_ID" in df.columns else ("depot_id" if "depot_id" in df.columns else None)
    if depot_id_col is None:
        raise ValueError(f"depots file must have Depot_ID (or depot_id). Found: {list(df.columns)}")
    if "row" not in df.columns or "col" not in df.columns:
        raise ValueError(f"depots file must have row,col. Found: {list(df.columns)}")

    depots = []
    for _, r in df.iterrows():
        depots.append({
            "depot_id": str(r[depot_id_col]),
            "row": int(r["row"]),
            "col": int(r["col"]),
        })
    return depots

def read_assignment(assign_csv: str):
    df = pd.read_csv(assign_csv)

    # kỳ vọng 2 cột: customer_id_idx,depot_id
    # nhưng vẫn linh hoạt
    cust_col = None
    for c in ["customer_id_idx", "customer_id", "customer", "id"]:
        if c in df.columns:
            cust_col = c
            break
    if cust_col is None:
        raise ValueError(f"assignment file must have a customer column. Found: {list(df.columns)}")

    depot_col = None
    for c in ["depot_id", "Depot_ID", "depot", "Depot"]:
        if c in df.columns:
            depot_col = c
            break
    if depot_col is None:
        raise ValueError(f"assignment file must have a depot column. Found: {list(df.columns)}")

    df = df[[cust_col, depot_col]].copy()
    df.columns = ["customer_key", "depot_id"]
    df["customer_key"] = df["customer_key"].astype(str).str.strip()
    df["depot_id"] = df["depot_id"].astype(str).str.strip()
    return df

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--grid_out", required=True, help="folder chứa customers_sparse_grid.csv + depots_grid_min.csv + output assignment")
    ap.add_argument("--customers", default="customers_sparse_grid.csv")
    ap.add_argument("--depots", default="depots_grid_min.csv")
    ap.add_argument("--assign", default="customers_to_nearest_depot.csv")
    ap.add_argument("--out_png", default="viz_assignment.png")
    ap.add_argument("--s", type=float, default=6.0, help="marker size")
    ap.add_argument("--invert_y", action="store_true", help="invert Y axis (giống ma trận ảnh)")
    ap.add_argument("--show", action="store_true", help="show plot window")
    args = ap.parse_args()

    customers_csv = os.path.join(args.grid_out, args.customers)
    depots_csv = os.path.join(args.grid_out, args.depots)
    assign_csv = os.path.join(args.grid_out, args.assign)
    out_png = os.path.join(args.grid_out, args.out_png) if not os.path.isabs(args.out_png) else args.out_png

    id_to_rc = build_id_to_cell(args.grid_out)
    depots = load_depots(depots_csv)
    asg = read_assignment(assign_csv)

    # map depot_id -> color index
    depot_ids = sorted(asg["depot_id"].unique().tolist())
    depot_to_idx = {d:i for i,d in enumerate(depot_ids)}

    xs, ys, cs = [], [], []
    missing = 0

    for _, r in asg.iterrows():
        key = r["customer_key"]
        dep = r["depot_id"]

        # nếu output của bạn dùng fallback "row_col" thì parse được luôn
        if key in id_to_rc:
            rr, cc = id_to_rc[key]
        else:
            m = re.match(r"^(\d+)[,_](\d+)$", key)  # "row_col" hoặc "row,col"
            if m:
                rr, cc = int(m.group(1)), int(m.group(2))
            else:
                missing += 1
                continue

        xs.append(cc)
        ys.append(rr)
        cs.append(depot_to_idx.get(dep, -1))

    xs = np.array(xs)
    ys = np.array(ys)
    cs = np.array(cs)

    plt.figure(figsize=(10, 10))
    sc = plt.scatter(xs, ys, c=cs, s=args.s)

    # plot depots
    for d in depots:
        plt.scatter([d["col"]], [d["row"]], marker="X", s=180, edgecolors="k", linewidths=1.0)

    plt.gca().set_aspect("equal", adjustable="box")
    plt.xlabel("col")
    plt.ylabel("row")
    plt.title(f"Customers colored by nearest depot (N={len(xs)}, missing={missing})")

    if args.invert_y:
        plt.gca().invert_yaxis()

    # Legend theo depot_id (không phụ thuộc màu cụ thể)
    handles = []
    for dep_id in depot_ids:
        idx = depot_to_idx[dep_id]
        handles.append(Line2D([0], [0], marker='o', linestyle='',
                              markerfacecolor=sc.cmap(sc.norm(idx)),
                              markersize=8, label=f"Depot {dep_id}"))
    # marker cho depot
    handles.append(Line2D([0], [0], marker='X', linestyle='',
                          markerfacecolor='none', markeredgecolor='k',
                          markersize=10, label="Depot location"))
    plt.legend(handles=handles, loc="best", framealpha=0.9)

    plt.tight_layout()
    plt.savefig(out_png, dpi=200)
    print(f"[OK] saved: {out_png}")
    if missing > 0:
        print(f"[WARN] {missing} customers in assignment not found in customers_sparse_grid mapping.")
    if args.show:
        plt.show()

if __name__ == "__main__":
    main()
