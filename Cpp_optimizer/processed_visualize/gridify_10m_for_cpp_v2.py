"""
gridify_10m_for_cpp.py

Mục tiêu:
- Chuẩn hoá customers + depots từ lat/lon -> hệ lưới (grid) ô = CELL_M (m), mặc định 10m/ô.
- Xuất CSV "đủ thông số" để C++ đọc trực tiếp (row/col + các thông số khách/kho).
- Với CUSTOMERS: xuất thêm sparse grid CSV (KHÔNG xuất ảnh, KHÔNG numpy mặc định).
- Với DEPOTS: chỉ xuất CSV (KHÔNG ảnh / numpy).

Projection (copy y hệt DataLoader::compute_projection trong C++):
    lon2km = lon * 111.32 * cos(ref_lat)
    lat2km = lat * 110.57
    ref_lat = mean(lat của customers + depots)

Outputs (OUT_DIR):
  - customers_grid.csv          : giữ cột gốc + x_km,y_km,row,col + Territory_Depot + Customer_ID_idx
  - depots_grid_min.csv         : Depot_ID,row,col,Capacity_Storage
  - customers_sparse_grid.csv   : one row / occupied cell:
        row,col,weight_sum,volume_sum,priority_max,count,customer_id,customer_id_idx
        * nếu ô bị đè (>=2 khách): weight/volume = tổng; priority = MAX; customer_id = "ID1 ID2 ..." (cách nhau dấu cách)
  - customer_id_map.csv         : Customer_ID_idx, Customer_ID
  - grid_meta.txt               : tham số lưới để C++ đọc đồng bộ

Ghi chú:
- customers_sparse_grid.csv có cột string `customer_id` (các ID cách nhau bằng dấu cách) để bạn trace/debug ô bị đè.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
import numpy as np
import pandas as pd

# ===================== CONFIG =====================
CUSTOMERS_CSV = r"D:\A UEH_UNIVERSITY\RESEACH\OR_AI_paper\Zzz_data\LMDO processed\Ho_Chi_Minh_City\customers__Sheet1.csv"
DEPOTS_CSV    = r"D:\A UEH_UNIVERSITY\RESEACH\OR_AI_paper\Zzz_data\LMDO processed\Ho_Chi_Minh_City\depots__Sheet1.csv"
OUT_DIR       = r"Cpp_optimizer\end_to_end\grid_out"

CELL_M = 10.0  # 10m / cell

# User yêu cầu: không xuất customers_grid_10m.npy (numpy). Nếu cần bật lại thì set True.
SAVE_CUSTOMER_NPY = False

# ===================== UTILS =====================

def _safe_mkdir(p: str | Path) -> Path:
    out = Path(p)
    out.mkdir(parents=True, exist_ok=True)
    return out


def _require_cols(df: pd.DataFrame, cols: list[str], name: str) -> None:
    missing = [c for c in cols if c not in df.columns]
    if missing:
        raise ValueError(f"{name} missing columns: {missing}. Existing: {list(df.columns)}")


def _to_num(df: pd.DataFrame, cols: list[str]) -> None:
    for c in cols:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors="coerce")


@dataclass
class GridMeta:
    ref_lat: float
    cos_ref: float
    min_lat: float
    min_lon: float
    cell_m: float
    cell_km: float
    x_km_min: float
    y_km_min: float
    width: int
    height: int


def _lon2km(lon: np.ndarray, cos_ref: float) -> np.ndarray:
    return lon * 111.32 * cos_ref


def _lat2km(lat: np.ndarray) -> np.ndarray:
    return lat * 110.57


def _compute_meta(customers: pd.DataFrame, depots: pd.DataFrame, cell_m: float) -> GridMeta:
    all_lat = pd.concat([customers["Latitude"], depots["Latitude"]], ignore_index=True).to_numpy(dtype=float)
    all_lon = pd.concat([customers["Longitude"], depots["Longitude"]], ignore_index=True).to_numpy(dtype=float)

    ref_lat = float(np.mean(all_lat))
    cos_ref = float(math.cos(math.radians(ref_lat)))

    min_lat = float(np.min(all_lat))
    min_lon = float(np.min(all_lon))

    cell_km = float(cell_m) / 1000.0

    x_km_all = _lon2km(all_lon, cos_ref)
    y_km_all = _lat2km(all_lat)

    x_km_min = float(_lon2km(np.array([min_lon], dtype=float), cos_ref)[0])
    y_km_min = float(_lat2km(np.array([min_lat], dtype=float))[0])

    rel_x = x_km_all - x_km_min
    rel_y = y_km_all - y_km_min

    width = int(np.floor(np.max(rel_x) / cell_km) + 1)
    height = int(np.floor(np.max(rel_y) / cell_km) + 1)

    return GridMeta(
        ref_lat=ref_lat,
        cos_ref=cos_ref,
        min_lat=min_lat,
        min_lon=min_lon,
        cell_m=float(cell_m),
        cell_km=cell_km,
        x_km_min=x_km_min,
        y_km_min=y_km_min,
        width=width,
        height=height,
    )


def _add_grid_cols(df: pd.DataFrame, meta: GridMeta) -> pd.DataFrame:
    out = df.copy()
    lon = out["Longitude"].to_numpy(dtype=float)
    lat = out["Latitude"].to_numpy(dtype=float)

    x_km = _lon2km(lon, meta.cos_ref)
    y_km = _lat2km(lat)

    rel_x = x_km - meta.x_km_min
    rel_y = y_km - meta.y_km_min

    col = np.floor(rel_x / meta.cell_km).astype(np.int64)
    row = np.floor(rel_y / meta.cell_km).astype(np.int64)

    out["x_km"] = x_km
    out["y_km"] = y_km
    out["row"] = row
    out["col"] = col
    return out


def _assign_nearest_depot(customers_g: pd.DataFrame, depots_g: pd.DataFrame) -> np.ndarray:
    # nearest theo (row,col)
    d_rows = depots_g["row"].to_numpy(dtype=np.int64)
    d_cols = depots_g["col"].to_numpy(dtype=np.int64)
    d_ids = depots_g["Depot_ID"].astype(str).to_numpy()

    c_rows = customers_g["row"].to_numpy(dtype=np.int64)
    c_cols = customers_g["col"].to_numpy(dtype=np.int64)

    # broadcast distance^2
    dr = c_rows[:, None] - d_rows[None, :]
    dc = c_cols[:, None] - d_cols[None, :]
    d2 = dr * dr + dc * dc
    argmin = np.argmin(d2, axis=1)
    return d_ids[argmin]


def _make_customer_id_map(customers_g: pd.DataFrame, out_dir: Path) -> tuple[pd.DataFrame, dict[str, int]]:
    uniq = sorted(customers_g["Customer_ID"].astype(str).unique().tolist())
    id2idx = {cid: i for i, cid in enumerate(uniq)}
    map_df = pd.DataFrame({"Customer_ID_idx": list(range(len(uniq))), "Customer_ID": uniq})
    map_df.to_csv(out_dir / "customer_id_map.csv", index=False, encoding="utf-8-sig")
    return map_df, id2idx


def _first_two_indices(id_list: list[int]) -> tuple[int, int]:
    if not id_list:
        return -1, -1
    if len(id_list) == 1:
        return int(id_list[0]), -1
    return int(id_list[0]), int(id_list[1])


def _build_sparse_and_grid(customers_g: pd.DataFrame, meta: GridMeta, out_dir: Path, save_npy: bool = False) -> None:
    """
    customers_sparse_grid.csv:
        row,col,weight_sum,volume_sum,priority_max,count,customer_id,customer_id_idx

    (optional) customers_grid_10m.npy nếu save_npy=True:
        (H,W,6) float32: [weight_sum, volume_sum, priority_max, count, id1_idx, id2_idx]
    """
    # đảm bảo các cột numeric tồn tại
    if "Order_Weight" not in customers_g.columns:
        customers_g["Order_Weight"] = 0.0
    if "Order_Volume" not in customers_g.columns:
        customers_g["Order_Volume"] = 0.0
    if "Priority_Level" not in customers_g.columns:
        customers_g["Priority_Level"] = 0.0

    # group theo cell
    g = customers_g.groupby(["row", "col"], sort=False)

    sparse = g.agg(
        weight_sum=("Order_Weight", "sum"),
        volume_sum=("Order_Volume", "sum"),
        priority_max=("Priority_Level", "max"),
        count=("Customer_ID", "size"),
    ).reset_index()

    # join customer_id string (cách nhau dấu cách)
    ids_join = g["Customer_ID"].apply(lambda s: " ".join(map(str, s.tolist()))).reset_index(name="customer_id")
    idx_join = g["Customer_ID_idx"].apply(lambda s: " ".join(map(str, s.tolist()))).reset_index(name="customer_id_idx")

    sparse = sparse.merge(ids_join, on=["row", "col"], how="left")
    sparse = sparse.merge(idx_join, on=["row", "col"], how="left")

    # xuất sparse CSV
    sparse.to_csv(out_dir / "customers_sparse_grid.csv", index=False, encoding="utf-8-sig", float_format="%.6f")

    if not save_npy:
        return

    # build numpy grid
    H, W = meta.height, meta.width
    grid = np.zeros((H, W, 6), dtype=np.float32)

    # id1/id2 theo list idx
    id_lists = g["Customer_ID_idx"].apply(list).reset_index(name="id_list")
    id12 = id_lists["id_list"].apply(_first_two_indices)
    id_lists["id1_idx"] = id12.apply(lambda t: t[0])
    id_lists["id2_idx"] = id12.apply(lambda t: t[1])

    # merge để align row/col
    m = sparse.merge(id_lists[["row", "col", "id1_idx", "id2_idx"]], on=["row", "col"], how="left")

    rows = m["row"].to_numpy(dtype=np.int64)
    cols = m["col"].to_numpy(dtype=np.int64)

    grid[rows, cols, 0] = m["weight_sum"].to_numpy(dtype=np.float32)
    grid[rows, cols, 1] = m["volume_sum"].to_numpy(dtype=np.float32)
    grid[rows, cols, 2] = m["priority_max"].to_numpy(dtype=np.float32)
    grid[rows, cols, 3] = m["count"].to_numpy(dtype=np.float32)
    grid[rows, cols, 4] = m["id1_idx"].fillna(-1).to_numpy(dtype=np.float32)
    grid[rows, cols, 5] = m["id2_idx"].fillna(-1).to_numpy(dtype=np.float32)

    np.save(out_dir / "customers_grid_10m.npy", grid)


def _write_meta(meta: GridMeta, out_dir: Path, n_customers: int, n_depots: int) -> None:
    lines = [
        f"cell_m={meta.cell_m}",
        f"cell_km={meta.cell_km}",
        f"ref_lat={meta.ref_lat}",
        f"cos_ref={meta.cos_ref}",
        f"min_lat={meta.min_lat}",
        f"min_lon={meta.min_lon}",
        f"x_km_min={meta.x_km_min}",
        f"y_km_min={meta.y_km_min}",
        f"width={meta.width}",
        f"height={meta.height}",
        f"customers={n_customers}",
        f"depots={n_depots}",
    ]
    (out_dir / "grid_meta.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    out_dir = _safe_mkdir(OUT_DIR)

    cus_path = Path(CUSTOMERS_CSV)
    dep_path = Path(DEPOTS_CSV)
    if not cus_path.exists():
        raise FileNotFoundError(f"CUSTOMERS_CSV not found: {cus_path}")
    if not dep_path.exists():
        raise FileNotFoundError(f"DEPOTS_CSV not found: {dep_path}")

    customers = pd.read_csv(cus_path)
    depots = pd.read_csv(dep_path)

    _require_cols(customers, ["Customer_ID", "Latitude", "Longitude"], "customers")
    _require_cols(depots, ["Depot_ID", "Latitude", "Longitude"], "depots")

    # numeric conversions
    _to_num(customers, ["Latitude", "Longitude", "Order_Weight", "Order_Volume", "Priority_Level"])
    _to_num(depots, ["Latitude", "Longitude", "Capacity_Storage"])

    # drop rows thiếu lat/lon
    customers = customers.dropna(subset=["Latitude", "Longitude"]).reset_index(drop=True)
    depots = depots.dropna(subset=["Latitude", "Longitude"]).reset_index(drop=True)
    if len(customers) == 0:
        raise ValueError("customers is empty after dropping NaN lat/lon")
    if len(depots) == 0:
        raise ValueError("depots is empty after dropping NaN lat/lon")

    meta = _compute_meta(customers, depots, CELL_M)

    cus_g = _add_grid_cols(customers, meta)
    dep_g = _add_grid_cols(depots, meta)

    # map Customer_ID -> idx
    _, id2idx = _make_customer_id_map(cus_g, out_dir)
    cus_g["Customer_ID_idx"] = cus_g["Customer_ID"].astype(str).map(id2idx).astype(int)

    # Territory: nearest depot
    cus_g["Territory_Depot"] = _assign_nearest_depot(cus_g, dep_g)

    # write row-wise CSV for C++
    cus_g.to_csv(out_dir / "customers_grid.csv", index=False, encoding="utf-8-sig", float_format="%.6f")


    # depots minimal theo yêu cầu: chỉ giữ Depot_ID,row,col,Capacity_Storage
    depot_cols = [c for c in ["Depot_ID", "row", "col", "Capacity_Storage"] if c in dep_g.columns]
    dep_g[depot_cols].to_csv(out_dir / "depots_grid_min.csv", index=False, encoding="utf-8-sig", float_format="%.6f")


    # customers sparse + numpy
    _build_sparse_and_grid(cus_g, meta, out_dir, save_npy=SAVE_CUSTOMER_NPY)

    # meta
    _write_meta(meta, out_dir, n_customers=len(cus_g), n_depots=len(dep_g))

    print("[OK] Wrote:")
    print(" -", out_dir / "customers_grid.csv")
    print(" -", out_dir / "depots_grid_min.csv")
    print(" -", out_dir / "customers_sparse_grid.csv")
    if SAVE_CUSTOMER_NPY:
        print(" -", out_dir / "customers_grid_10m.npy")
    print(" -", out_dir / "customer_id_map.csv")
    print(" -", out_dir / "grid_meta.txt")


if __name__ == "__main__":
    main()
