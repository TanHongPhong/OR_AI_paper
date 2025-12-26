# make_rgb_30m.py
# Create one RGB image ~30M pixels from customers CSV.
# R = Order_Weight (direct, clipped 0-255)
# G = Order_Volume (direct, clipped 0-255)
# B = Priority_Level (int clipped to 0-255)

# Outputs in OUT_DIR:
#  - rgb_30M.png
#  - rgb_30M_uint8.npy
#  - sparse_30M.csv (row,col,weight,volume,priority)
#  - metadata_30M.txt

from pathlib import Path
import pandas as pd
import numpy as np
from PIL import Image
import time, sys, traceback

# ====== CONFIGURE (already provided) ======
INPUT_CSV = r"D:\A UEH_UNIVERSITY\RESEACH\OR_AI_paper\Zzz_data\LMDO processed\Ho_Chi_Minh_City\customers__Sheet1.csv"
OUT_DIR   = r"D:\A UEH_UNIVERSITY\RESEACH\OR_AI_paper\Zzz_data\image_csv_out"
RESOLUTION = 4.381e-05   # deg per pixel (approx for ~30M pixels)
SAVE_NPY = True          # save uint8 array too
# ==========================================

def now(): return time.strftime("%Y-%m-%d %H:%M:%S")
def info(msg): print(f"[{now()}] {msg}")

def to_pixel_coord(val, res):
    return int(round(float(val) / res))

def safe_mkdir(p):
    Path(p).mkdir(parents=True, exist_ok=True)

def main():
    t0 = time.time()
    info("Start make_rgb_30m.py")
    out_dir = Path(OUT_DIR); safe_mkdir(out_dir)

    if not Path(INPUT_CSV).exists():
        info(f"ERROR: input CSV not found: {INPUT_CSV}")
        return

    # load csv
    try:
        info(f"Loading CSV: {INPUT_CSV}")
        df = pd.read_csv(INPUT_CSV)
    except Exception as e:
        info("ERROR reading CSV:")
        traceback.print_exc()
        return

    info(f"Rows loaded: {len(df)}")

    # ensure numeric cols
    for c in ['Latitude','Longitude','Order_Weight','Order_Volume','Priority_Level']:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors='coerce')

    # drop invalid coords
    df = df.dropna(subset=['Latitude','Longitude']).reset_index(drop=True)
    info(f"Rows with valid coords: {len(df)}")
    if df.empty:
        info("No valid coordinates. Exiting.")
        return

    # compute pixel indices using RESOLUTION
    df['px_abs'] = df['Latitude'].apply(lambda x: to_pixel_coord(x, RESOLUTION))
    df['py_abs'] = df['Longitude'].apply(lambda x: to_pixel_coord(x, RESOLUTION))

    px_min = int(df['px_abs'].min()); py_min = int(df['py_abs'].min())
    px_max = int(df['px_abs'].max()); py_max = int(df['py_abs'].max())

    height = px_max - px_min + 1
    width  = py_max - py_min + 1
    total = int(height) * int(width)
    info(f"Grid computed -> height={height}, width={width}, total pixels={total}")

    # sanity check: warn if extremely large
    if total > 200_000_000:
        info("WARNING: total pixels exceed 200M — this may require a lot of RAM/disk.")
        info("Proceeding, but if your machine is limited, consider tiling or sparse mode.")
        # no automatic abort; user asked for this case

    # Relative pixel coords
    df['px_rel'] = df['px_abs'] - px_min
    df['py_rel'] = df['py_abs'] - py_min

    # ensure channel columns present
    if 'Order_Weight' not in df.columns: df['Order_Weight'] = 0.0
    if 'Order_Volume' not in df.columns: df['Order_Volume'] = 0.0
    if 'Priority_Level' not in df.columns: df['Priority_Level'] = 0.0

    # aggregate: sum weight & volume, priority = min
    info("Aggregating customers falling into same pixel (sum weight/volume, min priority)...")
    grouped = df.groupby(['px_rel','py_rel']).agg({
        'Order_Weight':'sum',
        'Order_Volume':'sum',
        'Priority_Level':'min'
    }).reset_index()
    info(f"Non-zero pixels after grouping: {len(grouped)}")

    # prepare uint8 image (may be large)
    try:
        info("Allocating uint8 image array...")
        img = np.zeros((height, width, 3), dtype=np.uint8)  # default zeros for empty pixels
    except Exception as e:
        info("ERROR allocating image array (insufficient memory).")
        traceback.print_exc()
        info("Suggestion: use tiling or sparse export instead.")
        return

    # vectorized fill: compute indices and scaled values
    rows = grouped['px_rel'].to_numpy(dtype=np.int64)
    cols = grouped['py_rel'].to_numpy(dtype=np.int64)

    # R channel: Order_Weight direct, round and clip 0-255
    r_vals_f = grouped['Order_Weight'].to_numpy(dtype=np.float64)
    r_vals = np.clip(np.round(r_vals_f).astype(np.int32), 0, 255).astype(np.uint8)

    # G channel: Order_Volume trực tiếp, làm tròn và clip 0..255
    g_vals_f = grouped['Order_Volume'].to_numpy(dtype=np.float64)
    g_vals = np.clip(np.round(g_vals_f).astype(np.int32), 0, 255).astype(np.uint8)

    # B channel: Priority_Level clipped to 0..255 (cast int)
    p_vals_f = grouped['Priority_Level'].to_numpy(dtype=np.float64)
    p_vals = np.clip(np.round(p_vals_f).astype(np.int32), 0, 255).astype(np.uint8)

    # assign to image
    info("Filling image array with aggregated pixels...")
    img[rows, cols, 0] = r_vals
    img[rows, cols, 1] = g_vals
    img[rows, cols, 2] = p_vals

    # Save sparse CSV for exact values (float)
    try:
        sparse_df = grouped.copy()
        sparse_out = Path(OUT_DIR) / "sparse_30M.csv"
        sparse_df.rename(columns={'px_rel':'row','py_rel':'col',
                                  'Order_Weight':'weight','Order_Volume':'volume','Priority_Level':'priority'}, inplace=True)
        sparse_df.to_csv(sparse_out, index=False, encoding='utf-8-sig', float_format="%.6f")
        info(f"Sparse CSV saved -> {sparse_out}")
    except Exception:
        info("Failed to save sparse CSV:")
        traceback.print_exc()

    # Save metadata
    try:
        meta = {
            'resolution_deg': RESOLUTION,
            'px_min_abs': int(px_min),
            'py_min_abs': int(py_min),
            'px_max_abs': int(px_max),
            'py_max_abs': int(py_max),
            'height_pixels': int(height),
            'width_pixels': int(width),
            'nonzero_pixels': int(len(grouped))
        }

        meta_out = Path(OUT_DIR) / "metadata_30M.txt"
        with open(meta_out, "w", encoding="utf-8") as f:
            for k,v in meta.items():
                f.write(f"{k}={v}\n")
        info(f"Metadata saved -> {meta_out}")
    except Exception:
        info("Failed to save metadata:")
        traceback.print_exc()

    # Save numeric .npy (optional) and PNG image
    Path(OUT_DIR).mkdir(parents=True, exist_ok=True)
    if SAVE_NPY:
        try:
            npy_out = Path(OUT_DIR) / "rgb_30M_uint8.npy"
            np.save(npy_out, img)
            info(f"Saved uint8 image array (.npy) -> {npy_out}")
        except Exception:
            info("Failed to save .npy:")
            traceback.print_exc()

    # Save PNG (lossless compressed)
    try:
        png_out = Path(OUT_DIR) / "rgb_30M.png"
        info(f"Saving PNG -> {png_out}")
        img_pil = Image.fromarray(img, mode="RGB")
        img_pil.save(png_out, format="PNG", optimize=True)
        info(f"Saved PNG -> {png_out}")
    except Exception:
        info("Failed to save PNG:")
        traceback.print_exc()
        return

    # summary & time
    t1 = time.time()
    info(f"Image shape: {img.shape}, dtype={img.dtype}")
    info("Note: R mapped directly to Order_Weight (clipped 0..255).")
    info("      G mapped directly to Order_Volume (clipped 0..255).")
    info("      B mapped to Priority_Level (clipped 0..255).")


if __name__ == "__main__":
    main()
