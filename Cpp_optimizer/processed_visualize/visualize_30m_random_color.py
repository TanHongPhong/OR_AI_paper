# visualize_30m_random_color.py
# Load sparse_30M.csv + metadata_30M.txt and visualize them
# into a 1080x1080 image, drawing each nonzero pixel as a circle
# with a random color on white background.
#
# Output: lowres_visual_1080_random.png

import pandas as pd
import numpy as np
from pathlib import Path
from PIL import Image, ImageDraw
import random

# ======= USER CONFIG =======
IN_DIR = r"D:\A UEH_UNIVERSITY\RESEACH\OR_AI_paper\Zzz_data\image_csv_out"
SPARSE_CSV = "sparse_30M.csv"
META_TXT = "metadata_30M.txt"
OUT_IMG = "lowres_visual_1080_random.png"

OUT_SIZE = 1080          # width = height = OUT_SIZE
POINT_RADIUS = 2         # circle radius
BG_COLOR = (255, 255, 255)   # background = WHITE
# ===========================


def load_metadata(path):
    meta = {}
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            if "=" in line:
                k, v = line.strip().split("=")
                meta[k] = float(v)
    return meta


def main():
    base = Path(IN_DIR)
    sparse_path = base / SPARSE_CSV
    meta_path = base / META_TXT
    out_path = base / OUT_IMG

    print(f"Loading sparse CSV: {sparse_path}")
    df = pd.read_csv(sparse_path)

    print(f"Loading metadata: {meta_path}")
    meta = load_metadata(meta_path)

    H = int(meta["height_pixels"])
    W = int(meta["width_pixels"])
    print(f"Original resolution: {W} x {H} pixels")

    # Create blank canvas OUT_SIZE x OUT_SIZE (white background)
    canvas = Image.new("RGB", (OUT_SIZE, OUT_SIZE), BG_COLOR)
    draw = ImageDraw.Draw(canvas)

    # Scaling to map original coords -> OUT_SIZE x OUT_SIZE
    scale_x = OUT_SIZE / W
    scale_y = OUT_SIZE / H

    print("Projecting points and drawing circles with random colors...")

    rows = df["row"].to_numpy()
    cols = df["col"].to_numpy()

    # Optional: set seed nếu muốn tái lập màu (comment nếu muốn random mỗi lần chạy)
    # random.seed(42)

    for r, c in zip(rows, cols):
        # Convert original pixel -> lowres pixel
        x = int(c * scale_x)
        y = int(r * scale_y)

        # Clamp vào khung [0, OUT_SIZE-1] cho chắc
        if x < 0 or x >= OUT_SIZE or y < 0 or y >= OUT_SIZE:
            continue

        # Random màu cho "customer/pixel"
        color = (
            random.randint(0, 255),
            random.randint(0, 255),
            random.randint(0, 255),
        )

        # Draw circle
        draw.ellipse(
            (
                x - POINT_RADIUS,
                y - POINT_RADIUS,
                x + POINT_RADIUS,
                y + POINT_RADIUS,
            ),
            fill=color,
        )

    print(f"Saving output image -> {out_path}")
    canvas.save(out_path, format="PNG")
    print("Done.")


if __name__ == "__main__":
    main()
