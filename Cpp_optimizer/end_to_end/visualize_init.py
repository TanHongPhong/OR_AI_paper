#!/usr/bin/env python3
"""
Visualize grid-tensor customers + initial solution assignments.
Each route/cluster is colored differently, with center marked as a dot.

Inputs (from C++ init test):
  - init_assignment.csv  (required)
  - init_routes.csv      (required for plotting centers)
  - depots_grid_min.csv  (optional, for plotting depot locations)

Usage:
  python visualize_init.py --assignment grid_out/init_assignment.csv --routes grid_out/init_routes.csv --depots grid_out/depots_grid_min.csv --out init_viz.png
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')  # Non-GUI backend
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--assignment", required=True, help="Path to init_assignment.csv")
    ap.add_argument("--routes", default="", help="Path to init_routes.csv (for centers)")
    ap.add_argument("--depots", default="", help="Path to depots_grid_min.csv")
    ap.add_argument("--point_size", type=float, default=12.0, help="Scatter point size for customers")
    ap.add_argument("--center_size", type=float, default=60.0, help="Scatter point size for centers")
    ap.add_argument("--depot_size", type=float, default=200.0, help="Scatter point size for depots")
    ap.add_argument("--figsize", type=float, default=14.0, help="Figure size (square)")
    ap.add_argument("--dpi", type=int, default=200, help="Output DPI")
    ap.add_argument("--invert_y", action="store_true", help="Invert y-axis (matrix view)")
    ap.add_argument("--out", default="", help="Output image path. If omitted, show window.")
    args = ap.parse_args()

    # ===== Load assignment data =====
    df = pd.read_csv(args.assignment)
    df.columns = [c.lower() for c in df.columns]

    required = ["row", "col", "cluster"]
    for c in required:
        if c not in df.columns:
            raise ValueError(f"Missing column '{c}' in {args.assignment}")

    # Filter valid rows (has customer_id and cluster >= 0)
    df["cluster"] = pd.to_numeric(df["cluster"], errors="coerce").fillna(-1).astype(int)
    df_assigned = df[df["cluster"] >= 0].copy()
    df_unassigned = df[df["cluster"] < 0].copy()

    # ===== Load routes for centers =====
    centers = {}
    if args.routes and os.path.exists(args.routes):
        rt_df = pd.read_csv(args.routes)
        rt_df.columns = [c.lower() for c in rt_df.columns]
        if "center_row" in rt_df.columns and "center_col" in rt_df.columns:
            for _, r in rt_df.iterrows():
                # We assume center is unique per trip, use (vehicle_id, trip_idx) or just row/col
                centers[(int(r["center_row"]), int(r["center_col"]))] = True

    # ===== Load depots =====
    depots = []
    if args.depots and os.path.exists(args.depots):
        dep_df = pd.read_csv(args.depots)
        dep_df.columns = [c.lower() for c in dep_df.columns]
        if "row" in dep_df.columns and "col" in dep_df.columns:
            for _, r in dep_df.iterrows():
                depots.append({"row": int(r["row"]), "col": int(r["col"]),
                               "id": str(r.get("depot_id", r.get("Depot_ID", "")))})

    # ===== Prepare colors for clusters =====
    cluster_ids = sorted(df_assigned["cluster"].unique())
    cluster_to_color = {cid: i for i, cid in enumerate(cluster_ids)}
    n_clusters = len(cluster_ids)

    # Use a colormap with many distinct colors
    cmap = plt.cm.get_cmap("tab20" if n_clusters <= 20 else "hsv", max(n_clusters, 1))

    # ===== Plot =====
    fig, ax = plt.subplots(figsize=(args.figsize, args.figsize))

    # Plot assigned customers colored by cluster
    if len(df_assigned) > 0:
        xs = df_assigned["col"].to_numpy(dtype=float)
        ys = df_assigned["row"].to_numpy(dtype=float)
        colors = [cluster_to_color[c] for c in df_assigned["cluster"]]
        ax.scatter(xs, ys, c=colors, cmap=cmap, s=args.point_size, marker="s", alpha=0.8, edgecolors="none")

    # Plot unassigned customers (X marker, gray)
    if len(df_unassigned) > 0:
        ux = df_unassigned["col"].to_numpy(dtype=float)
        uy = df_unassigned["row"].to_numpy(dtype=float)
        ax.scatter(ux, uy, c="gray", s=args.point_size * 1.5, marker="x", alpha=0.6, label="Unassigned")

    # Plot centers (black dot with white edge)
    if centers:
        cx = [c[1] for c in centers.keys()]  # col
        cy = [c[0] for c in centers.keys()]  # row
        ax.scatter(cx, cy, c="black", s=args.center_size, marker="o", 
                   edgecolors="white", linewidths=1.5, zorder=10, label="Route Center")

    # Plot depots (star marker)
    if depots:
        dx = [d["col"] for d in depots]
        dy = [d["row"] for d in depots]
        ax.scatter(dx, dy, c="red", s=args.depot_size, marker="*", 
                   edgecolors="black", linewidths=1.0, zorder=15, label="Depot")
        # Add depot labels
        for d in depots:
            ax.text(d["col"] + 5, d["row"] + 5, d["id"], fontsize=9, fontweight="bold")

    # ===== Formatting =====
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("col", fontsize=12)
    ax.set_ylabel("row", fontsize=12)
    ax.set_title(f"Initial Solution: {n_clusters} Routes, {len(df_assigned)} Assigned, {len(df_unassigned)} Unassigned", 
                 fontsize=14, fontweight="bold")
    ax.grid(True, alpha=0.3)

    if args.invert_y:
        ax.invert_yaxis()

    # Legend
    handles = [
        Line2D([0], [0], marker="s", linestyle="", color="C0", markersize=10, label="Customer (by cluster)"),
        Line2D([0], [0], marker="o", linestyle="", color="black", markeredgecolor="white", 
               markersize=10, markeredgewidth=1.5, label="Route Center"),
        Line2D([0], [0], marker="*", linestyle="", color="red", markeredgecolor="black", 
               markersize=14, label="Depot"),
    ]
    if len(df_unassigned) > 0:
        handles.append(Line2D([0], [0], marker="x", linestyle="", color="gray", markersize=10, label="Unassigned"))
    ax.legend(handles=handles, loc="upper right", framealpha=0.9)

    plt.tight_layout()

    if args.out:
        out_dir = os.path.dirname(args.out)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        plt.savefig(args.out, dpi=args.dpi)
        print(f"[OK] Saved: {args.out}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
