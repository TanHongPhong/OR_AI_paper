#!/usr/bin/env python3
"""
Visualize VRP initial solution with clusters
- Each cluster has unique color
- Convex hull mesh for each cluster
- Depots marked with stars
"""

import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import LineCollection
import numpy as np
from scipy.spatial import ConvexHull
import os
import sys

def load_data(output_dir):
    """Load solution data from CSV files"""
    customers_file = os.path.join(output_dir, "initial_customer_assignments.csv")
    routes_file = os.path.join(output_dir, "initial_routes_summary.csv")
    
    customers_df = pd.read_csv(customers_file)
    routes_df = pd.read_csv(routes_file)
    
    return customers_df, routes_df

def load_coordinates(data_dir):
    """Load customer and depot coordinates from original data"""
    customers_file = os.path.join(data_dir, "customers__Sheet1.csv")
    depots_file = os.path.join(data_dir, "depots__Sheet1.csv")
    
    customers_df = pd.read_csv(customers_file)
    depots_df = pd.read_csv(depots_file)
    
    # Print columns for debugging
    print(f"Customer columns: {list(customers_df.columns[:5])}")
    
    # Try different possible column names
    id_col = 'customer_id' if 'customer_id' in customers_df.columns else 'Customer_ID'
    lon_col = 'Longitude' if 'Longitude' in customers_df.columns else 'longitude'
    lat_col = 'Latitude' if 'Latitude' in customers_df.columns else 'latitude'
    
    depot_id_col = 'Depot_ID' if 'Depot_ID' in depots_df.columns else 'depot_id'
    depot_lon_col = 'Longitude' if 'Longitude' in depots_df.columns else 'longitude'
    depot_lat_col = 'Latitude' if 'Latitude' in depots_df.columns else 'latitude'
    
    # Create coordinate dictionaries
    customer_coords = dict(zip(customers_df[id_col], 
                               zip(customers_df[lon_col], customers_df[lat_col])))
    depot_coords = dict(zip(depots_df[depot_id_col], 
                            zip(depots_df[depot_lon_col], depots_df[depot_lat_col])))
    
    return customer_coords, depot_coords

def visualize_solution(output_dir, data_dir, save_path=None):
    """
    Visualize the solution with clusters
    
    Args:
        output_dir: Directory with solution CSVs
        data_dir: Directory with original data CSVs
        save_path: Path to save figure (optional)
    """
    print("Loading data...")
    assignments, routes = load_data(output_dir)
    customer_coords, depot_coords = load_coordinates(data_dir)
    
    # Create figure
    fig, ax = plt.subplots(figsize=(16, 12))
    
    # Get unique routes (vehicle + trip combinations)
    served = assignments[assignments['Vehicle_ID'].notna()].copy()
    served['route_id'] = served['Vehicle_ID'] + '_' + served['Trip_Index'].astype(str)
    
    unique_routes = served['route_id'].unique()
    print(f"Visualizing {len(unique_routes)} routes...")
    
    # Generate colors for each route
    np.random.seed(42)
    colors = plt.cm.tab20c(np.linspace(0, 1, len(unique_routes)))
    if len(unique_routes) > 20:
        # Use more colors if needed
        colors = plt.cm.rainbow(np.linspace(0, 1, len(unique_routes)))
    
    route_colors = dict(zip(unique_routes, colors))
    
    # Plot each cluster
    for route_id in unique_routes:
        route_customers = served[served['route_id'] == route_id]['Customer_ID'].values
        
        # Get coordinates
        coords = []
        for cid in route_customers:
            if cid in customer_coords:
                lon, lat = customer_coords[cid]
                coords.append([lon, lat])
        
        if len(coords) < 3:
            # Too few points for convex hull, just plot points
            coords = np.array(coords)
            ax.scatter(coords[:, 0], coords[:, 1], 
                      c=[route_colors[route_id]], s=30, alpha=0.7, edgecolors='black', linewidth=0.5)
            continue
        
        coords = np.array(coords)
        color = route_colors[route_id]
        
        # Plot customer points
        ax.scatter(coords[:, 0], coords[:, 1], 
                  c=[color], s=30, alpha=0.8, edgecolors='black', linewidth=0.5, zorder=3)
        
        # Compute and draw convex hull
        try:
            hull = ConvexHull(coords)
            hull_points = coords[hull.vertices]
            
            # Draw filled polygon
            polygon = Polygon(hull_points, alpha=0.15, facecolor=color, 
                            edgecolor=color, linewidth=1.5, zorder=1)
            ax.add_patch(polygon)
            
            # Draw mesh (grid) connecting hull vertices
            # Create triangulation from centroid to hull vertices
            centroid = coords.mean(axis=0)
            
            lines = []
            for i, vertex in enumerate(hull_points):
                # Line from centroid to vertex
                lines.append([centroid, vertex])
                # Line to next vertex (hull edge)
                next_vertex = hull_points[(i + 1) % len(hull_points)]
                lines.append([vertex, next_vertex])
            
            lc = LineCollection(lines, colors=color, linewidths=0.5, alpha=0.4, zorder=2)
            ax.add_collection(lc)
            
        except Exception as e:
            print(f"Warning: Could not compute hull for route {route_id}: {e}")
    
    # Plot unserved customers (gray)
    unserved = assignments[assignments['Vehicle_ID'].isna()]['Customer_ID'].values
    unserved_coords = []
    for cid in unserved:
        if cid in customer_coords:
            lon, lat = customer_coords[cid]
            unserved_coords.append([lon, lat])
    
    if len(unserved_coords) > 0:
        unserved_coords = np.array(unserved_coords)
        ax.scatter(unserved_coords[:, 0], unserved_coords[:, 1], 
                  c='gray', s=15, alpha=0.3, marker='x', label='Unserved', zorder=2)
    
    # Plot depots
    depot_lons = [coords[0] for coords in depot_coords.values()]
    depot_lats = [coords[1] for coords in depot_coords.values()]
    ax.scatter(depot_lons, depot_lats, 
              c='red', s=300, marker='*', edgecolors='black', linewidth=2, 
              label='Depots', zorder=5)
    
    # Labels and formatting
    ax.set_xlabel('Longitude', fontsize=12)
    ax.set_ylabel('Latitude', fontsize=12)
    ax.set_title(f'VRP Solution Visualization\n{len(unique_routes)} Routes | '
                f'{len(served)} Served | {len(unserved)} Unserved', fontsize=14, fontweight='bold')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save or show
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved to {save_path}")
    else:
        plt.show()
    
    return fig, ax

if __name__ == "__main__":
    # Default paths
    output_dir = "outputs/final_fixed"
    data_dir = r"D:\A UEH_UNIVERSITY\RESEACH\OR_AI_paper\Zzz_data\LMDO processed\Ho_Chi_Minh_City"
    save_path = "outputs/final_fixed/solution_visualization.png"
    
    # Allow command line override
    if len(sys.argv) > 1:
        output_dir = sys.argv[1]
    if len(sys.argv) > 2:
        data_dir = sys.argv[2]
    if len(sys.argv) > 3:
        save_path = sys.argv[3]
    
    visualize_solution(output_dir, data_dir, save_path)
