import pandas as pd
import matplotlib.pyplot as plt
import os
import sys

def visualize(assignment_path, routes_path, out_file):
    print(f"Reading {assignment_path}...")
    df_assign = pd.read_csv(assignment_path)
    
    plt.figure(figsize=(12, 12))
    
    # Plot unassigned
    unassigned = df_assign[df_assign['cluster'] == -1]
    if not unassigned.empty:
        plt.scatter(unassigned['col'], unassigned['row'], c='lightgray', s=10, label='Unassigned')

    # Plot clusters
    assigned = df_assign[df_assign['cluster'] != -1]
    if not assigned.empty:
        # Get unique clusters
        clusters = assigned['cluster'].unique()
        # Use a colormap
        cmap = plt.get_cmap('tab20')
        
        for i, cid in enumerate(clusters):
            cluster_data = assigned[assigned['cluster'] == cid]
            color = cmap(i % 20)
            plt.scatter(cluster_data['col'], cluster_data['row'], color=color, s=15)
            
    # Plot routes/centers if available
    if os.path.exists(routes_path):
        print(f"Reading {routes_path}...")
        df_routes = pd.read_csv(routes_path)
        plt.scatter(df_routes['center_col'], df_routes['center_row'], c='black', marker='x', s=50, label='Route Center')

    plt.title(f"Initialization Result\nServed: {len(assigned)} / {len(df_assign)}")
    plt.xlabel("Column")
    plt.ylabel("Row")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.gca().invert_yaxis() # Match matrix coordinates
    
    print(f"Saving to {out_file}...")
    plt.savefig(out_file, dpi=300, bbox_inches='tight')
    print("Done.")

if __name__ == "__main__":
    base_dir = "init_out"
    if len(sys.argv) > 1:
        base_dir = sys.argv[1]
        
    assign_file = os.path.join(base_dir, "init_assignment.csv")
    routes_file = os.path.join(base_dir, "init_routes.csv")
    out_png = os.path.join(base_dir, "viz_init.png")
    
    visualize(assign_file, routes_file, out_png)
