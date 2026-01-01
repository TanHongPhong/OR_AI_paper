"""
Visualization and Analysis for Territory-Free Centers Patch Optimizer
"""
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

# Configuration
OUTPUT_DIR = Path(r"D:\A UEH_UNIVERSITY\RESEACH\OR_AI_paper\Cpp_optimizer\territory_free_centers_patch\output")

def find_latest_run(output_dir):
    """Find the most recent run directory"""
    runs = sorted([d for d in output_dir.iterdir() if d.is_dir() and d.name.startswith("run_")])
    return runs[-1] if runs else None

def load_results(run_dir):
    """Load all result CSV files"""
    results = {}
    
    # Load objective summary
    obj_file = run_dir / "best_objective_summary.csv"
    if obj_file.exists():
        results['objective'] = pd.read_csv(obj_file)
        
    # Load routes summary
    routes_file = run_dir / "best_routes_summary.csv"
    if routes_file.exists():
        results['routes'] = pd.read_csv(routes_file)
        
    # Load customer assignments
    cust_file = run_dir / "best_customer_assignments.csv"
    if cust_file.exists():
        results['customers'] = pd.read_csv(cust_file)
        
    # Load vehicles summary
    veh_file = run_dir / "best_vehicles_summary.csv"
    if veh_file.exists():
        results['vehicles'] = pd.read_csv(veh_file)
        
    return results

def analyze_results(results):
    """Analyze and print key metrics"""
    print("=" * 60)
    print("TERRITORY-FREE CENTERS PATCH - ANALYSIS REPORT")
    print("=" * 60)
    
    # Objective summary
    if 'objective' in results:
        obj = results['objective']
        print("\n📊 OBJECTIVE SUMMARY:")
        print(obj.to_string(index=False))
    
    # Routes analysis
    if 'routes' in results:
        routes = results['routes']
        print(f"\n🚗 ROUTES SUMMARY:")
        print(f"  Total routes: {len(routes)}")
        print(f"  Total distance: {routes['distance'].sum():.2f} km")
        print(f"  Avg distance per route: {routes['distance'].mean():.2f} km")
        print(f"  Total load (weight): {routes['load_w'].sum():.2f}")
        
        # Routes by vehicle
        if 'vehicle_id' in routes.columns:
            veh_routes = routes.groupby('vehicle_id').agg({
                'distance': 'sum',
                'load_w': 'sum'
            }).reset_index()
            print(f"\n  Routes by vehicle:")
            print(f"  Active vehicles: {len(veh_routes)}")
    
    # Customer assignments
    if 'customers' in results:
        cust = results['customers']
        total = len(cust)
        if 'cluster_id' in cust.columns:
            assigned = cust[cust['cluster_id'].notna() & (cust['cluster_id'] != '')]
            unassigned = total - len(assigned)
        else:
            assigned = cust
            unassigned = 0
            
        print(f"\n👥 CUSTOMER ASSIGNMENTS:")
        print(f"  Total customers: {total}")
        print(f"  Assigned: {len(assigned)} ({100*len(assigned)/total:.1f}%)")
        print(f"  Unassigned: {unassigned} ({100*unassigned/total:.1f}%)")
    
    # Vehicle utilization
    if 'vehicles' in results:
        veh = results['vehicles']
        print(f"\n🚚 VEHICLE UTILIZATION:")
        print(f"  Total vehicles: {len(veh)}")
        if 'total_km' in veh.columns:
            print(f"  Total km used: {veh['total_km'].sum():.2f}")
            print(f"  Avg km per vehicle: {veh['total_km'].mean():.2f}")

def visualize_routes(results, run_dir):
    """Create route visualization"""
    if 'customers' not in results:
        print("No customer data for visualization")
        return
        
    cust = results['customers']
    
    # Check required columns
    if 'x_km' not in cust.columns or 'y_km' not in cust.columns:
        print("Missing coordinate columns")
        return
    
    fig, ax = plt.subplots(1, 1, figsize=(14, 10))
    
    # Plot customers by cluster
    if 'cluster_id' in cust.columns:
        clusters = cust['cluster_id'].unique()
        colors = plt.cm.tab20(np.linspace(0, 1, min(20, len(clusters))))
        
        for i, cluster in enumerate(clusters):
            if pd.isna(cluster) or cluster == '':
                subset = cust[(cust['cluster_id'].isna()) | (cust['cluster_id'] == '')]
                ax.scatter(subset['x_km'], subset['y_km'], c='gray', s=10, alpha=0.3, label='Unassigned' if i == 0 else None)
            else:
                subset = cust[cust['cluster_id'] == cluster]
                color = colors[i % len(colors)]
                ax.scatter(subset['x_km'], subset['y_km'], c=[color], s=30, alpha=0.7)
    else:
        ax.scatter(cust['x_km'], cust['y_km'], c='blue', s=10, alpha=0.5)
    
    ax.set_xlabel('X (km)')
    ax.set_ylabel('Y (km)')
    ax.set_title('Territory-Free Centers Patch - Customer Assignments')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    
    # Save
    out_path = run_dir / "analysis_visualization.png"
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    print(f"\n📈 Visualization saved to: {out_path}")
    plt.close()

def main():
    # Find latest run
    run_dir = find_latest_run(OUTPUT_DIR)
    if not run_dir:
        print("No run directories found!")
        return
        
    print(f"Analyzing run: {run_dir.name}")
    
    # Load and analyze
    results = load_results(run_dir)
    analyze_results(results)
    
    # Visualize
    visualize_routes(results, run_dir)
    
    print("\n" + "=" * 60)
    print("Analysis complete!")
    print("=" * 60)

if __name__ == "__main__":
    main()
