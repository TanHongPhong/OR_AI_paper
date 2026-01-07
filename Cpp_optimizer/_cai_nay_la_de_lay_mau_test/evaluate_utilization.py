
import json
import pandas as pd

V_CSV = r"d:\A UEH_UNIVERSITY\RESEACH\OR_AI_paper\Zzz_data\LMDO processed\Ho_Chi_Minh_City\vehicles__Sheet1.csv"
UTIL_JSON = "temp_routes.json"

def evaluate():
    try:
        with open(UTIL_JSON, 'r') as f:
            data = json.load(f)
        df_util_raw = pd.DataFrame(data)
        df_veh = pd.read_csv(V_CSV)
        
        # Aggregation
        df_util_raw['trip_km'] = pd.to_numeric(df_util_raw['trip_km'], errors='coerce')
        df_util = df_util_raw.groupby('vehicle_id').agg(total_km=('trip_km','sum'), trips=('vehicle_id','count')).reset_index()
        
        # Strip and lower all column names
        df_util.columns = [c.strip().lower() for c in df_util.columns]
        df_veh.columns = [c.strip().lower() for c in df_veh.columns]
        
        # Strip and lower for clean matching
        df_util['match_id'] = df_util['vehicle_id'].astype(str).str.strip().str.upper()
        df_veh['match_id'] = df_veh['vehicle_id'].astype(str).str.strip().str.upper()
        
        # Merge on cleaned key
        df_merged = pd.merge(df_util, df_veh, on="match_id", how="inner")
        
        if df_merged.empty:
            print("Warning: Merged dataframe is empty.")
            print("Cleanup Summary:")
            print("- Util Samples:", df_util['match_id'].head(3).tolist())
            print("- Veh Samples:", df_veh['match_id'].head(3).tolist())
            return

        # Calc
        df_merged['total_km'] = pd.to_numeric(df_merged['total_km'], errors='coerce')
        df_merged['max_distance'] = pd.to_numeric(df_merged['max_distance'], errors='coerce')
        df_merged['km_util_pct'] = (df_merged['total_km'] / df_merged['max_distance']) * 100
        
        # Determine actual column names for output
        id_col = 'vehicle_id_x' if 'vehicle_id_x' in df_merged.columns else 'vehicle_id'
        type_col = 'vehicle_type' if 'vehicle_type' in df_merged.columns else 'vehicle_type_y'

        report = df_merged[[id_col, type_col, 'trips', 'total_km', 'max_distance', 'km_util_pct']].copy()
        report.columns = ['ID', 'Type', 'Trips', 'Total_KM', 'Max_Distance', 'Util_Pct']
        report = report.sort_values(by="Util_Pct", ascending=False)
        
        print("\n=== VEHICLE MILEAGE UTILIZATION REPORT ===")
        print(report.to_string(index=False, formatters={
            'total_km': '{:,.1f}'.format,
            'km_util_pct': '{:.1f}%'.format
        }))
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    evaluate()
