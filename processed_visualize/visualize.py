# viz_from_csv_static.py
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

customers_csv = r"D:\A UEH_UNIVERSITY\RESEACH\OR_AI_paper\Zzz_data\LMDO processed\Ho_Chi_Minh_City\customers__Sheet1.csv"
depots_csv    = r"D:\A UEH_UNIVERSITY\RESEACH\OR_AI_paper\Zzz_data\LMDO processed\Ho_Chi_Minh_City\depots__Sheet1.csv"
roads_csv     = r"D:\A UEH_UNIVERSITY\RESEACH\OR_AI_paper\Zzz_data\LMDO processed\Ho_Chi_Minh_City\roads__Sheet1.csv"  # optional
out_png = r"D:\A UEH_UNIVERSITY\RESEACH\OR_AI_paper\Zzz_data\LMDO processed\Ho_Chi_Minh_City\map_static.png"

cust = pd.read_csv(customers_csv)
dep  = pd.read_csv(depots_csv)

plt.figure(figsize=(10,10))

# màu priority
priority_to_color = {1:'red',2:'orange',3:'blue',4:'green'}
colors = []
for p in cust.get('Priority_Level', pd.Series([None]*len(cust))):
    try:
        colors.append(priority_to_color.get(int(p), 'gray'))
    except:
        colors.append('gray')

sizes = [max(1, float(w)) for w in cust.get('Order_Weight', pd.Series([1]*len(cust)))]
sizes = [ (s**0.5)*10 for s in sizes ]

plt.scatter(cust['Longitude'], cust['Latitude'], s=sizes, c=colors, alpha=0.7, label='Customers', edgecolors='k', linewidths=0.3)
plt.scatter(dep['Longitude'], dep['Latitude'], marker='s', s=150, c='black', label='Depots')

# vẽ roads nếu có dạng start/end hoặc geometry như trên
roads_path = Path(roads_csv)
if roads_path.exists():
    try:
        roads = pd.read_csv(roads_path)
        if {'Start_Lat','Start_Lon','End_Lat','End_Lon'}.issubset(roads.columns):
            for _, r in roads.iterrows():
                plt.plot([r['Start_Lon'], r['End_Lon']], [r['Start_Lat'], r['End_Lat']], linewidth=1, alpha=0.6)
        elif 'geometry' in roads.columns:
            for geom in roads['geometry'].dropna():
                pts = []
                for seg in str(geom).split(';'):
                    latlon = seg.strip().split(',')
                    if len(latlon)==2:
                        pts.append((float(latlon[1]), float(latlon[0])))  # (lon, lat) nếu cần
                if pts:
                    lons = [p[0] for p in pts]; lats = [p[1] for p in pts]
                    plt.plot(lons, lats, linewidth=1, alpha=0.6)
    except Exception as e:
        print("Không vẽ roads:", e)

# label depots
for _, r in dep.iterrows():
    plt.text(r['Longitude']+0.002, r['Latitude']+0.002, r.get('Depot_ID',''), fontsize=9, weight='bold')

plt.xlabel("Longitude"); plt.ylabel("Latitude")
plt.title("Customers and Depots — Ho Chi Minh City")
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.tight_layout()
plt.savefig(out_png, dpi=300)
print("Saved static map to:", out_png)
plt.show()
