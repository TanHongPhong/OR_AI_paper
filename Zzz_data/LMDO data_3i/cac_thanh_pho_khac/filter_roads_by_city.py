import pandas as pd
from pathlib import Path
import glob
import os

def filter_roads_by_city():
    """
    Lọc các tuyến đường theo từng thành phố dựa trên customers đã được tách
    Logic:
    1. Đọc customers từ mỗi thành phố trong data/process
    2. Đọc tất cả các file road từ data/LMDO data_3i/roads
    3. Với mỗi thành phố:
       - Lấy danh sách Customer_ID
       - Lọc các road có Origin_Node_ID hoặc Destination_Node_ID là customer trong thành phố đó
       - Chỉ giữ lại road nếu cả Origin và Destination đều là customer trong thành phố đó
       - Lưu vào folder của thành phố đó
    """
    
    # Đường dẫn
    base_dir = Path(__file__).parent
    process_dir = base_dir / "data" / "process"
    roads_dir = base_dir / "data" / "LMDO data_3i" / "roads"
    
    # Kiểm tra thư mục tồn tại
    if not process_dir.exists():
        print(f"❌ Không tìm thấy thư mục: {process_dir}")
        return
    
    if not roads_dir.exists():
        print(f"❌ Không tìm thấy thư mục: {roads_dir}")
        return
    
    # Lấy danh sách các folder thành phố
    city_folders = [f for f in process_dir.iterdir() if f.is_dir()]
    
    if len(city_folders) == 0:
        print(f"❌ Không tìm thấy folder thành phố nào trong: {process_dir}")
        print("⚠️ Vui lòng chạy split_by_city.py trước để tách dữ liệu theo thành phố.")
        return
    
    print(f"📂 Tìm thấy {len(city_folders)} thành phố trong {process_dir}")
    print("\n" + "="*60)
    
    # Lấy danh sách tất cả các thư mục road (roads_D001_D002, roads_D003_D004, etc.)
    road_subdirs = [d for d in roads_dir.iterdir() if d.is_dir() and d.name.startswith("roads_")]
    print(f"📂 Tìm thấy {len(road_subdirs)} thư mục road")
    
    # Đọc tất cả customers từ tất cả thành phố để xác định customer nào thuộc thành phố nào
    print("\n📖 Đang đọc danh sách customers từ tất cả thành phố...")
    all_city_customer_ids = {}  # {city_name: set of customer_ids}
    
    for city_folder in city_folders:
        city_name = city_folder.name
        customers_file = city_folder / "customers.xlsx"
        
        if customers_file.exists():
            try:
                df_city_customers = pd.read_excel(customers_file)
                
                # Tìm cột Customer_ID
                customer_id_col = None
                for col in df_city_customers.columns:
                    col_lower = str(col).lower()
                    if 'customer' in col_lower and 'id' in col_lower:
                        customer_id_col = col
                        break
                
                if not customer_id_col:
                    for col in df_city_customers.columns:
                        col_lower = str(col).lower()
                        if 'id' in col_lower:
                            customer_id_col = col
                            break
                    if not customer_id_col:
                        customer_id_col = df_city_customers.columns[0]
                
                df_city_customers[customer_id_col] = df_city_customers[customer_id_col].astype(str).str.strip().str.upper()
                city_customer_ids = set(df_city_customers[customer_id_col].dropna().unique())
                all_city_customer_ids[city_name] = city_customer_ids
            except Exception as e:
                print(f"⚠️ {city_name}: Lỗi khi đọc customers - {str(e)}")
                continue
    
    # Tạo set tất cả customer IDs từ tất cả thành phố
    all_customers_all_cities = set()
    for customer_set in all_city_customer_ids.values():
        all_customers_all_cities.update(customer_set)
    
    print(f"✅ Đã đọc customers từ {len(all_city_customer_ids)} thành phố")
    print(f"   - Tổng số customers: {len(all_customers_all_cities)}")
    
    # Xử lý từng thành phố
    print("\n" + "="*60)
    print("🔄 Đang lọc tuyến đường theo từng thành phố...")
    
    for city_folder in city_folders:
        city_name = city_folder.name
        customers_file = city_folder / "customers.xlsx"
        depots_file = city_folder / "depots.xlsx"
        
        # Kiểm tra file customers tồn tại
        if not customers_file.exists():
            print(f"\n⚠️ {city_name}: Không tìm thấy file customers.xlsx, bỏ qua")
            continue
        
        if city_name not in all_city_customer_ids:
            print(f"\n⚠️ {city_name}: Không có dữ liệu customers, bỏ qua")
            continue
        
        try:
            city_customer_ids = all_city_customer_ids[city_name]
            
            if len(city_customer_ids) == 0:
                print(f"\n⚠️ {city_name}: Không có customer nào, bỏ qua")
                continue
            
            # Đọc depots của thành phố
            city_depot_ids = set()
            if depots_file.exists():
                try:
                    df_city_depots = pd.read_excel(depots_file)
                    
                    # Tìm cột Depot_ID
                    depot_id_col = None
                    for col in df_city_depots.columns:
                        col_lower = str(col).lower()
                        if 'depot' in col_lower and 'id' in col_lower:
                            depot_id_col = col
                            break
                    
                    if not depot_id_col:
                        for col in df_city_depots.columns:
                            col_lower = str(col).lower()
                            if 'id' in col_lower:
                                depot_id_col = col
                                break
                        if not depot_id_col:
                            depot_id_col = df_city_depots.columns[0]
                    
                    df_city_depots[depot_id_col] = df_city_depots[depot_id_col].astype(str).str.strip().str.upper()
                    city_depot_ids = set(df_city_depots[depot_id_col].dropna().unique())
                except Exception as e:
                    print(f"   ⚠️ Không đọc được depots: {str(e)}")
            
            if len(city_depot_ids) == 0:
                print(f"\n⚠️ {city_name}: Không có depot nào, bỏ qua")
                continue
            
            print(f"\n📊 {city_name}:")
            print(f"   - Số lượng customers: {len(city_customer_ids)}")
            print(f"   - Số lượng depots: {len(city_depot_ids)}")
            print(f"   - Depot IDs: {sorted(list(city_depot_ids))[:5]}{'...' if len(city_depot_ids) > 5 else ''}")
            
            # Tìm và đọc file road liên quan đến depot của thành phố này
            # Format: depot D001_1 -> file roads_D001_D002/roads_D001_1.csv
            df_city_roads_list = []
            
            for depot_id in city_depot_ids:
                # Tìm thư mục chứa depot này (ví dụ: D001_1 -> roads_D001_D002)
                depot_prefix = depot_id.split('_')[0]  # D001 từ D001_1
                
                # Tìm thư mục road chứa depot này
                matching_subdir = None
                for subdir in road_subdirs:
                    # Kiểm tra xem tên thư mục có chứa depot prefix không
                    # Ví dụ: roads_D001_D002 chứa D001
                    if depot_prefix in subdir.name:
                        matching_subdir = subdir
                        break
                
                if not matching_subdir:
                    print(f"   ⚠️ Không tìm thấy thư mục road cho depot {depot_id} (prefix: {depot_prefix})")
                    continue
                
                # Tìm file road cho depot này
                # Format: roads_D001_1.csv, roads_D001_2.csv, etc.
                road_file_pattern = matching_subdir / f"roads_{depot_id}.csv"
                
                if not road_file_pattern.exists():
                    # Thử tìm file với format khác (có thể không có prefix "roads_")
                    alt_pattern = matching_subdir / f"{depot_id}.csv"
                    if alt_pattern.exists():
                        road_file_pattern = alt_pattern
                    else:
                        print(f"   ⚠️ Không tìm thấy file road cho depot {depot_id}: {road_file_pattern.name}")
                        # Liệt kê các file có sẵn trong thư mục để debug
                        available_files = list(matching_subdir.glob("*.csv"))
                        if len(available_files) > 0:
                            print(f"      Các file có sẵn: {[f.name for f in available_files[:5]]}{'...' if len(available_files) > 5 else ''}")
                        continue
                
                try:
                    df_road = pd.read_csv(road_file_pattern)
                    
                    # Chuẩn hóa dữ liệu
                    if "Origin_Node_ID" in df_road.columns and "Destination_Node_ID" in df_road.columns:
                        df_road["Origin_Node_ID"] = df_road["Origin_Node_ID"].astype(str).str.strip().str.upper()
                        df_road["Destination_Node_ID"] = df_road["Destination_Node_ID"].astype(str).str.strip().str.upper()
                        df_road = df_road.dropna(subset=["Origin_Node_ID", "Destination_Node_ID"])
                        
                        # Lọc chỉ giữ lại road có customer thuộc thành phố này
                        # Loại bỏ road có customer ngoài tỉnh
                        customers_other_cities = all_customers_all_cities - city_customer_ids
                        
                        df_road_filtered = df_road[
                            (
                                (df_road["Origin_Node_ID"].isin(city_customer_ids)) |
                                (df_road["Destination_Node_ID"].isin(city_customer_ids))
                            ) &
                            ~(
                                (df_road["Origin_Node_ID"].isin(customers_other_cities)) |
                                (df_road["Destination_Node_ID"].isin(customers_other_cities))
                            )
                        ].copy()
                        
                        if len(df_road_filtered) > 0:
                            df_city_roads_list.append(df_road_filtered)
                            print(f"   - ✅ Đọc {road_file_pattern.name}: {len(df_road_filtered)} tuyến đường")
                        else:
                            print(f"   - ⚠️ File {road_file_pattern.name} không có road nào phù hợp với customers của thành phố")
                    else:
                        print(f"   ⚠️ File {road_file_pattern.name} không có cột Origin_Node_ID hoặc Destination_Node_ID")
                except Exception as e:
                    print(f"   ⚠️ Lỗi khi đọc file {road_file_pattern}: {str(e)}")
                    import traceback
                    print(f"      Chi tiết: {traceback.format_exc()}")
                    continue
            
            # Gộp tất cả road của thành phố
            if len(df_city_roads_list) > 0:
                df_city_roads = pd.concat(df_city_roads_list, ignore_index=True)
                # Loại bỏ duplicate nếu có
                df_city_roads = df_city_roads.drop_duplicates()
            else:
                df_city_roads = pd.DataFrame()
            
            print(f"   - Tổng số tuyến đường: {len(df_city_roads)}")
            
            # Lưu file road cho thành phố
            roads_output = city_folder / "roads.xlsx"
            
            try:
                if len(df_city_roads) > 0:
                    df_city_roads.to_excel(roads_output, index=False, engine='openpyxl')
                    print(f"   - ✅ Đã lưu {len(df_city_roads)} tuyến đường vào: {roads_output}")
                else:
                    # Tạo file rỗng nếu không có dữ liệu
                    pd.DataFrame().to_excel(roads_output, index=False, engine='openpyxl')
                    print(f"   - ⚠️ Không có tuyến đường nào, đã tạo file rỗng")
            except ImportError:
                # Nếu không có openpyxl, thử dùng mặc định
                try:
                    if len(df_city_roads) > 0:
                        df_city_roads.to_excel(roads_output, index=False)
                        print(f"   - ✅ Đã lưu {len(df_city_roads)} tuyến đường vào: {roads_output}")
                    else:
                        pd.DataFrame().to_excel(roads_output, index=False)
                        print(f"   - ⚠️ Không có tuyến đường nào, đã tạo file rỗng")
                except Exception as e:
                    print(f"   - ❌ Lỗi khi lưu file: {str(e)}")
            
        except Exception as e:
            print(f"\n❌ {city_name}: Lỗi khi xử lý - {str(e)}")
            continue
    
    print("\n" + "="*60)
    print("✅ Hoàn thành! Đã lọc tuyến đường cho tất cả các thành phố.")
    print(f"📂 Dữ liệu được lưu trong các folder thành phố tại: {process_dir}")

if __name__ == "__main__":
    filter_roads_by_city()

