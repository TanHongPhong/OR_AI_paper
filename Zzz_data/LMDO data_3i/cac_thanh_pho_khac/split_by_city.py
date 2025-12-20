import pandas as pd
import os
from pathlib import Path

def split_excel_by_city():
    """
    Tách các file Excel (customers, depots, vehicles) theo thành phố
    Mỗi thành phố sẽ được lưu vào một folder riêng trong data/process
    """
    
    # Đường dẫn thư mục chứa file Excel (có thể chỉnh sửa đường dẫn này)
    base_dir = Path(__file__).parent
    input_folder = base_dir / "data" / "LMDO data_3i"
    
    # Đường dẫn các file Excel
    customers_file = input_folder / "customers_vietnam.xlsx"
    depots_file = input_folder / "depots_vietnam.xlsx"
    vehicles_file = input_folder / "vehicles_vietnam.xlsx"
    
    # Kiểm tra file tồn tại
    missing_files = []
    if not customers_file.exists():
        missing_files.append("customers_vietnam.xlsx")
    if not depots_file.exists():
        missing_files.append("depots_vietnam.xlsx")
    if not vehicles_file.exists():
        missing_files.append("vehicles_vietnam.xlsx")
    
    if missing_files:
        print(f"❌ Không tìm thấy các file:\n{', '.join(missing_files)}")
        print(f"📂 Đang tìm trong thư mục: {input_folder}")
        return
    
    # Thư mục output
    output_base = base_dir / "data" / "process"
    output_base.mkdir(parents=True, exist_ok=True)
    
    print(f"📂 Thư mục input: {input_folder}")
    print(f"📂 Thư mục output: {output_base}")
    print("\n" + "="*60)
    
    # Đọc các file Excel
    print("\n📖 Đang đọc các file Excel...")
    try:
        df_customers = pd.read_excel(customers_file)
        df_depots = pd.read_excel(depots_file)
        df_vehicles = pd.read_excel(vehicles_file)
        print("✅ Đã đọc xong 3 file Excel")
    except Exception as e:
        print(f"❌ Không thể đọc file Excel:\n{str(e)}")
        return
    
    # Hiển thị cấu trúc dữ liệu để xác định cột thành phố
    print("\n📊 Cấu trúc dữ liệu:")
    print(f"\nCustomers columns: {list(df_customers.columns)}")
    print(f"Depots columns: {list(df_depots.columns)}")
    print(f"Vehicles columns: {list(df_vehicles.columns)}")
    
    # Hiển thị một vài dòng mẫu
    print("\n📋 Mẫu dữ liệu Customers (5 dòng đầu):")
    print(df_customers.head())
    print("\n📋 Mẫu dữ liệu Depots (5 dòng đầu):")
    print(df_depots.head())
    
    # Tìm cột chứa thông tin thành phố
    # Thử các tên cột phổ biến (case-insensitive)
    city_col_candidates = ['City', 'city', 'City_Name', 'city_name', 'Thành phố', 'Thành Phố', 
                          'Province', 'province', 'Tỉnh', 'Location', 'location', 'Address', 'address',
                          'Địa chỉ', 'Địa Chỉ', 'Area', 'area', 'Region', 'region']
    
    city_col_customers = None
    city_col_depots = None
    
    # Tìm trong customers (case-insensitive)
    for candidate in city_col_candidates:
        for col in df_customers.columns:
            if str(col).strip().lower() == candidate.lower():
                city_col_customers = col
                break
        if city_col_customers:
            break
    
    # Tìm trong depots (case-insensitive)
    for candidate in city_col_candidates:
        for col in df_depots.columns:
            if str(col).strip().lower() == candidate.lower():
                city_col_depots = col
                break
        if city_col_depots:
            break
    
    # Nếu không tìm thấy, thử tìm các cột có chứa từ khóa "city", "thành phố", "tỉnh"
    if not city_col_customers:
        for col in df_customers.columns:
            col_lower = str(col).lower()
            if any(keyword in col_lower for keyword in ['city', 'thành phố', 'tỉnh', 'province', 'location']):
                city_col_customers = col
                break
    
    if not city_col_depots:
        for col in df_depots.columns:
            col_lower = str(col).lower()
            if any(keyword in col_lower for keyword in ['city', 'thành phố', 'tỉnh', 'province', 'location']):
                city_col_depots = col
                break
    
    # Nếu vẫn không tìm thấy, hiển thị lỗi
    if not city_col_customers:
        error_msg = "⚠️ Không tìm thấy cột thành phố tự động trong customers.\n"
        error_msg += f"Các cột có sẵn: {', '.join(df_customers.columns)}\n"
        error_msg += "Vui lòng kiểm tra lại cấu trúc dữ liệu."
        print(f"\n{error_msg}")
        return
    
    if not city_col_depots:
        error_msg = "⚠️ Không tìm thấy cột thành phố tự động trong depots.\n"
        error_msg += f"Các cột có sẵn: {', '.join(df_depots.columns)}\n"
        error_msg += "Vui lòng kiểm tra lại cấu trúc dữ liệu."
        print(f"\n{error_msg}")
        return
    
    print(f"\n✅ Sử dụng cột thành phố:")
    print(f"   - Customers: {city_col_customers}")
    print(f"   - Depots: {city_col_depots}")
    
    # Lấy danh sách thành phố từ customers và depots
    cities_customers = df_customers[city_col_customers].dropna().unique()
    cities_depots = df_depots[city_col_depots].dropna().unique()
    all_cities = sorted(set(list(cities_customers) + list(cities_depots)))
    
    print(f"\n🏙️ Tìm thấy {len(all_cities)} thành phố: {', '.join(map(str, all_cities))}")
    
    # Tách dữ liệu theo từng thành phố
    print("\n" + "="*60)
    print("🔄 Đang tách dữ liệu theo thành phố...")
    
    for city in all_cities:
        city_name = str(city).strip()
        # Tạo tên folder an toàn (loại bỏ ký tự đặc biệt)
        safe_city_name = "".join(c for c in city_name if c.isalnum() or c in (' ', '-', '_')).strip()
        safe_city_name = safe_city_name.replace(' ', '_')
        
        city_folder = output_base / safe_city_name
        city_folder.mkdir(exist_ok=True)
        
        # Tách customers theo thành phố
        df_city_customers = df_customers[df_customers[city_col_customers] == city].copy()
        
        # Tách depots theo thành phố
        df_city_depots = df_depots[df_depots[city_col_depots] == city].copy()
        
        # Tách vehicles theo depot của thành phố
        if len(df_city_depots) > 0:
            # Tìm cột Depot_ID trong depots (case-insensitive)
            depot_id_col = None
            for col in df_city_depots.columns:
                col_lower = str(col).lower()
                if 'depot' in col_lower and 'id' in col_lower:
                    depot_id_col = col
                    break
            
            if not depot_id_col:
                # Thử tìm cột có tên chứa "depot" hoặc "id"
                for col in df_city_depots.columns:
                    col_lower = str(col).lower()
                    if 'depot' in col_lower:
                        depot_id_col = col
                        break
                if not depot_id_col:
                    depot_id_col = df_city_depots.columns[0]  # Fallback: dùng cột đầu tiên
            
            # Chuẩn hóa Depot_ID từ depots
            depot_ids = set(df_city_depots[depot_id_col].astype(str).str.strip().str.upper())
            
            # Tìm cột depot trong vehicles (case-insensitive)
            depot_col_start = None
            depot_col_end = None
            
            # Danh sách các tên cột có thể
            start_candidates = ['Start_Depot_ID', 'start_depot_id', 'StartDepotID', 'Depot_ID_Start', 
                               'StartDepot', 'start_depot', 'From_Depot', 'from_depot']
            end_candidates = ['End_Depot_ID', 'end_depot_id', 'EndDepotID', 'Depot_ID_End',
                            'EndDepot', 'end_depot', 'To_Depot', 'to_depot']
            
            # Tìm cột start (case-insensitive)
            for candidate in start_candidates:
                for col in df_vehicles.columns:
                    if str(col).strip().lower() == candidate.lower():
                        depot_col_start = col
                        break
                if depot_col_start:
                    break
            
            # Nếu không tìm thấy exact match, tìm partial match
            if not depot_col_start:
                for col in df_vehicles.columns:
                    col_lower = str(col).lower()
                    if any(keyword in col_lower for keyword in ['start', 'depot']) and 'end' not in col_lower:
                        depot_col_start = col
                        break
            
            # Tìm cột end (case-insensitive)
            for candidate in end_candidates:
                for col in df_vehicles.columns:
                    if str(col).strip().lower() == candidate.lower():
                        depot_col_end = col
                        break
                if depot_col_end:
                    break
            
            # Nếu không tìm thấy exact match, tìm partial match
            if not depot_col_end:
                for col in df_vehicles.columns:
                    col_lower = str(col).lower()
                    if any(keyword in col_lower for keyword in ['end', 'depot']) and 'start' not in col_lower:
                        depot_col_end = col
                        break
            
            if depot_col_start and depot_col_end:
                df_vehicles_temp = df_vehicles.copy()
                df_vehicles_temp['Start_Depot_ID_clean'] = df_vehicles_temp[depot_col_start].astype(str).str.strip().str.upper()
                df_vehicles_temp['End_Depot_ID_clean'] = df_vehicles_temp[depot_col_end].astype(str).str.strip().str.upper()
                df_city_vehicles = df_vehicles_temp[
                    (df_vehicles_temp['Start_Depot_ID_clean'].isin(depot_ids)) |
                    (df_vehicles_temp['End_Depot_ID_clean'].isin(depot_ids))
                ].copy()
                df_city_vehicles = df_city_vehicles.drop(columns=['Start_Depot_ID_clean', 'End_Depot_ID_clean'])
            else:
                print(f"   ⚠️ Không tìm thấy cột depot trong vehicles cho {city_name}")
                df_city_vehicles = pd.DataFrame()
        else:
            df_city_vehicles = pd.DataFrame()
        
        # Lưu file Excel cho từng thành phố (chỉ lưu nếu có dữ liệu)
        customers_output = city_folder / "customers.xlsx"
        depots_output = city_folder / "depots.xlsx"
        vehicles_output = city_folder / "vehicles.xlsx"
        
        try:
            if len(df_city_customers) > 0:
                df_city_customers.to_excel(customers_output, index=False, engine='openpyxl')
            else:
                # Tạo file rỗng nếu không có dữ liệu
                pd.DataFrame().to_excel(customers_output, index=False, engine='openpyxl')
            
            if len(df_city_depots) > 0:
                df_city_depots.to_excel(depots_output, index=False, engine='openpyxl')
            else:
                # Tạo file rỗng nếu không có dữ liệu
                pd.DataFrame().to_excel(depots_output, index=False, engine='openpyxl')
            
            if len(df_city_vehicles) > 0:
                df_city_vehicles.to_excel(vehicles_output, index=False, engine='openpyxl')
            else:
                # Tạo file rỗng nếu không có dữ liệu
                pd.DataFrame().to_excel(vehicles_output, index=False, engine='openpyxl')
        except ImportError:
            # Nếu không có openpyxl, thử dùng xlsxwriter hoặc mặc định
            try:
                if len(df_city_customers) > 0:
                    df_city_customers.to_excel(customers_output, index=False)
                else:
                    pd.DataFrame().to_excel(customers_output, index=False)
                
                if len(df_city_depots) > 0:
                    df_city_depots.to_excel(depots_output, index=False)
                else:
                    pd.DataFrame().to_excel(depots_output, index=False)
                
                if len(df_city_vehicles) > 0:
                    df_city_vehicles.to_excel(vehicles_output, index=False)
                else:
                    pd.DataFrame().to_excel(vehicles_output, index=False)
            except Exception as e:
                print(f"   ❌ Lỗi khi lưu file cho {city_name}: {str(e)}")
                continue
        
        print(f"\n✅ {city_name}:")
        print(f"   - Customers: {len(df_city_customers)} dòng")
        print(f"   - Depots: {len(df_city_depots)} dòng")
        print(f"   - Vehicles: {len(df_city_vehicles)} dòng")
        print(f"   - Lưu tại: {city_folder}")
    
    print("\n" + "="*60)
    print(f"✅ Hoàn thành! Đã tách dữ liệu cho {len(all_cities)} thành phố.")
    print(f"📂 Dữ liệu được lưu tại: {output_base}")

if __name__ == "__main__":
    split_excel_by_city()
