# excel_to_csv_batch.py
import pandas as pd
from pathlib import Path

# Danh sách đường dẫn Excel (thay đổi nếu cần)
excel_paths = [
    r"D:\A UEH_UNIVERSITY\RESEACH\OR_AI_paper\Zzz_data\LMDO processed\Ho_Chi_Minh_City\customers.xlsx",
    r"D:\A UEH_UNIVERSITY\RESEACH\OR_AI_paper\Zzz_data\LMDO processed\Ho_Chi_Minh_City\depots.xlsx",
    r"D:\A UEH_UNIVERSITY\RESEACH\OR_AI_paper\Zzz_data\LMDO processed\Ho_Chi_Minh_City\roads.xlsx",
    r"D:\A UEH_UNIVERSITY\RESEACH\OR_AI_paper\Zzz_data\LMDO processed\Ho_Chi_Minh_City\vehicles.xlsx",
]

# Nếu muốn xuất ra một thư mục khác, đặt ở đây; để None để xuất vào cùng thư mục file Excel
out_dir = None  # ví dụ: r"D:\output_csvs"

def excel_to_csv(excel_path: str, out_dir: str | None = None, encoding: str = "utf-8-sig"):
    p = Path(excel_path)
    if not p.exists():
        print(f"[SKIP] File not found: {excel_path}")
        return

    # Đọc tất cả sheet; sheet_name=None -> trả về dict sheet_name -> DataFrame
    try:
        sheets = pd.read_excel(p, sheet_name=None, engine="openpyxl")
    except Exception as e:
        print(f"[ERROR] Không đọc được file {excel_path}: {e}")
        return

    target_dir = Path(out_dir) if out_dir else p.parent
    target_dir.mkdir(parents=True, exist_ok=True)

    for sheet_name, df in sheets.items():
        # Tạo tên file an toàn: thay khoảng trắng và ký tự đặc biệt
        safe_sheet = "".join(c if c.isalnum() or c in ("_", "-") else "_" for c in sheet_name).strip("_")
        csv_name = f"{p.stem}__{safe_sheet}.csv"
        out_path = target_dir / csv_name

        try:
            df.to_csv(out_path, index=False, encoding=encoding)
            print(f"[OK] {out_path}")
        except Exception as e:
            print(f"[ERROR] Ghi {out_path} thất bại: {e}")

if __name__ == "__main__":
    for xp in excel_paths:
        excel_to_csv(xp, out_dir)
    print("Hoàn tất.")
