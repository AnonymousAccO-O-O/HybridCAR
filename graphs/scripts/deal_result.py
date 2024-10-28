import os
import pandas as pd
from openpyxl import Workbook

base_dir = '.' # fill in here, the root dir, where sub-dirs each contains result of a checker. 
sub_dirs = [d for d in os.listdir(base_dir) if os.path.isdir(os.path.join(base_dir, d))]
log_files = set()
for sub_dir in sub_dirs:
    files = [f for f in os.listdir(os.path.join(base_dir, sub_dir)) if os.path.isfile(os.path.join(base_dir, sub_dir, f))]
    log_files.update(f for f in files if f.endswith('.log'))


# 收集数据
data = {}
for log_file in log_files:
    log_data = []
    for sub_dir in sub_dirs:
        file_path = os.path.join(base_dir, sub_dir, log_file)
        if os.path.exists(file_path):
            res_path = file_path.replace(".log", ".res")
            with open(res_path, 'r') as g:
                content = g.readlines()
                if len(content) == 0:
                    values = [3600]
                else:
                    with open(file_path, 'r') as f:
                        lines = f.readlines()
                        values = [3600]
                        for line in lines:
                            if line.startswith("Total Time"):
                                total_time = float(line.replace("Total Time: ",""))
                                if total_time > 3550: # we set a longer wall time for the cluster to wait for the termination
                                    total_time = 3600
                                values = [total_time]
                                break
                log_data.append(values)
        else:
            log_data.append([3600])  # 如果文件不存在，则用None填充
    data[log_file.split('.')[0]] = log_data

# print(data)

wb = Workbook()
Names = [(0,"time")]

for index,name in Names:
    sheet = wb.create_sheet(title=name)
    # name, s1, s2, s3, ...
    header = ["name"] + sub_dirs
    sheet.append(header)
    
    for case in data:
        new_line = [case]
        dt = data[case]
        for i, _ in enumerate(sub_dirs):
            log = dt[i]
            d = log[index]
            new_line.append(d)
        sheet.append(new_line)
    
wb.save('result.xlsx')


