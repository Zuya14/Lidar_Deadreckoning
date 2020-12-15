import sys
import os
from pathlib import Path

if __name__ == '__main__':

    assert len(sys.argv) == 2

    p = Path(sys.argv[1])

    if Path.is_dir(p):
        pass
    else:
        print("no file")
        exit()

    for i in range(1, 31):
        
        file_name = sys.argv[1] + str(i) + "/LiDAR.csv"
        
        with open(file_name) as f:
            data_lines = f.read()

        # 文字列置換
        data_lines = data_lines.replace(",\n", "\n")

        # 同じファイル名で保存
        with open(sys.argv[1] + "LiDAR" + str(i) + ".csv", mode="w") as f:
            f.write(data_lines)