import sys
import os
from pathlib import Path

if __name__ == '__main__':

    assert len(sys.argv) == 2

    p = Path(sys.argv[1])

    if Path.is_dir(p):
        files = list(p.glob("*.csv"))
    elif Path.is_file(p):
        files = [p]
    else:
        print("no file")
        exit()

    print([f.stem for f in files])

    for file_name in files:
        with open(file_name, encoding="cp932") as f:
            data_lines = f.read()

        # 文字列置換
        data_lines = data_lines.replace(",\n", "\n")

        # 同じファイル名で保存
        with open(file_name.stem + "_fixed.csv", mode="w") as f:
            f.write(data_lines)
