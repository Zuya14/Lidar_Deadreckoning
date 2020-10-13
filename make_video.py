from pathlib import Path
import cv2

if __name__ == '__main__':
    p = Path()

    files = list(p.glob("output/normal*.png"))

    fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')  
    rows, cols, channels = cv2.imread("output/normal0.png").shape
    video = cv2.VideoWriter("output/ICP_normal.mp4", fourcc, 5, (cols,rows))  # 動画の仕様（ファイル名、fourcc, FPS, サイズ）
    rows, cols, channels = cv2.imread("output/filtered0.png").shape
    video2 = cv2.VideoWriter("output/ICP_filtered.mp4", fourcc, 5, (cols,rows))  # 動画の仕様（ファイル名、fourcc, FPS, サイズ）

    for i in range(len(files)):
        dst = cv2.imread("output/normal{}.png".format(i))
        dst2 = cv2.imread("output/filtered{}.png".format(i))
        video.write(dst)
        video2.write(dst2)