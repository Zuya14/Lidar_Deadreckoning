import numpy as np
from utils import Converter, readLidarFile, readLidarFile2, calcR
from lidar_map import LidarMap, getLineSegments0, getLineSegments2
from draw_lidar import DrawLidar
from ICP import ICP
import random

if __name__ == '__main__':

    lineSegments =  getLineSegments0()
    lidarMap = LidarMap(lineSegments)

    # drawLidar = DrawLidar(maxLen = 4.0)
    drawLidar = DrawLidar()
    
    drawLidar.drawMap(lidarMap.getLineSegments(), color='grey', linewidth=5)

    # data = readLidarFile("lidar_10_long_fixed.txt", delimiter='\t')
    flag, data = readLidarFile2("LiDAR.txt", delimiter=',')
    
    print(flag.shape)
    print(data.shape)

    # data = data * 0.01
    data = data * 10.0

    converter = Converter(deg_offset = -45)

    icp = ICP(maxIter = 50, epcilon=1e-2)

    R = calcR(0.0)
    T = np.array([[1.0], [1.0]])

    pos = []
    dtm = []
    rnd = []

    # for dist in data:
    for dist, f in zip(data, flag):
        xy = converter.dist_to_xy(dist)

        R, T = icp.estimate(xy, lidarMap, R.T, -T)

        pos.append([T[0], T[1]])

        if f > 0:
            rnd.append([T[0], T[1]])
        else:
            dtm.append([T[0], T[1]])

    pos = np.array(pos)
    print(pos)

    dtm = np.array(dtm)
    rnd = np.array(rnd)

    drawLidar.draw_lines(pos, color='blue')
    drawLidar.save("demo0.png")

    drawLidar.draw_points(dtm, color='blue', psize=50, alpha=0.5)
    drawLidar.save("demo1.png")

    drawLidar.draw_points(rnd, color='red', psize=50, alpha=0.9)
    drawLidar.save("demo2.png")

    drawLidar.clear()
    drawLidar.drawMap(lidarMap.getLineSegments(), color='grey', linewidth=5)
    drawLidar.draw_lines(pos, color='blue')
    drawLidar.draw_points(rnd, color='red', psize=50, alpha=0.9)
    drawLidar.save("demo3.png")

    # drawLidar.update(-1)
    drawLidar.update(0.1)
