import numpy as np
from utils import Converter, readLidarFile, calcR
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

    data = readLidarFile("lidar_10_long_fixed.txt", delimiter='\t')
  
    converter = Converter(deg_offset = -45)

    icp = ICP(maxIter = 50, epcilon=1e-2)

    R = calcR(0.0)
    T = np.array([[1.0], [1.0]])

    pos = []
    dtm = []
    rnd = []

    for dist in data:
        xy = converter.dist_to_xy(dist)

        R, T = icp.estimate(xy, lidarMap, R.T, -T)

        pos.append([T[0], T[1]])

        if random.random() < 0.2:
            rnd.append([T[0], T[1]])
        else:
            dtm.append([T[0], T[1]])

    pos = np.array(pos)
    print(pos)

    dtm = np.array(dtm)
    rnd = np.array(rnd)

    drawLidar.draw_lines(pos, color='blue')
    drawLidar.draw_points(dtm, color='blue', psize=50, alpha=0.5)
    drawLidar.draw_points(rnd, color='red', psize=50, alpha=0.5)

    drawLidar.update(-1)
