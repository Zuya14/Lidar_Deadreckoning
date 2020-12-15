import numpy as np
from utils import Converter, readLidarFile, readLidarFile2, calcR
from lidar_map import LidarMap, getLineSegments0, getLineSegments2
from draw_lidar import DrawLidar
from ICP import ICP

import sys
import os
from pathlib import Path

def test():
    lineSegments =  getLineSegments0()
    lidarMap = LidarMap(lineSegments)

    # drawLidar = DrawLidar(maxLen = 4.0)
    drawLidar = DrawLidar()
    
    drawLidar.drawMap(lidarMap.getLineSegments(), color='grey', linewidth=5)

    # data = readLidarFile("lidar_10_long_fixed.txt", delimiter='\t')
    flag, data = readLidarFile2("LiDAR.txt", delimiter=',')
    
    print(flag.shape)
    print(data.shape)

    data = data * 0.01
    # data = data * 10.0

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

def getPosPoints(filename, lidarMap):
    
    flag, data = readLidarFile2(filename, delimiter=',')
    
    data = data * 0.001
    # data = data * 10.0

    converter = Converter(deg_offset = -45)

    icp = ICP(maxIter = 50, epcilon=1e-2)

    R = calcR(0.0)
    # T = np.array([[1.0], [1.0]])
    T = np.array([[0.75], [0.75]])

    all_pos = []
    policy_pos = []
    rnd_action_pos = []

    for dist, f in zip(data, flag):
        xy = converter.dist_to_xy(dist)

        R, T = icp.estimate(xy, lidarMap, R.T, -T)

        all_pos.append([T[0], T[1]])

        if f == 0:
            policy_pos.append([T[0], T[1]])
        else:
            rnd_action_pos.append([T[0], T[1]])

    return np.array(all_pos), np.array(policy_pos), np.array(rnd_action_pos)

def plot(drawLidar, all_pos=None, policy_pos=None, rnd_action_pos=None):

    if all_pos is not None:
        drawLidar.draw_lines(all_pos, color='blue')

    if policy_pos is not None:
        drawLidar.draw_points(policy_pos, color='blue', psize=50, alpha=0.5)

    if rnd_action_pos is not None:
        drawLidar.draw_points(rnd_action_pos, color='red', psize=50, alpha=0.9)

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


    out_dir = "./result"

    if not os.path.exists(out_dir):
        os.makedirs(out_dir)

    lineSegments =  getLineSegments0()
    lidarMap = LidarMap(lineSegments)

    drawLidar = DrawLidar()
    
    drawLidar.drawMap(lidarMap.getLineSegments(), color='grey', linewidth=5)

    rnd_action_pos_array = []

    for f in files:
        all_pos, policy_pos, rnd_action_pos = getPosPoints(f, lidarMap)
        # plot(drawLidar, all_pos, policy_pos, rnd_action_pos)
        plot(drawLidar, all_pos, None, None)
        rnd_action_pos_array.append(rnd_action_pos)
        drawLidar.update(0.001)

    drawLidar.update(1)

    drawLidar.save(out_dir + "/all.png")

    drawLidar.clear()
    drawLidar.drawMap(lidarMap.getLineSegments(), color='grey', linewidth=5)

    for pos in rnd_action_pos_array:
        # drawLidar.draw_points(rnd_action_pos, color='red', psize=50, alpha=0.9)
        plot(drawLidar, None, None, pos)

    drawLidar.update(1)

    drawLidar.save(out_dir + "/random_pos.png")
