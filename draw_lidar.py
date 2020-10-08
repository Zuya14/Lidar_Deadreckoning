import numpy as np
import matplotlib.pyplot as plt
import sys
import os
from pathlib import Path

from utils import Converter, readLidarFile
from lidar_map import LidarMap, getLineSegments1

class DrawLidar:

    def __init__(self, maxLen):

        self.fig, self.ax = plt.subplots()

        self.ax.set_xlim(-maxLen, maxLen)
        self.ax.set_ylim(-maxLen, maxLen)
        self.ax.grid()

        self.maxLen = maxLen
    
    def update(self, interval=0.01):
        plt.pause(interval)

    def clear(self):
        self.ax.collections.clear()

    def draw_points(self, points, psize=1, color='red'):
        p = points.reshape((-1, 2))
        self.ax.scatter(p[:,0], p[:,1], s=psize, c=color)

    def drawMap(self, lidarMap, color='brown'):
        for points in lidarMap.lineSegments:
            s = points[0]
            e = points[1]
            self.ax.plot([s[0], e[0]],[s[1], e[1]], c=color)
        

if __name__ == '__main__':
    lineSegments =  getLineSegments1()
    lidarMap = LidarMap(lineSegments)

    data = readLidarFile("lidar_10_long_fixed.txt", delimiter='\t')

    converter = Converter(deg_offset = -45)
    drawLidar = DrawLidar(maxLen = 4.0)

    drawLidar.drawMap(lidarMap)

    for dist in data:
        drawLidar.clear()

        xy = converter.dist_to_xy(dist)

        nearest = lidarMap.calcNearestPointsInMap(xy, lineSegments)
        
        drawLidar.draw_points(nearest, psize=2, color='blue')
        drawLidar.draw_points(xy)

        drawLidar.update(0.1)