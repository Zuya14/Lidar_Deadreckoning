import numpy as np
import matplotlib.pyplot as plt

from utils import Converter, readLidarFile
from lidar_map import LidarMap, getLineSegments0

class DrawLidar:

    def __init__(self, maxLen=None):

        self.fig, self.ax = plt.subplots()

        if maxLen:
            self.ax.set_xlim(-maxLen, maxLen)
            self.ax.set_ylim(-maxLen, maxLen)

        self.ax.set_aspect('equal')
        self.ax.grid()

        self.maxLen = maxLen
    
    def update(self, interval=0.01):
        plt.pause(interval)

    def clear(self):
        self.ax.lines.clear()
        self.ax.collections.clear()

    def draw_points(self, points, psize=1, color='red', marker='o', alpha=1):
        p = points.reshape((-1, 2))
        self.ax.scatter(p[:,0], p[:,1], s=psize, c=color, marker=marker, alpha=alpha)

    def draw_lines(self, points, psize=1, color='red', linestyle='solid'):
        p = points.reshape((-1, 2))
        self.ax.plot(p[:,0], p[:,1], ms=psize, c=color, marker="o", linestyle=linestyle)

    def drawMap(self, lineSegments, color='green', linewidth=1):
        for points in lineSegments:
            s = points[0]
            e = points[1]
            self.ax.plot([s[0], e[0]],[s[1], e[1]], c=color, linewidth=linewidth)
        

if __name__ == '__main__':
    lidarMap = LidarMap(getLineSegments0())

    data = readLidarFile("lidar_10_long_fixed.txt", delimiter='\t')

    converter = Converter(deg_offset = -45)
    drawLidar = DrawLidar(maxLen = 4.0)

    drawLidar.drawMap(lidarMap.getLineSegments())

    for dist in data:
        drawLidar.clear()

        xy = converter.dist_to_xy(dist)

        nearest = lidarMap.calcNearestPointsInMap(xy, lidarMap.getLineSegments())
        
        drawLidar.draw_points(nearest, psize=2, color='blue')
        drawLidar.draw_points(xy)

        drawLidar.update(0.1)