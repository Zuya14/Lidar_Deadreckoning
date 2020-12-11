import numpy as np
import sys
import os
from pathlib import Path

class Converter:

    def __init__(self, deg_offset=0):
        deg_range = 270
        data_num = 1081
        degs = np.arange(0, deg_range+0.01, deg_range/(data_num-1))

        self.rads = np.deg2rad(degs + deg_offset).reshape(-1,1)

    def dist_to_xy(self, distances):
        d = distances.reshape(-1,1)
        x = d * np.cos(self.rads)
        y = d * np.sin(self.rads)
        xy = np.concatenate([x, y], axis=1)

        return xy

def readLidarFile(filename, delimiter=','):
    p = Path(filename)
    assert Path.is_file(p)

    with open(p, newline='') as file:
        datas = np.loadtxt(file, delimiter=delimiter)

    return datas

def readLidarFile2(filename, delimiter=','):
    p = Path(filename)
    assert Path.is_file(p)

    with open(p, newline='') as file:
        datas = np.loadtxt(file, delimiter=delimiter)

    return datas[:,0], datas[:, 1:]

def calcR(deg):
    t = np.deg2rad(deg)
    R = np.array([[np.cos(t), -np.sin(t)],
                  [np.sin(t),  np.cos(t)]])
    return R