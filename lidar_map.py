import numpy as np

'''
lineSegments = [
    [[x0 y0],[x1,y1]]
    ...
]
'''

def getLineSegments1():
    ls = np.array(
        [
            [[0.0 ,0.0],[3.0, 0.0]],
            [[0.0, 0.0],[0.0, 3.0]],
            [[3.0, 0.0],[3.0, 3.0]],
            [[0.0, 3.0],[3.0, 3.0]],
            [[1.5, 1.5],[3.0, 1.5]]
        ]
        )
    return ls


class LidarMap:

    def __init__(self, lineSegments):
        self.lineSegments = lineSegments

    def calcNearestPointsInMap(self, points, lineSegments):
        nearest_points = np.array([self.calcNearestPointInMap(pt, lineSegments) for pt in points])
        return nearest_points

    def calcNearestPointInMap(self, point, lineSegments):
        near_points = [self.calcNearestPoint(point, ls) for ls in lineSegments]
        error = [self.calcSquaredError(point, p) for p in near_points]

        nearest_point = near_points[error.index(min(error))]
        return nearest_point        

    # ある点に対する線分上の最近点を返す
    def calcNearestPoint(self, point, lineSegment):
        vecA = self.ls2Vec(lineSegment)
        vecB = self.pts2Vec(lineSegment[0], point)
        
        dot_product = np.dot(vecA, vecB)

        if dot_product > 0:
            vecA_len = self.calcVecLen(vecA)
            proj_len = dot_product / vecA_len

            if proj_len < vecA_len:
                proj_vec = vecA * (proj_len / vecA_len)
                return lineSegment[0] + proj_vec
            else:
                return lineSegment[1]
        else:
            return lineSegment[0]

    def ls2Vec(self, lineSegment):
        start = lineSegment[0]
        end   = lineSegment[1]
        vec = end - start
        return vec

    def pts2Vec(self, start, end):
        vec = end - start
        return vec

    def calcVecLen(self, vec):
        return np.sqrt((vec[0]**2) + (vec[1]**2))

    def calcSquaredError(self, p1, p2):
        diff = p1-p2
        return np.sum(diff**2)

    def calcDistance(self, p1, p2):
        return np.sqrt(self.calcSquaredError(p1, p2))

if __name__ == '__main__':
    lineSegments =  getLineSegments1()
    lidarMap = LidarMap(lineSegments)

    point = np.array([2,1])
    neatPt = lidarMap.calcNearestPointInMap(point, lineSegments)
    print(neatPt)