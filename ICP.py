import numpy as np
from lidar_map import LidarMap

	# int iter;
	# for (iter = 0; iter < MAX_ITERATIONS; iter++) {
	# 	closest(A_move.transpose(), B, NEAREST_K);

	# 	//std::cout << A_move.transpose().rows() << ", " << A_move.transpose().cols() << std::endl << std::endl;
	# 	//std::cout << B.rows() << ", " << B.cols() << std::endl << std::endl;

	# 	for (int j = 0; j < row; j++) {
	# 		choice.block<2, 1>(0, j) = B.block<1, 2>(mPtPairsIndex[j], 0);
	# 	}

	# 	T = fit_transform(A_move.transpose(), choice.transpose());
	# 	A_temp = T * A_temp;
		
	# 	for (int j = 0; j < row; j++) {
	# 		A_move.block<2, 1>(0, j) = A_temp.block<2, 1>(0, j);
	# 	}

	# 	error = std::accumulate(mPtPairsDistance.begin(), mPtPairsDistance.end(), 0.0) / mPtPairsDistance.size();
		
	# 	if(error < 10000){
	# 		if (std::abs(prev_error - error) < EPS) {
	# 			break;
	# 		}
	# 	}

	# 	prev_error = error;
	# }

class ICP:

    def __init__(self, maxIter = 10, epcilon=0.01):
        self.maxIter = maxIter
        self.epcilon = epcilon

    def estimate(self, points, lidarMap, initR, initT):
        movedLS = lidarMap.getLineSegments(initR, initT)

        prev_error = 0.0

        for iter in range(self.maxIter):
            # print("{}/{}".format(iter, self.maxIter))

            nearest = lidarMap.calcNearestPointsInMap(points, movedLS)

            R, T = self.fitTransform(nearest, points) # 逆？
            # R, T = self.fitTransform(points, nearest)

            # update movedLS 
            movedLS = lidarMap.calcLineSegments(movedLS, R, T)

            error, std = lidarMap.calcError(nearest, points) # 可能ならstd利用して外れ値を除外したい

            # print("error:{}, std:{}".format(error, std))

            if np.abs(error - prev_error) < self.epcilon:
                break

            prev_error = error

        print("error:{}, std:{}".format(error, std))

        R, T = self.fitTransform(lidarMap.getLineSegments().reshape((-1,2)), movedLS.reshape((-1,2)))

        return R, T

    def estimate2(self, points, lidarMap, initR, initT):
        movedLS = lidarMap.getLineSegments(initR, initT)

        prev_error = 0.0

        for iter in range(self.maxIter):
            # print("{}/{}".format(iter, self.maxIter))

            nearest = lidarMap.calcNearestPointsInMap(points, movedLS)

            R, T = self.fitTransform(nearest, points) # 逆？
            # R, T = self.fitTransform(points, nearest)

            # update movedLS 
            movedLS = lidarMap.calcLineSegments(movedLS, R, T)

            error, std = lidarMap.calcError(nearest, points) # 可能ならstd利用して外れ値を除外したい

            # print("error:{}, std:{}".format(error, std))

            if np.abs(error - prev_error) < self.epcilon:
                break

            prev_error = error
            
        prev_error = 0.0

        for iter in range(self.maxIter):
            # print("{}/{}".format(iter, self.maxIter))

            nearest, index = lidarMap.calcNearestPointsInMap2(points, movedLS)

            R, T = self.fitTransform(nearest, points[index]) # 逆？
            # R, T = self.fitTransform(points, nearest)

            # update movedLS 
            movedLS = lidarMap.calcLineSegments(movedLS, R, T)

            error, std = lidarMap.calcError(nearest, points[index]) # 可能ならstd利用して外れ値を除外したい

            # print("error:{}, std:{}".format(error, std))

            if np.abs(error - prev_error) < self.epcilon:
                break

            prev_error = error

        print("error:{}, std:{}".format(error, std))

        R, T = self.fitTransform(lidarMap.getLineSegments().reshape((-1,2)), movedLS.reshape((-1,2)))

        return R, T



    def fitTransform(self, A, B):
        meanA = np.mean(A, axis=0)
        meanB = np.mean(B, axis=0)

        nA = A - meanA
        nB = B - meanB

        sigma = self.cov(nA, nB)
        U, S, V = np.linalg.svd(sigma)

        R = V @ U.transpose()
        T = meanB - R @ meanA

        return R, T

    def cov(self, A, B):
        return np.dot(A.transpose(), B) / A.shape[0]

if __name__ == '__main__':
    from utils import Converter, readLidarFile
    from lidar_map import LidarMap, getLineSegments1

    lineSegments =  getLineSegments1()
    lidarMap = LidarMap(lineSegments)

    # data = readLidarFile("lidar_10_long_fixed.txt", delimiter='\t')
    data = readLidarFile("lidar_25_long_fixed.txt", delimiter='\t')

    converter = Converter(deg_offset = -45)

    icp = ICP()

    from utils import calcR
    R = calcR(0.0)
    T = np.array([[-1.0], [-1.0]])

    R2 = calcR(0.0)
    T2 = np.array([[-1.0], [-1.0]])
    # print(lidarMap.getLineSegments(R, -T))
    # print(lidarMap.getLineSegments(np.linalg.inv(R), -T))

    # movedLS = lidarMap.getLineSegments()
    # print(movedLS.shape)
    # print(movedLS)

    # for i in range(3):
    #     movedLS = lidarMap.calcLineSegments(movedLS, R, T)
    #     print(movedLS.shape)
    #     print(movedLS)

    from draw_lidar import DrawLidar
    drawLidar = DrawLidar(maxLen = 4.0)
    drawLidar2 = DrawLidar(maxLen = 4.0)

    results = []
    results2 = []

    for dist in data:
        drawLidar.clear()
        drawLidar2.clear()

        xy = converter.dist_to_xy(dist)
        # nearest = lidarMap.calcNearestPointsInMap(xy, lineSegments)
        
        # icp.fitTransform(xy, nearest)

        # R, T = icp.estimate(xy, lidarMap, R, T)
        R, T = icp.estimate(xy, lidarMap, R, T)
        R2, T2 = icp.estimate2(xy, lidarMap, R2, T2)
        # print(R)
        # print("deg:", np.rad2deg(np.arctan2(R[0,0], R[1,0])))
        # print("m:", T)
        # print()

        results += [T]
        results2 += [T2]

        drawLidar.drawMap(lidarMap.getLineSegments())
        drawLidar.drawMap(lidarMap.getLineSegments(R, T))
        drawLidar.draw_points(xy)

        drawLidar2.drawMap(lidarMap.getLineSegments())
        drawLidar2.drawMap(lidarMap.getLineSegments(R2, T2))
        drawLidar2.draw_points(xy)

        # drawLidar.update(0.01)
        drawLidar.update(0.5)
        drawLidar2.update(0.5)

    for i in range(len(results)-1):
        diff = results[i+1]-results[i]
        mm = np.sqrt(np.sum(diff**2))*1000

        diff2 = results2[i+1]-results2[i]
        mm2 = np.sqrt(np.sum(diff2**2))*1000
        
        # print('{:>12.5f}'.format(mm))
        print('{:>12.5f},   {:>12.5f}'.format(mm, mm2))