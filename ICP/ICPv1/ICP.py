import numpy as np
import math
import time
import pcl
from sklearn.neighbors import NearestNeighbors

def nearest_neighbor(src, dst):
    '''
    Find the nearest (Euclidean) neighbor in dst for each point in src
    Input:
        src: Nxm array of points
        dst: Nxm array of points
    Output:
        distances: Euclidean distances of the nearest neighbor
        indices: dst indices of the nearest neighbor
    '''

    # assert src.shape == dst.shape

    neigh = NearestNeighbors(n_neighbors=1)
    neigh.fit(dst)       #拟合
    distances, indices = neigh.kneighbors(src, return_distance=True)
    return distances.ravel(), indices.ravel()

def rotation_matrix(axis, theta):
    axis = axis/np.sqrt(np.dot(axis, axis))
    a = np.cos(theta/2.)
    b, c, d = -axis*np.sin(theta/2.)

    return np.array([[a*a+b*b-c*c-d*d, 2*(b*c-a*d), 2*(b*d+a*c)],
                  [2*(b*c+a*d), a*a+c*c-b*b-d*d, 2*(c*d-a*b)],
                  [2*(b*d-a*c), 2*(c*d+a*b), a*a+d*d-b*b-c*c]])

def rotationMatrixToEulerAngles(R):

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    return np.array([x, y, z])


def SVDslove(pts1, pts2):

    assert pts1.shape == pts2.shape

    N = pts1.shape[0]    # 10
    m = pts1.shape[1]    # 3
    # print('N', N, '\n', 'm', m)
    #定义质心
    p1, p2 = [0, 0, 0], [0, 0, 0]    #(1,3)
    for i in range(0, N):
        p1 += pts1[i]
        p2 += pts2[i]
    # print('p1:', p1)
    # print('p2:', p2)
    p1 = p1/N     #(1,3)
    p2 = p2/N
    # p1 = np.mean(pts1, axis=0)
    # p2 = np.mean(pts2, axis=0)
    # print('p1:', p1)
    # print('p2:', p2)


    #去质心坐标    (3,10)
    q1 = pts1 - p1
    q2 = pts2 - p2
    # print('q1:', q1)
    # print('q2:', q2)

    #svd求U， VT
    # W = np.zeros((3, 3))
    # # print(W)
    # for i in range(0, N):
    #     W = W + np.dot(np.transpose(q1[i]), q2[i])  #(3,1)*(1,3)=(3,3)
    #     print(W)
    # print('W = q1T*q2:', W)   #(3,3)
    W = np.dot(q1.T, q2)
    # print('W = q1T*q2:', W)   #(3,3)

    U, sigma, VT = np.linalg.svd(W)   #U(3,3)  VT(3,3)
    # print(np.dot(U, U.T))
    # print(np.dot(VT, VT.T))
    # print(np.dot(np.dot(U, sigma), VT))
    # print('U:', U, '\n', 'sigma:', sigma, '\n', 'VT:', VT)

    R = np.dot(VT.T, U.T)    #R(3,3)

    if np.linalg.det(R) < 0:
        VT[m-1, :] *= -1
        R = np.dot(VT.T, U.T)

    print('R:', R)
    Rotation = rotationMatrixToEulerAngles(R)
    print('Rotation-->EulerAngles', Rotation)

    t = p2.T - np.dot(R, p1.T)   #(1,3)
    print('t:', t)

    T = np.identity(m + 1)
    T[:m, :m] = R
    T[:m, m] = t
    # print('T:', T)
    return T, R, t


def ICP(pts1, pts2, max_iterations=20, tolerance=0.001, k=1.5):
    # NumPoint = min(pts1.shape[0], pts2.shape[0])
    m = pts1.shape[1]

    src = np.ones((m+1, pts1.shape[0]))    #(4,12)
    dst = np.ones((m+1, pts2.shape[0]))    #(4, 11)
    # print('src:', src, '\n', 'dst:', dst)
    src[:m, :] = np.copy(pts1.T)   #列取到m，行取所有    （4, 12）    第四列为1
    dst[:m, :] = np.copy(pts2.T)          #（4, 11）                     第四列为1
    # print('src:', src, '\n', 'dst:', dst)

    prev_error = 0

    for i in range(max_iterations):

        # 找出当前源点和目的点之间最近的邻居
        # distances, indices = nearest_neighbor(src[:m, :].T, dst[:m, :].T)
        distances, indices = nearest_neighbor(src[:m, :].T, dst[:m, :].T)    #(12,3 ) (11, 3)
        # print('src[:m, :].T:', src[:m, :].T, '\n', 'dst[:m, :].T:', dst[:m, :].T)   #(12,3 ) (12, 3)
        print('distance:', distances, '\n', 'indices:', indices)

#         # compute the transformation between the current source and nearest destination points
#         #计算当前源点和最近目的地点之间的转换
        mean_error = np.mean(distances)
        print('mean_error:', mean_error)

        # if i == 0:
        j = 0
        pts11 = []
        pts22 = []

        # for i in range(0, pts1.shape[0]):
        for i in range(0, src[:m, :].T.shape[0]):    #迭代12次
            if distances[i] <= mean_error * k:
                # print('第', i, indices[i], '对匹配')
                j = j + 1

                # pts11 = np.append(pts11, pts1[i])
                # pts22 = np.append(pts22, pts2[indices[i]])
                pts11 = np.append(pts11, src[:m, :].T[i])
                pts22 = np.append(pts22, dst[:m, :].T[indices[i]])
        # pts1 = pts11.reshape(j, 3)
        # pts2 = pts22.reshape(j, 3)
        # src[:m, :] = pts11.reshape(j, 3).T
        # dst[:m, :] = pts22.reshape(j, 3).T
        pts11 = pts11.reshape(j, 3)
        pts22 = pts22.reshape(j, 3)
        print('成功匹配：', j, '对点')
        # print('pts11:', pts11, '\n', 'pts22:', pts22)
        # T, R, t = SVDslove(pts1=src[:m, :].T, pts2=dst[:m, :].T)
        T, R, t = SVDslove(pts1=pts11, pts2=pts22)

        # update the current source
        # pts2 = np.dot(R, pts2.T).T + t
        # pts1 = np.dot(R, pts2.T).T + t
        # pts2 = np.dot(R, (pts2 + t).T).T
        src = np.dot(T, src)

       # check error
#         mean_error = np.mean(distances)
        print('迭代误差:', np.abs(prev_error - mean_error))
        if np.abs(prev_error - mean_error) < tolerance:
             break
        prev_error = mean_error

    T, R, t = SVDslove(pts1, src[:m, :].T)


    return R, t



if __name__ == '__main__':
    rotation = 0.5
    noise_sigmod = 0.01
    Tk = 1
    N = 10
    m = 3
    noise = np.random.randn(N, m)*noise_sigmod   #高斯噪声

    # pts1 = np.ones((N, m))   #(N,m)
    pts1 = np.random.rand(N, m)    #(N,m)
    pts1 = pts1 * 10
    pts2 = np.copy(pts1)
    t = np.random.rand(m)*Tk
    print('t:', t)
    R = rotation_matrix(np.random.rand(m)*10, np.random.rand()*rotation)
    print('R:', R)
    Rotation = rotationMatrixToEulerAngles(R)
    print('Rotation-->EulerAngles', Rotation)
    print('***************************************************************************', '\n'
          '***************************************************************************')
    pts2 = np.dot(R, pts2.T).T + t
    # pts2 = np.dot(R, (pts2 + t).T).T
    pts2 = pts2 + noise

    #定义pts1 pts2  10个匹配
    pts1 = np.append(pts1, [0.4, 0.6, 0.8])
    pts1 = np.append(pts1, [1.2, 1.7, 1.7]).reshape(12, 3)
    pts2 = np.append(pts2, [1.1, 1.9, 1.9]).reshape(11, 3)
    print('pts1:', pts1)
    print('pts2:', pts2)
    np.random.shuffle(pts2)

    time_start = time.time()
    # R,  t = SVDslove(pts1, pts2)
    # R,  t = pose_estimation3d3d(pts2, pts1)
    ICP(pts1, pts2)
    time_end = time.time()
    print('cost time:', time_end-time_start, 's')
    # pose_estimation3d3d(pts1, pts2)
    # print(np.linalg.pinv(R)) #(pts2, pts1) (pts1, pts2)逆的关系












