import sys
import cv2
import os
import numpy as np
from matplotlib import pyplot as plt
from array import array

if __name__ == '__main__':
    gtPath = "../build/gt.csv"
    gt = np.loadtxt(gtPath, dtype=np.float)

    resPath = "../build/res.csv"
    res = np.loadtxt(resPath, dtype=np.float)

    # plt.plot(gt[:, 1], gt[:,0])
    # plt.plot(res[:, 1], res[:,0])

    # plt.plot(gt[:, 0])
    # plt.plot(gt[:, 1])
    # plt.plot(res[:, 0])
    # plt.plot(res[:, 1])

    # plt.plot(gt[:, 2])
    # plt.plot(gt[:, 3])
    # plt.plot(res[:, 2])
    # plt.plot(res[:, 3])

    # plt.plot(res[:, 6])

    plt.plot(gt[:, 3])
    plt.plot(res[:, 3])

    # ax = plt.gca()
    # ax.set_aspect(1)
    plt.show()
