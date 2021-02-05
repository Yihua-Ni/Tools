import sys
import cv2
import os
import numpy as np
from matplotlib import pyplot as plt
from array import array
import csv

if __name__ == '__main__':
    gtPath = "../build/test2.csv"
    gt = np.loadtxt(gtPath, dtype=np.float)

    # plt.plot(gt[:, 1], gt[:,0])
    # plt.plot(gt[:, 0], gt[:,1])
    # plt.plot(gt[:, 0], gt[:,2])
    # plt.plot(gt[:, 0], gt[:,3])
    plt.plot(gt[500:5000, 0], gt[500:5000,4], label = 'Horizontal Error (m)')
    plt.plot(gt[500:5000, 0], gt[500:5000,5], label = 'Vertical Error (m)')

    print(np.mean(np.abs(gt[500:5000,4])))
    print(np.mean(np.abs(gt[500:5000,5])))

    plt.ylabel('error (m)')
    plt.xlabel('frame id')
    plt.title('VIO Lane Analysis')
    plt.legend()
    plt.show()

    f = open('./result.csv','w',encoding='utf-8')
    csv_writer = csv.writer(f)
    csv_writer.writerow(["Frame id","Horizontal Error (m)","Vertical Error (m)"])

    for i in range(500, 5000):
       csv_writer.writerow([i, gt[i,4], gt[i,5]])


