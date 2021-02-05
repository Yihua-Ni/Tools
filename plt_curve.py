import matplotlib.pyplot as plt
import numpy as np

csv_file = '/home/minieye/桌面/2020-11-02/ts1.csv'

header_stamp = []
packet_stamp = []
with open(csv_file, 'r') as f:
    for line in f.readlines():
        header_point = line.strip().split(',')[0]
        packet_point = line.strip().split(',')[1]
        # print('head_point:', header_point)
        # print('packet_point:', packet_point)
        header_stamp.append(header_point)
        packet_stamp.append(packet_point)
# print('head_stamp:', header_stamp)
# print('packet_stamp:', packet_stamp)
print('时间戳数量:', len(header_stamp))


x = np.arange(0, len(header_stamp))

plt.plot(x, header_stamp, label='header_stamp')
plt.plot(x, packet_stamp, label='packet_stamp')

plt.xlabel('count')
plt.ylabel('timestamp')

plt.title('ts header_stamp and packet_stamp')
plt.legend(loc='upper right')       #图例

plt.show()






