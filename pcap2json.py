from scapy.all import *
import json
 
#limit = 26000000
limit = 26000
count = 0
timestamps = []
filename = "/home/minieye/桌面/2020-11-02-11-43-26_Velodyne-HDL-128-Data.pcap"
dst = "/home/minieye/桌面/2020-11-02-11-43-26_Velodyne-HDL-128-Data.pcap.json"
def func(pkt):
	global limit, count, timestamps
	print('pkt.time:', str(pkt.time))
	#print((pkt.time))
	#print((int((pkt.time)/3600))*3600)
	m = (pkt.time) - (int((pkt.time)/3600))*3600
	print('m:', m)
	#timestamps.append(str(pkt.time))
	timestamps.append(str(m))
	count = count + 1
	if limit > 0 and count >= limit:
		return True
	else:
		return False

if "__main__" == __name__:
	sniff(offline=filename, stop_filter=func, store=False)
	with open(dst, "w") as f:
		json.dump(timestamps, f)
