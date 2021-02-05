import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

import cv2
import pyproj
import os

import numpy as np
from matplotlib import pyplot as plt

from array import array


class VideoFrame:
	def __init__(self, dir_name, file_name):

		self.file_name = file_name
		self.dir_name = dir_name
		self.video_file = cv2.VideoCapture(dir_name + file_name)
		self.start_frame_id = int(file_name.split('.')[0].split('_')[1])
		self.end_frame_id = self.start_frame_id + self.video_file.get(cv2.CAP_PROP_FRAME_COUNT)
		print('start frame id:', self.start_frame_id, 'end frame id:', self.end_frame_id)

	def isValid(self, frame_id: int):
		if frame_id >= self.start_frame_id and frame_id < self.end_frame_id:
			return True
		else:
			return False

	def convert_frame_id(self, frame_id: int) -> int:
		return frame_id - self.start_frame_id


class LogConvert:
	def __init__(self,
	             dir_name,
	             Gsensor_name='Gsensor.1',
	             vehicle_can_name='CAN4',
	             speed_can_id='0xfd', 
				 is_convert_video = 1
	             ):
		self.is_convert_video = is_convert_video
		self.vehicle_can_name = vehicle_can_name
		self.Gsensor_name = Gsensor_name
		self.speed_can_id = speed_can_id
		self.dir_name = dir_name
		if self.dir_name[-1] != '/':
			self.dir_name += '/'
		self.converted_dir = self.dir_name + 'converted_data/'
		# if os.path.exists(self.converted_dir):
		# os.rmdir(self.converted_dir)
		os.system('rm -rf {}'.format(
			self.converted_dir
		))
		os.mkdir(self.converted_dir)

		self.log_file = self.dir_name + 'log.txt'
		self.time_stamp = list()
		self.info_list = list()

		self.video_dir = self.dir_name + 'video/'

		self.video_file_list = list()

		for file in os.listdir(self.video_dir):
			if 'avi' in file and self.is_convert_video > 0:
				vf = VideoFrame(self.video_dir, file)
				# os.system('cp {} {}'.format(
				# 	self.video_dir + file,
				# 	self.converted_dir)
				# )
				self.video_file_list.append(vf)

		self.p1 = pyproj.Proj(proj='latlong', datum='WGS84')
		# self.p2 = pyproj.Proj(proj="utm", zone=8, datum='WGS84')
		self.p2 = pyproj.Proj(init='epsg:4795')  # projected coordiate for china
		self.transformer = pyproj.transformer

		self.img_frame_id_list = list()
		self.imu_time_list = list()
		self.rtk_time_list = list()
		self.vel_time_list = list()

	def load_log_file(self):
		'''
		load log file, pre-process data and plot source data
		:return:
		'''
		ACCSCALE = 1.0 / 4096.0
		GYRSCALE = 2000.0 * np.pi / 180.0 / 32768.0
		G = 9.7925
		rtk_buffer = array('d')
		vel_buffer = array('d')
		imu_buffer = array('d')
		gps_buffer = array('d')
		# cv2.namedWindow("showImage")
		# log_file = self.log_file
		for line_str in open(self.log_file).readlines():
			# print(line_str)
			str_list = line_str.split(' ')
			time_stampe = float(str_list[0]) + float(str_list[1]) * 1e-6
			# print('source time:', str_list[0], str_list[1], ' converted time:', time_stampe)
			self.time_stamp.append(time_stampe)
			self.info_list.append(line_str)
			# if str_list[2] == 'CAN2' and str_list[3] == '0xfd':
			# 	print(self.str2vel(line_str))
			if ('NMEA' in line_str or 'gps' in line_str) and 'GNRMC' in line_str:
				# print(line_str)
				# lat = int(str_list[])
				# print(str_list)
				lat, lon = self.str2gps(str_list[3])
				gps_buffer.append(lon)
				gps_buffer.append(lat)

			# if 'rtk' in line_str and 'bestpos' in line_str:
			if 'rtk' in line_str and 'bestpos' in line_str and 'SOL_COMPUTED' in line_str and 'NARROW_INT' in line_str:				# print(self.str2rtk(line_str))
				rtk_val, rtk_sgm = self.str2rtk(line_str)
				# print("rtk val:", rtk_val , "rtk sgm:", rtk_sgm)
				if np.linalg.norm(rtk_sgm) < 10.56 and np.linalg.norm(rtk_val[:2]) > 1.:
					rtk_buffer.append(rtk_val[0])
					rtk_buffer.append(rtk_val[1])
					rtk_buffer.append(rtk_val[2])
					self.rtk_time_list.append(time_stampe)
			# print(self.transformer.transform(self.p1, self.p2, rtk_val[0], rtk_val[1]))
			if 'camera' in line_str:
				self.img_frame_id_list.append(int(str_list[3]))

			if 'Gsensor' in line_str:
				self.imu_time_list.append(time_stampe)
				for i in range(3):
					imu_buffer.append(float(str_list[i + 3]) * G * ACCSCALE)
				for i in range(3):
					imu_buffer.append(float(str_list[i + 6]) * GYRSCALE)

			if 'CAN2' in line_str and '0xfd' in line_str:
				self.vel_time_list.append(time_stampe)
				vel_buffer.append(self.str2velScoda(line_str))

		rtk_llh = np.frombuffer(rtk_buffer, dtype=np.float).reshape([-1, 3])
		gps_ll = np.frombuffer(gps_buffer, dtype=np.float).reshape([-1, 2])
		self.gps_raw = gps_ll * 1.

		print('img frame list size:', len(self.img_frame_id_list))

		# converte

		self.rtk_pos = np.zeros_like(rtk_llh)
		if self.rtk_pos.shape[0] > 0:
			self.rtk_pos[:, 0], self.rtk_pos[:, 1], self.rtk_pos[:, 2] = self.transformer.transform(
				self.p1,
				self.p2,
				rtk_llh[:, 0],
				rtk_llh[:, 1],
				rtk_llh[:, 2]
			)

		self.gps_pos = np.zeros_like(gps_ll)
		if self.gps_pos.shape[0] > 0:
			self.gps_pos[:, 0], self.gps_pos[:, 1] = self.transformer.transform(
				self.p1,
				self.p2,
				gps_ll[:, 0],
				gps_ll[:, 1]
			)

		plt.figure()
		plt.title('gps rtk trace')
		plt.plot(self.gps_pos[:, 0], self.gps_pos[:, 1], '-+', label='gps')
		plt.plot(self.rtk_pos[:, 0], self.rtk_pos[:, 1], '-+', label='rtk')
		plt.grid()
		plt.legend()

		# if self.gps_pos.shape[0] > 0:
		# 	self.gps_pos -= self.gps_pos[0, :]
		# if self.rtk_pos.shape[0] > 0:
		# 	self.rtk_pos -= self.rtk_pos[0, :]
		# self.rtk_pos /= 1.8

		plt.figure()
		plt.title('rtk pos')
		plt.plot(self.rtk_pos[:, 0], self.rtk_pos[:, 1])

		plt.figure()
		plt.title('camera frame id diff')
		id_array = np.asarray(
			self.img_frame_id_list
		)
		plt.plot(id_array[1:] - id_array[:-1], '-+')
		plt.grid()

		plt.figure()
		plt.title('imu time diff')
		imu_time_array = np.asarray(
			self.imu_time_list
		)
		plt.plot(imu_time_array[1:] - imu_time_array[:-1], '-+')

		imu_data = np.frombuffer(imu_buffer, dtype=np.float).reshape([-1, 6])
		plt.figure()
		plt.title('imu data')
		plt.subplot(211)
		for i in range(3):
			plt.plot(imu_data[:, i], '-+', label='imu' + str(i))
		plt.subplot(212)
		for i in range(3):
			plt.plot(imu_data[:, i + 3], '-+', label='gyr' + str(i))

		plt.figure()
		plt.title('rtk time diff')
		rtk_time_array = np.asarray(self.rtk_time_list)
		plt.plot(rtk_time_array[1:] - rtk_time_array[:-1], '-+')

		plt.figure()
		plt.title('vel time diff')
		vel_time_array = np.asarray(
			self.vel_time_list
		)
		plt.plot(vel_time_array[1:] - vel_time_array[:-1], '-+')

		vel_array = np.frombuffer(vel_buffer, dtype=np.float).reshape([-1])
		plt.figure()
		plt.title('vel compare')
		plt.plot(vel_time_array, vel_array / 3.6, '-+', label='vel velocity')

		rtk_vel = np.linalg.norm(self.rtk_pos[10:, :] - self.rtk_pos[:-10, :], axis=1) / (
				rtk_time_array[10:] - rtk_time_array[:-10])
		plt.plot(rtk_time_array[10:], rtk_vel[:], '-+', label='rtk vel')
		plt.legend()

	# print('rtk vel\n:', rtk_vel)
	# print(rtk_vel.shape)

	def generate_log_file(self, file_name):
		sorted_index = np.argsort(self.time_stamp)
		print('sorted index:', sorted_index)
		# plt.figure()
		# plt.plot(sorted_index, '-+')
		# plt.grid()

		log_file = open(self.converted_dir + file_name, 'w')

		video_name_list = list()
		for vf in self.video_file_list:
			path = vf.dir_name + vf.file_name
			if 'avi' in path:
				video_name_list.append(path)

		# for name in os.listdir(self.converted_dir):
		# 	if 'avi' in name:
		# 		video_name_list.append(name)

		video_name_list = sorted(video_name_list)
		print('sorted video name space:',
		      video_name_list)

		if len(video_name_list) < 1 and self.is_convert_video > 0:
			return

		video_cap = cv2.VideoCapture(
			# self.converted_dir + video_name_list[0]
			video_name_list[0]
		)
		ret, first_img = video_cap.read()
		height, width, layers = first_img.shape
		size = (width, height)
		in_index = 0
		video_cap.release()
		video_cap = cv2.VideoCapture(
			# self.converted_dir + video_name_list[0]
			video_name_list[0]
		)

		if	self.is_convert_video:
			out_video_name = 'output.mp4'
			out_video_file = cv2.VideoWriter(
				self.converted_dir + out_video_name,
				cv2.VideoWriter_fourcc(*'XVID'),
				20.0, size
			)

		out_index = 0

		rtk_index = 0

		gps_index = 0

		vel_coeff = 1.0

		for index in range(sorted_index.shape[0]):
			src_str = self.info_list[sorted_index[index]]
			str_list = src_str.split(' ')
			line_str = self.info_list[sorted_index[index]]

			if ('NMEA' in line_str or 'gps' in line_str) and 'GNRMC' in line_str:
				gps_info = '{} {} GPS {} {} {}'.format(
					str_list[0],
					str_list[1],
					self.gps_pos[gps_index, 0],
					self.gps_pos[gps_index, 1],
					0.0
				)
				gps_raw_info = '{} {} GPSRAW {} {} {}'.format(
					str_list[0],
					str_list[1],
					self.gps_raw[gps_index, 0],
					self.gps_raw[gps_index, 1],
					0.
				)
				gps_index += 1
				print(gps_info, file=log_file)
				print(gps_raw_info, file=log_file)

			if 'tcp' in src_str and ('INS_RTKFIXED' in src_str or 'INS_RTKFLOAT' in src_str):
				str_list_tmp = str_list[3].split(',')
				raw_rtk_info = '{} {} RTK {} {} {} {} {} {} {}'.format(str_list[0], str_list[1],
																			str_list_tmp[11],
																			str_list_tmp[12],
																			str_list_tmp[13],
																			str_list_tmp[18],
																			str_list_tmp[19],
																			str_list_tmp[20],
																			str_list_tmp[14])
				print(raw_rtk_info, file=log_file)
				# print(raw_rtk_info)
				# print(str_list[3])
				# print(src_str)

			# if 'rtk' in line_str and 'bestpos' in line_str and 'SOL_COMPUTED' in line_str and 'NARROW_INT' in line_str:				# src_rtk_val = self.str2rtk(line_str=src_str)
			# 	rtk_val, rtk_sgm = self.str2rtk(line_str)
			# 	if np.linalg.norm(rtk_sgm) < 10.56 and np.linalg.norm(rtk_val[:2]) > 1.:
			# 		rtk_info = '{} {} Position {} {} {}'.format(str_list[0], str_list[1],
			# 													self.rtk_pos[rtk_index, 0],
			# 													self.rtk_pos[rtk_index, 1],
			# 													self.rtk_pos[rtk_index, 2])
			# 		rtk_index += 1
			# 		print(rtk_info, file=log_file)
			# 		print('{} {} Orientation 0 0 0 1.0'.format(
			# 			str_list[0], str_list[1]
			# 		), file=log_file)

			# 		raw_rtk_info = '{} {} RTK {} {} {} {} {} {}'.format(str_list[0], str_list[1],
			# 																str_list[5],
			# 																str_list[6],
			# 																str_list[7],
			# 																str_list[10],
			# 																str_list[11],
			# 																str_list[12])
			# 		print(raw_rtk_info, file=log_file)


			if 'CAN' in src_str and len(str_list) == 5 and len(str_list[-1]) >= 16:
				tmp_str = str_list[4]
				tmp_list = [tmp_str[i:i+2] for i in range(0, len(tmp_str), 2)]
				new_str = str_list[0] + " " + \
							str_list[1] + " " + \
							str_list[2] + " " + \
							str_list[3] + " " + \
							tmp_list[0] + " " + \
							tmp_list[1] + " " + \
							tmp_list[2] + " " + \
							tmp_list[3] + " " + \
							tmp_list[4] + " " + \
							tmp_list[5] + " " + \
							tmp_list[6] + " " + \
							tmp_list[7]
				src_str = new_str
				str_list = new_str.split(' ')

			if self.vehicle_can_name in src_str:
				if self.speed_can_id == '0x260':
					if str_list[3] == '0x260':
						speed = ( (int(str_list[8], 16) & 0x1f) * 16 ** 2 + int(str_list[9], 16)) * 0.05625
						sign = (int(str_list[8], 16) & 0x20)
						if(sign == 0):
							speed = -speed
						# print(src_str + ' ' + str(speed))
						print('{} {} speed {}'.format(str_list[0],
						                              str_list[1],
						                              speed * vel_coeff),
						      file=log_file)

				if self.speed_can_id == '0x68f':
					if str_list[3] == '0x68f':
						speed = ( (int(str_list[11], 16)) * 16 ** 2 + int(str_list[10], 16)) * 0.01
						print('{} {} speed {}'.format(str_list[0],
						                              str_list[1],
						                              speed * vel_coeff),
						      file=log_file)

				if self.speed_can_id == '0xfd':
					if str_list[3] == '0xfd' or str_list[3] == '0x300':
						print('{} {} speed {}'.format(str_list[0],
						                              str_list[1],
						                              self.str2velScoda(src_str) * vel_coeff),
						      file=log_file)
					if str_list[3] == '0xad':
						vel_coeff = self.str2gearScoda(src_str)
				if self.speed_can_id == '0x300':
					if str_list[3] == '0x300':
						print('{} {} speed {}'.format(str_list[0],
						                              str_list[1],
						                              self.str2velRadar(src_str)),
						      file=log_file)
				if self.speed_can_id == '0x200':
					if str_list[3] == '0x200':
						print('{} {} speed {}'.format(
							str_list[0],
							str_list[1],
							self.str2velT5(src_str)
						), file=log_file)

				if self.speed_can_id == '0x440':
					if str_list[3] == '0x440':
						print('{} {} speed {}'.format(
							str_list[0],
							str_list[1],
							self.str2velQ3(src_str)
						), file=log_file)

				if self.speed_can_id == '0x18febf0b':
					if str_list[3] == '0x18febf0b':
						print('{} {} speed {}'.format(
							str_list[0],
							str_list[1],
							self.str2veltruck(src_str)),
							file=log_file
						)
						print('{} {} speed {}'.format(
							str_list[0],
							str_list[1],
							self.str2veltruck(src_str))
						)

			if self.Gsensor_name in src_str:
				print(src_str.split('\n')[0], file=log_file)

			if 'camera' in src_str:
				ret, img = video_cap.read()
				if not ret:
					video_cap.release()
					in_index += 1
					if in_index < len(video_name_list):
						video_cap = cv2.VideoCapture(
							# self.converted_dir + video_name_list[in_index])
							video_name_list[in_index])
					else:
						return
					ret, img = video_cap.read()
				if self.is_convert_video:
					out_video_file.write(img)
				print('{} {} cam_frame {} {}'.format(
					str_list[0],
					str_list[1],
					out_video_name,
					out_index
				),
					file=log_file)
				out_index += 1

	def str2rtk(self, line_str: str):
		'''
		string to rtk data.
		:param line_str:
		:return:
		'''
		fields = line_str.strip().split()[3:]
		r = dict()
		r['sol_stat'] = fields[0]
		r['pos_type'] = fields[1]
		r['lat'] = float(fields[2])
		r['lon'] = float(fields[3])
		r['hgt'] = float(fields[4])
		# r['undulation'] = float(fields[5])
		# r['datum'] = fields[6]
		r['lat_sgm'] = float(fields[7])
		r['lon_sgm'] = float(fields[8])
		r['hgt_sgm'] = float(fields[9])
		# r['diff_age'] = float(fields[10])
		# r['sol_age'] = float(fields[11])
		# r['#SVs'] = int(fields[12])
		# r['#solSVs'] = int(fields[13])
		# r['ext_sol_stat'] = int(fields[14], 16)
		rtk_pos = np.asarray((r['lon'], r['lat'], r['hgt']))
		rtk_sgm = np.asarray((r['lat_sgm'], r['lon_sgm'], r['hgt_sgm']))
		return rtk_pos, rtk_sgm

	def str2gps(self, line_str: str):
		fields = line_str.strip().split(',')
		src_lat = float(fields[3]) / 100.0
		src_lon = float(fields[5]) / 100.0
		# print(src_lat, src_lon)
		lat = int(src_lat) + (src_lat - int(src_lat)) / 60.0 * 100.0
		lon = int(src_lon) + (src_lon - int(src_lon)) / 60.0 * 100.0
		# print(lat, lon)
		return lat, lon

	def str2velScoda(self, line_str: str):
		fields = line_str.strip().split(' ')
		# print('in str2vel:', fields[2])
		i = 8
		speed = (int(fields[i + 1], 16) * 16 ** 2 + int(fields[i], 16)) * 0.01
		return speed

	def str2velRadar(self, line_str: str):
		fields = line_str.strip().split(' ')
		i = 4
		vel = (((int(fields[i], 16) & 0x1f)) * (16 ** 2) + int(fields[i + 1], 16)) * 0.02
		return vel * 3.6

	def str2gearScoda(self, line_str: str) -> float:
		fields = line_str.strip().split(' ')
		i = 8
		ob = (int(fields[9], 16) >> 2) & 0xf
		if ob == 6:
			return -1.0

		return 1.0

	def str2velT5(self, line_str: str):
		fields = line_str.strip().split(' ')
		i = 5
		speed = (int(fields[i], 16) * (16 ** 2) + int(fields[i + 1], 16) >> 3) * 0.0078125
		# speed = float(((int(fields[i + 1] + fields[i], 16) >> 3) & 0x1f) * (16 ** 2)
		#               + int(fields[i], 16)) * 0.05625 * 3.6  # Km/h
		# speed = float(((int(fields[i + 1] + fields[i], 16) >> 3) & 0x1ff )) * 0.05625 * 3.6  # Km/h
		# print( speed ,' field: ', fields)
		return speed

	def str2velQ3(self, line_str: str):
		fields = line_str.strip().split(' ')
		i = 6
		speed = (int(fields[i + 1], 16) * (16 ** 2) + int(fields[i], 16)) * 0.0078125 * 3.6
		# speed = (int(fields[i], 16) * (16 ** 2) + int(fields[i + 1], 16) >> 3) * 0.0078125
		# print (speed)
		return speed

	def str2veltruck(self, line_str: str):
		fields = line_str.strip().split(' ')
		i = 4
		speed = (int(fields[i + 1], 16) * (16 ** 2) + int(fields[i], 16)) * 0.00390625
		return speed


if __name__ == '__main__':
	# source_dir = '/media/steve/264A70504A701F2D/SourceData/vio_data/vio_20190722/20190722172013/'
	# source_dir = '/media/steve/264A70504A701F2D/SourceData/vio_data/vio_20190722/20190722172554/'
	# source_dir = '/media/steve/264A70504A701F2D/SourceData/vio_data/vio_20190722/20190722173511/'
	# source_dir = '/media/steve/264A70504A701F2D/SourceData/vio_data/vio_20190722/20190722174447/'

	# source_dir = '/media/steve/264A70504A701F2D/SourceData/vio_data/vio_20190720/20190720182429'
	# source_dir = '/media/steve/264A70504A701F2D/SourceData/vio_data/vio_20190720/20190720182630'
	# source_dir = '/media/steve/264A70504A701F2D/SourceData/vio_data/vio_20190720/20190720183203'
	# source_dir = '/media/steve/264A70504A701F2D/SourceData/vio_data/vio_20190720/20190720183328/'
	# source_dir = '/media/steve/264A70504A701F2D/SourceData/vio_data/vio_20190720/20190720183605/'
	# source_dir = '/media/steve/264A70504A701F2D/SourceData/vio_data/vio_20190720/20190720183750/'
	# source_dir = '/media/steve/264A70504A701F2D/SourceData/vio_data/vio_20190720/20190720184305/'
	# source_dir = '/media/steve/264A70504A701F2D/SourceData/vio_data/vio_20190720/20190720185240/'

	source_dir = '/home/steve/SourceData/vio_3180/20190819161354_正常道路'
	source_dir = '/home/steve/SourceData/vio_data/vio_20190828/20190828152551'

	# source_dir = '/home/minieye/SourceData/0924-li/20190924123321'
	source_dir = '/home/minieye/SourceData/vio_3180/20190820110856_高速行驶_10km_100kmh'
	source_dir = '/home/minieye/SourceData/vio_3180/20190820120002_高速行驶_10km_100kmh'
	source_dir = '/home/minieye/SourceData/vio_3180/20190820112216_弯道行驶'
	source_dir = '/home/minieye/SourceData/vio_3180/20190821103951_倒车入库'
	source_dir = '/home/minieye/SourceData/vio_3180/20190821103256_停车场'
	source_dir = '/home/minieye/SourceData/vio_3180/20190821104641_停车场'
	source_dir = '/home/minieye/SourceData/vio_data/20191105143437'
	source_dir = '/home/minieye/SourceData/vio_data/20191112181529'
	source_dir = '/home/steve/Data/20190722172013'
	# source_dir = '/home/minieye/SourceData/vio_data/vio_20191107/20191107172631'

	# lc = LogConvert(source_dir, vehicle_can_name='CAN3', Gsensor_name='Gsensor.1', speed_can_id='0x300')
	# lc = LogConvert(source_dir, vehicle_can_name='CAN2', Gsensor_name='Gsensor', speed_can_id='0xfd')
	# # lc = LogConvert(source_dir, vehicle_can_name='CAN4', Gsensor_name='Gsensor.1',speed_can_id='0x200')
	# lc.load_log_file()
	# lc.generate_log_file('log.txt')

	# plt.show()
	# 	source_dir = "/home/minieye/SourceData/vio_3180/"
	# 	source_dir = "/home/minieye/SourceData/vio_data/vio_20191114/"
	# 	source_dir = '/home/minieye/SourceData/vio_data/20191212/'
	# source_dir = '/home/steve/DataDisk/A1M-for-PCC/'
	# source_dir = '/home/steve/DataDisk/20200403_sz斯柯达_深南_map_A1M-2_x1/x1/'
	# source_dir = '/home/lili/D/20200415/pcc/'
	# source_dir = '/home/lili/D/aeb/tmp_data'
	# source_dir = '/home/lili/D/960/'
	# source_dir = '/home/lili/D/aeb/day/'
	# source_dir = '/home/lili/D/20200918_shangshan/'
	# source_dir = '/home/lili/D/0825/'
	# source_dir = '/home/lili/D/aeb/calib/'
	# source_dir = '/home/lili/D/jizhi/'
	# source_dir = '/home/lili/D/0924/20200924_suzhouskoda_q3_x1j_under_control/'
	# source_dir = '/home/lili/Downloads/20201027/'
	# source_dir = '/home/lili/D/107/'
	# source_dir = '/home/lili/D/slope_est_data/test/'
	# source_dir = '/home/lili/D/shenzhen_super_x1j_vio/'
	source_dir = '/home/minieye/桌面/camera_calibration/video/'
	# # #
	for sub_dir in os.listdir(source_dir):
		print('sub dir:', sub_dir)

		# lc = LogConvert(source_dir + sub_dir, vehicle_can_name='CAN4', Gsensor_name='Gsensor.1', speed_can_id='0x440')
		# lc = LogConvert(source_dir + sub_dir, vehicle_can_name='CAN4', Gsensor_name='Gsensor.1', speed_can_id='0xfd')
		# lc = LogConvert(source_dir + sub_dir, vehicle_can_name='CAN3', Gsensor_name='Gsensor.2', speed_can_id='0xfd')
		# lc = LogConvert(source_dir + sub_dir, vehicle_can_name='CAN6', Gsensor_name='Gsensor.2', speed_can_id='0xfd')
		# lc = LogConvert(source_dir + sub_dir, vehicle_can_name='CAN1', Gsensor_name='Gsensor.0', speed_can_id='0x68f')
		lc = LogConvert(source_dir + sub_dir, vehicle_can_name='CAN6', Gsensor_name='Gsensor.2', speed_can_id='0xfd')
		# lc = LogConvert(source_dir + sub_dir, vehicle_can_name='CAN4', Gsensor_name='Gsensor.1', speed_can_id='0xfd')
		# lc = LogConvert(source_dir + sub_dir, vehicle_can_name='CAN0', Gsensor_name='Gsensor.2', speed_can_id='0x300')
		# lc = LogConvert(source_dir + sub_dir, vehicle_can_name='CAN3', Gsensor_name='Gsensor.2', speed_can_id='0x260')
		# lc = LogConvert(source_dir + sub_dir, vehicle_can_name='CAN6', Gsensor_name='Gsensor.2', speed_can_id='0xfd', is_convert_video = 1)
		# lc = LogConvert(source_dir + sub_dir, vehicle_can_name='CAN3', Gsensor_name='Gsensor.1', speed_can_id='0xfd')
		# lc = LogConvert(source_dir + sub_dir, vehicle_can_name='CAN2', Gsensor_name='Gsensor.2',speed_can_id='0x18febf0b')

		lc.load_log_file()
		lc.generate_log_file('log.txt')
#

