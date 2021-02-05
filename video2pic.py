import cv2
import os

# #视频文件夹地址
# vedio_dir = './video'



# pathDir = os.listdir(vedio_dir)     #读取video_dir所有视频文件名
# print(pathDir)
# piccount = 1
# for i in range(0, len(pathDir)):
#     video = pathDir[i]
#     print(video)
#     videopath = vedio_dir + '/' + video
#     # print(videopath)
#     vc = cv2.VideoCapture(videopath)
#     c = 1
#     if vc.isOpened():
#         rval, frame = vc.read()   #rval:是否读到图片 ture or false    frame:截取到一帧的图片
#     else:
#         rval = False
#     timeF = 100   #隔40帧取一帧
#     while rval:
#         rval, frame = vc.read()
#         if( c%timeF == 0 ):
#             picpath = './images/' + video.split('.')[0] + '_' + str(piccount) + '.jpg'
#             # print(picpath)
#             cv2.imwrite(picpath, frame)     #保存图片到当前目录
#             piccount += 1
#         c += 1
#         cv2.waitKey(1)
#     vc.release()


#视频文件位置
videopath = './output.mp4'

times = 0
frameFrequency = 25

vc = cv2.VideoCapture(videopath)

while True:
    times += 1
    rval, frame = vc.read()
    if not rval:
        print(str(times)+':''not rval, not image')
        break
    if times % frameFrequency == 0:
        picpath = './images/' + str(times) + '.jpg'
        print(picpath)
        cv2.imwrite(picpath, frame)     #保存图片到当前目录

print('图片提取结束')
vc.release()




