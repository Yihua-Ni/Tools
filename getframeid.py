
txt_path = './raw_data_real_send_log.txt'
frame_path = './frame_file.txt'
count = 0

frame_file = open(frame_path, 'a')

with open(txt_path) as f:
    for lines in f.readlines():
        list = lines.strip().split(' ')
        if list[2] == 'cam_frame':
                print(list[2])
                print('count:', count)
                print('frame_id:', list[5].split(':')[1])
                frame_file.write(str(count) + " " + list[5].split(':')[1] + '\n')
                count = count + 1


