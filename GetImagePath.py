import os

ImageDir = './images/'
imagename = os.listdir(ImageDir)


with open('images.xml', 'a') as f:
    f.write('<?xml version="1.0"?>'+'\n')
    f.write('<opencv_storage>'+'\n')
    f.write('<images>' + '\n')
    for imagenames in imagename:
        f.write('.' + ImageDir + '/' + imagenames + '\n')
    f.write('</images>'+'\n')
    f.write('</opencv_storage>'+'\n')









