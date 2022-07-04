# import pyqtgraph as pg
import numpy as np
import array
import serial  # 导入串口包
from datetime import datetime # 导入时间包
import threading
import math

# command of readding linux serials: "dmesg|grep tty*"
# ser = serial.Serial("/dev/cu.usbserial-14140", 115200, timeout=5)
ser = serial.Serial("/dev/cu.usbmodem141401", 2000000, timeout=5)
# ser = serial.Serial("/dev/cu.usbmodem141101",115200,timeout = 5)
ser.flushInput()  # 清空缓冲区



node = 128  # max length
filterLength = 4

# save the temporary data
qw = np.zeros((node, filterLength))
qx = np.zeros((node, filterLength))
qy = np.zeros((node, filterLength))
qz = np.zeros((node, filterLength))

# save the filtered data
filter_qw = np.zeros(node)
filter_qx = np.zeros(node)
filter_qy = np.zeros(node)
filter_qz = np.zeros(node)

i = 0
threshold = 0.4
error_num = np.zeros((node, 4))
filter_flag = True

# 中位数均值滤波函数
def filter(quaternions):
    global i
    sumqw, sumqx, sumqy, sumqz = [], [], [], []

    for num, w, x, y, z in quaternions:
        num = int(num)
        if i < filterLength:
            qw[num][i], qx[num][i], qy[num][i], qz[num][i] = w, x, y, z
        else:
            # remove the abnormal value
            if w > 1.1 or w < -1.1:  # made by noise
                w = qw[num][filterLength-1]
            elif math.fabs(w - qw[num][filterLength-1]) > threshold and error_num[num][0] <= 4:  # made by noise or fast moving
                w = qw[num][filterLength-1]
                error_num[num][0] += 1  # detect error made by fast moving
            else:
                error_num[num][0] = 0

            if x > 1.1 or x < -1.1:
                x = qx[num][filterLength-1]
            elif math.fabs(x - qx[num][filterLength-1]) > threshold and error_num[num][1] <= 4:
                x = qx[num][filterLength-1]
                error_num[num][1] += 1
            else:
                error_num[num][1] = 0

            if y > 1.1 or y < -1.1:
                y = qy[num][filterLength-1]
            elif math.fabs(y - qy[num][filterLength-1]) > threshold and error_num[num][2] <= 4:
                y = qy[num][filterLength-1]
                error_num[num][2] += 1
            else:
                error_num[num][2] = 0

            if z > 1.1 or z < -1.1:
                z = qz[num][filterLength-1]
            elif math.fabs(z - qz[num][filterLength-1]) > threshold and error_num[num][3] <= 4:
                z = qz[num][filterLength-1]
                error_num[num][3] += 1
            else:
                error_num[num][3] = 0

            # update the buffer array / move left 1 unit
            qw[num][:-1], qx[num][:-1], qy[num][:-1], qz[num][:-1] = qw[num][1:], qx[num][1:], qy[num][1:], qz[num][1:]
            qw[num][filterLength-1], qx[num][filterLength-1], qy[num][filterLength-1], qz[num][filterLength-1] = w, x, y, z

            # get the index of the max/min value in arrays
            maxqw, maxqx, maxqy, maxqz = np.argmax(qw[num]), np.argmax(qx[num]), np.argmax(qy[num]), np.argmax(qz[num])
            minqw, minqx, minqy, minqz = np.argmin(qw[num]), np.argmin(qx[num]), np.argmin(qy[num]), np.argmin(qz[num])

            # sum except the max/min value
            for k in range(filterLength):
                if k != maxqw or k != minqw:
                    sumqw.append(qw[num][k])
                if k != maxqx or k != minqx:
                    sumqx.append(qx[num][k])
                if k != maxqy or k != minqy:
                    sumqy.append(qy[num][k])
                if k != maxqz or k != minqz:
                    sumqz.append(qz[num][k])

            # mean
            w, x, y, z = sum(sumqw)/len(sumqw), sum(sumqx)/len(sumqx), sum(sumqy)/len(sumqy), sum(sumqz)/len(sumqz)

            # update the buffer array / remove the max/min value
            qw[num][maxqw], qw[num][minqw] = w, w
            qx[num][maxqx], qx[num][minqx] = x, x
            qy[num][maxqy], qy[num][minqy] = y, y
            qz[num][maxqz], qz[num][minqz] = z, z

            filter_qw[num], filter_qx[num], filter_qy[num], filter_qz[num] = w, x, y, z
            sumqw, sumqx, sumqy, sumqz = [], [], [], []

    if i < filterLength:
        i += 1


def SerialRead():
    j = 0
    filter_flag = True
    try:
        while True:
            # start_time = datetime.now()
            while True:
                if ser.inWaiting():
                    try:
                        recv = ser.readline().decode("gbk", 'ignore')
                        quaternions = recv.split(';')[:-1]
                        for i in range(len(quaternions)):
                            quaternion = quaternions[i].split(' ')
                            quaternions[i] = quaternion[:5]
                        quaternions = np.array(quaternions)
                        quaternions = quaternions.astype(np.float16)

                        filter(quaternions)

                        for num, w, x, y, z in quaternions:
                            num = int(num)

                            print("{} {:+4.2f} {:+4.2f} {:+4.2f} {:+4.2f}".format(num, filter_qw[num], filter_qx[num], filter_qy[num], filter_qz[num]))
                            with open("/Users/zzh/Desktop/3dPerception/Unity/3D_Perception/Assets/Scripts/python/txtfiles/cube{}.txt".format(num), 'w') as f:
                                f.write("{:+4.2f} {:+4.2f} {:+4.2f} {:+4.2f}".format(filter_qw[num], filter_qx[num], filter_qy[num], filter_qz[num]))

                    except Exception as e:
                        print(e)
                        continue
                    break

            # end_time = datetime.now()
            # print("----------------------------------- FPS: {:.1f}".format(1e6/(end_time-start_time).microseconds))

    except Exception as e:
        ser.close()
        print(e)


SerialRead()
