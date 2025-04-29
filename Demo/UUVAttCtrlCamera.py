import time
import os
import sys
import math
import UE4CtrlAPI
import PX4MavCtrlV4 as PX4MavCtrl
import VisionCaptureApi
import cv2

mav = PX4MavCtrl.PX4MavCtrler(20100)
vis = VisionCaptureApi.VisionCaptureApi()
ue = UE4CtrlAPI.UE4CtrlAPI()
time.sleep(2)


vis.jsonLoad()
time.sleep(2)

isSuss = vis.sendReqToUE4()
if not isSuss: # 如果请求取图失败，则退出
    print('无图')
    sys.exit(0)
vis.startImgCap() # 开启取图循环，执行本语句之后，已经可以通过vis.Img[i]读取到图片了
vis.sendImuReqCopterSim() # 发送请求，从目标飞机CopterSim读取IMU数据,回传地址为127.0.0.1，默认频率为200Hz
time.sleep(2)

while True:
    #显示当前图像
    for i in range(len(vis.hasData)):
        if vis.hasData[i]:
            cv2.imshow('img1'+str(i), vis.Img[i])
            cv2.imwrite("DepthCam"+str(i)+".png", vis.Img[i])
            cv2.waitKey(1)