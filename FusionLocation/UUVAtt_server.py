# import required libraries
import time
import math
import numpy as np
import cv2
import sys

# import RflySim APIs
import PX4MavCtrlV4 as PX4MavCtrl
import VisionCaptureApi
import UE4CtrlAPI
import ReqCopterSim

# Get IP
windID = 0  # RflySim3D窗口的ID
UUVCopterID = 1  # 初始飞机的ID号
req = ReqCopterSim.ReqCopterSim()  # 获取局域网内所有CopterSim程序的电脑IP列表
TargetIP = req.getSimIpID(UUVCopterID)  # 获取CopterSim的1号程序所在电脑的IP，作为目标IP
print('TargetIP', TargetIP)

# 开启ROS发布模式
VisionCaptureApi.isEnableRosTrans = True

# Create MAVLink control API instance
mav = PX4MavCtrl.PX4MavCtrler(UUVCopterID, TargetIP)
vis = VisionCaptureApi.VisionCaptureApi(TargetIP)

# Init MAVLink data receiving loop
mav.InitMavLoop()
# VisionCaptureApi 中的配置函数re
vis.jsonLoad()
isSuss = vis.sendReqToUE4(windID, TargetIP)
vis.startImgCap() # 开启取图循环，执行本语句之后，已经可以通过vis.Img[i]读取到图片了
print('Start Image Reciver')
vis.sendImuReqCopterSim(UUVCopterID, TargetIP) # 发送请求，从目标飞机CopterSim读取IMU数据,回传地址为127.0.0.1，默认频率为200Hz
# 执行本语句之后，会自动开启数据监听，已经可以通过vis.imu读取到IMU数据了。

lastTime = time.time()
startTime = time.time()
# time interval of the timer
timeInterval = 1/60.0 #here is 0.0333s (30Hz)

flag1 = False  # 解锁标记
flag2 = False  # 任务开始时间记录标记
flag3 = False  # 文件保存记录

# 路径相关参数
i = 0  # 当前迭代次数
count = 5  # 路径重复次数
cowTime = 50  # 纵向的时长
rowTime = 10  # 横向的时长
T = 2 * (cowTime + rowTime)  # 路径周期
state = 1  # 当前的行进状态
missionStartTime = time.time()
a = 0
b = 0
c = 0

uavTimeStmp = np.array([[0]])
trueTimeStmp = np.array([[0]])
uavAngEular = np.array([[0, 0, 0]])  # Estimated Eular angles from PX4
trueAngEular = np.array([[0, 0, 0]]) # True simulated Eular angles from CopterSim's DLL model
uavAngRate = np.array([[0, 0, 0]])  # Estimated angular rate from PX4
trueAngRate = np.array([[0, 0, 0]]) # True simulated angular rate from CopterSim's DLL model
uavPosNED = np.array([[0, 0, 0]]) # Estimated Local Pos (related to takeoff position) from PX4 in NED frame
truePosNED = np.array([[0, 0, 0]]) # True simulated position (related to UE4 map center) from CopterSim's DLL model
uavVelNED = np.array([[0, 0, 0]]) # Estimated local velocity from PX4 in NED frame
trueVelNED = np.array([[0, 0, 0]]) # True simulated speed from CopterSim's DLL model  in NED frame
uavAngQuatern = np.array([[0, 0, 0, 0]]) # Estimated AngQuatern from PX4
trueAngQuatern = np.array([[0, 0, 0, 0]]) # True simulated AngQuatern from CopterSim's DLL model
uavMotorRPMS = np.array([[0, 0, 0, 0, 0, 0, 0, 0]]) # Estimated MotorPWMs from PX4
trueMotorRPMS = np.array([[0, 0, 0, 0, 0, 0, 0, 0]]) # True simulated MotorRPMS from CopterSim's DLL model
uavAccB = np.array([[0, 0, 0]]) # Estimated acc from PX4
trueAccB = np.array([[0, 0, 0]]) # True simulated acc from CopterSim's DLL model

uavGyro = np.array([[0, 0, 0]]) # Estimated Gyro from PX4
uavMag = np.array([[0, 0, 0]]) # Estimated Gyro from PX4
uavVibr = np.array([[0, 0, 0]]) # Estimated vibration xyz from PX4
uavPosGPS = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0]]) # Estimated GPS position from PX4 in NED frame,lat lon alt relative_alt vx vy vz hdg
uavPosGPSHome = np.array([[0, 0, 0]]) # Estimated GPS home (takeoff) position from PX4 in NED frame
uavGlobalPos = np.array([[0, 0, 0]]) # Estimated global position from PX4 that transferred to UE4 map

while True:
    lastTime = lastTime + timeInterval
    sleepTime = lastTime - time.time()
    if sleepTime > 0:
        time.sleep(sleepTime) # sleep until the desired clock
    else:
        lastTime = time.time()
    
    # The following code will be executed 30Hz (0.0333s)
    # Create the mission to vehicle 1
    #显示当前图像
    # for i in range(len(vis.hasData)):
    #     if vis.hasData[i]:
    #         cv2.imshow('img1'+str(i), vis.Img[i])
    #         cv2.waitKey(1)
    
    if time.time() - startTime > 5 and flag1 == 0:
        # The following code will be executed at 5s
        print("5s, Arm the drone!")
        print("开始进入Offboard模式")
        mav.initOffboard()
        time.sleep(0.5)
        mav.SendMavArm(True) # Arm the drone
        time.sleep(0.5)
        print("Armed the drone!")
        mav.SendAttPX4([0, 0, 0], 0)
        ArmedTime = time.time()
        flag1 = True
    
    # 如果没解锁，就等待解锁
    if flag1 == False:
        continue

    # 如果任务没开始，就开始任务，并记录任务开始时间
    if flag2 == False:
        missionStartTime = time.time()
        flag2 = True

    if time.time() - missionStartTime > 0:
        mav.SendAttPX4([0,-10,0],0.5)
    
    if time.time() - missionStartTime > 45:
        mav.SendAttPX4([0,-20,0],0.5)

    if time.time() - missionStartTime > 90:
        mav.SendAttPX4([0,0,0],0.5)
    
    if time.time() - missionStartTime > 135:
        mav.SendAttPX4([0,0,0],0)
        # 保存数据
        if flag3 == False and time.time() - missionStartTime > 145:
            flag3 = True
            
            path = "./data/"

            np.save(path + "uavTimeStmp", uavTimeStmp)
            np.save(path + "trueTimeStmp", trueTimeStmp)

            np.save(path + "uavAngEular", uavAngEular)
            np.save(path + "trueAngEular", trueAngEular)

            np.save(path + "uavAngRate", uavAngRate)
            np.save(path + "trueAngRate", trueAngRate)

            np.save(path + "uavPosNED", uavPosNED)
            np.save(path + "truePosNED", truePosNED)

            np.save(path + "uavVelNED", uavVelNED)
            np.save(path + "trueVelNED", trueVelNED)

            np.save(path + "uavAngQuatern", uavAngQuatern)
            np.save(path + "trueAngQuatern", trueAngQuatern)

            np.save(path + "uavMotorRPMS", uavMotorRPMS)
            np.save(path + "trueMotorRPMS", trueMotorRPMS)

            np.save(path + "uavAccB", uavAccB)
            np.save(path + "trueAccB", trueAccB)

            np.save(path + "uavGyro", uavGyro)
            np.save(path + "uavMag", uavMag)
            np.save(path + "uavVibr", uavVibr)
            np.save(path + "uavPosGPS", uavPosGPS)
            np.save(path + "uavPosGPSHome", uavPosGPSHome)
            np.save(path + "uavGlobalPos", uavGlobalPos)

            print("完成！")

    uavTimeStmp = np.vstack((uavTimeStmp, np.array([[mav.uavTimeStmp]])))
    trueTimeStmp = np.vstack((trueTimeStmp, np.array([[mav.trueTimeStmp]])))

    uavAngEular = np.vstack((uavAngEular, np.array([mav.uavAngEular])))
    trueAngEular = np.vstack((trueAngEular, np.array([mav.trueAngEular])))

    uavAngRate = np.vstack((uavAngRate, np.array([mav.uavAngRate])))
    trueAngRate = np.vstack((trueAngRate, np.array([mav.trueAngRate])))

    uavPosNED = np.vstack((uavPosNED, np.array([mav.uavPosNED])))
    truePosNED = np.vstack((truePosNED, np.array([mav.truePosNED])))

    uavVelNED = np.vstack((uavVelNED, np.array([mav.uavVelNED])))
    trueVelNED = np.vstack((trueVelNED, np.array([mav.trueVelNED])))

    uavAngQuatern = np.vstack((uavAngQuatern, np.array([mav.uavAngQuatern])))
    trueAngQuatern = np.vstack((trueAngQuatern, np.array([mav.trueAngQuatern])))

    uavMotorRPMS = np.vstack((uavMotorRPMS, np.array([mav.uavMotorRPMS])))
    trueMotorRPMS = np.vstack((trueMotorRPMS, np.array([mav.trueMotorRPMS])))

    uavAccB = np.vstack((uavAccB, np.array([mav.uavAccB])))
    trueAccB = np.vstack((trueAccB, np.array([mav.trueAccB])))

    uavGyro = np.vstack((uavGyro, np.array([mav.uavGyro])))
    uavMag = np.vstack((uavMag, np.array([mav.uavMag])))
    uavVibr = np.vstack((uavVibr, np.array([mav.uavVibr])))
    uavPosGPS = np.vstack((uavPosGPS, np.array([mav.uavPosGPS])))
    uavPosGPSHome = np.vstack((uavPosGPSHome, np.array([mav.uavPosGPSHome])))
    uavGlobalPos = np.vstack((uavGlobalPos, np.array([mav.uavGlobalPos])))

