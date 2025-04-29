# import required libraries
import time
import math
import numpy as np

# import RflySim APIs
import PX4MavCtrlV4 as PX4MavCtrl

# Create MAVLink control API instance
mav1 = PX4MavCtrl.PX4MavCtrler(20100)

# Init MAVLink data receiving loop
mav1.InitMavLoop()

lastTime = time.time()
startTime = time.time()

# time interval of the timer
timeInterval = 1/30.0 #here is 0.0333s (30Hz)

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
    if flag1 == False and time.time() - startTime > 5:
        # The following code will be executed at 5s
        print("5s, Arm the drone!")
        print("开始进入Offboard模式")
        mav1.initOffboard()
        time.sleep(0.5)
        mav1.SendMavArm(True) # Arm the drone
        time.sleep(0.5)
        print("Armed the drone!")
        mav1.SendAttPX4([0, 0, 90], 0)
        ArmedTime = time.time()
        flag1 = True

    # 如果没解锁，就等待解锁
    if flag1 == False:
        continue

    # 如果任务没开始，就开始任务，并记录任务开始时间
    if flag2 == False:
        missionStartTime = time.time()
        flag2 = True

    tick = 0.2
    thrust = 0.6
    if time.time() - missionStartTime < 5:  # 转弯控制
        b -= tick
        c += tick
        b = -25 if b < -25 else b
        c = 9 if c > 9 else c
        mav1.SendAttPX4([0, b, c], 0)
    
    elif time.time() - missionStartTime < 45:
        mav1.SendAttPX4([0, -25, 9], thrust)  # 40
    
    elif time.time() - missionStartTime < 50:  # 转弯控制
        b -= tick
        b = -55 if b < -55 else b
        mav1.SendAttPX4([0, b, 9], 0)
    
    elif time.time() - missionStartTime < 70:
        mav1.SendAttPX4([0, -55, 9], thrust)  # 20
    
    elif time.time() - missionStartTime < 80:  # 转弯控制
        b += tick
        b = 0 if b > 0 else b
        mav1.SendAttPX4([0, b, 9], 0)
    
    elif time.time() - missionStartTime < 120:
        mav1.SendAttPX4([0, 0, 9], thrust)  # 40
    
    elif time.time() - missionStartTime < 130:  # 转弯控制
        c += tick + 0.1
        c = 90 if c > 90 else c
        mav1.SendAttPX4([0, 0, c], 0)
    
    elif time.time() - missionStartTime < 190:
        mav1.SendAttPX4([0, 0, 90], thrust)  # 60
    
    elif time.time() - missionStartTime < 200:  # 转弯控制
        c += tick + 0.3
        c = 210 if c > 210 else c
        mav1.SendAttPX4([0, 0, c], 0)
    
    elif time.time() - missionStartTime < 240:
        mav1.SendAttPX4([0, 0, 210], thrust)  # 40
    
    elif time.time() - missionStartTime < 250:  # 转弯控制
        c += tick + 0.05
        c = 270 if c > 270 else c
        mav1.SendAttPX4([0, 0, c], 0)
    
    elif time.time() - missionStartTime < 300:
        mav1.SendAttPX4([0, 0, 270], thrust)  # 50
    
    elif time.time() - missionStartTime < 500:
        mav1.SendAttPX4([0, 0, 270], 0)
        flag3 = True

    uavTimeStmp = np.vstack((uavTimeStmp, np.array([[mav1.uavTimeStmp]])))
    trueTimeStmp = np.vstack((trueTimeStmp, np.array([[mav1.trueTimeStmp]])))

    uavAngEular = np.vstack((uavAngEular, np.array([mav1.uavAngEular])))
    trueAngEular = np.vstack((trueAngEular, np.array([mav1.trueAngEular])))

    uavAngRate = np.vstack((uavAngRate, np.array([mav1.uavAngRate])))
    trueAngRate = np.vstack((trueAngRate, np.array([mav1.trueAngRate])))

    uavPosNED = np.vstack((uavPosNED, np.array([mav1.uavPosNED])))
    truePosNED = np.vstack((truePosNED, np.array([mav1.truePosNED])))

    uavVelNED = np.vstack((uavVelNED, np.array([mav1.uavVelNED])))
    trueVelNED = np.vstack((trueVelNED, np.array([mav1.trueVelNED])))

    uavAngQuatern = np.vstack((uavAngQuatern, np.array([mav1.uavAngQuatern])))
    trueAngQuatern = np.vstack((trueAngQuatern, np.array([mav1.trueAngQuatern])))

    uavMotorRPMS = np.vstack((uavMotorRPMS, np.array([mav1.uavMotorRPMS])))
    trueMotorRPMS = np.vstack((trueMotorRPMS, np.array([mav1.trueMotorRPMS])))

    uavAccB = np.vstack((uavAccB, np.array([mav1.uavAccB])))
    trueAccB = np.vstack((trueAccB, np.array([mav1.trueAccB])))

    uavGyro = np.vstack((uavGyro, np.array([mav1.uavGyro])))
    uavMag = np.vstack((uavMag, np.array([mav1.uavMag])))
    uavVibr = np.vstack((uavVibr, np.array([mav1.uavVibr])))
    uavPosGPS = np.vstack((uavPosGPS, np.array([mav1.uavPosGPS])))
    uavPosGPSHome = np.vstack((uavPosGPSHome, np.array([mav1.uavPosGPSHome])))
    uavGlobalPos = np.vstack((uavGlobalPos, np.array([mav1.uavGlobalPos])))


    # 保存数据
    if flag3 == True:
        path = "./new_data/SITL_HITL_6/hitl/"

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
        break

