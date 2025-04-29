import time
import math
import random
import quaternion
import numpy as np
import UE4CtrlAPI as UE4CtrlAPI
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from threading import Thread
from nav_msgs.msg import Path


# class RopeLocator(object):
#     def __init__(self, original_point=None):
#         self.original_point = original_point
#     def calculate_pos_by_rope(self, rope_info, bias_uuv):
#         x = rope_info[0] * math.cos(rope_info[1])
#         y = rope_info[0] * math.cos(rope_info[2])
#         z = rope_info[0] * math.cos(rope_info[3])
#         x -= bias_uuv[0]
#         y -= bias_uuv[1]
#         z -= bias_uuv[2]
#         return np.array([x, y, z])
#     def calculate_original_point(self, rope_info, bias_uuv):
#         self.original_point = self.calculate_pos_by_rope(rope_info, bias_uuv)
#     def get_position(self, rope_info, bias_uuv):
#         position = self.calculate_pos_by_rope(rope_info, bias_uuv)
#         return position - self.original_point


class RopeLocator(object):
    def __init__(self, ip="127.0.0.1", bias_uuv=None, bias_boat=None):
        self.ip = ip
        self.uuvTargetCopterID = 1
        self.boatTargetCopterID = 2
        self.original_point = None  # 初始点
        self.bias_uuv = bias_uuv  # 无人机本地坐标系下的绳索端点偏置
        self.bias_boat = bias_boat  # 无人船本地坐标系下的绳索端点偏置
        if self.bias_boat == None or self.bias_uuv == None:
            self.load_bias()
        self.is_init = False  # 初始化标记
        self.init_ue()

    
    def init_ue(self):
        self.ue = UE4CtrlAPI.UE4CtrlAPI(self.ip)
        # 发送消息给RflySim3D，让其将当前收到的飞机数据转发出来，回传到组播地址224.0.0.10的20006端口
        self.ue.sendUE4Cmd('RflyReqVehicleData 1')
        time.sleep(0.5)
        # 注：只有飞机位置发生改变时，才会将位置数据传出，因此本语句要放在最前面，确保后续创建的物体（Python一次性创建）都能被传出
        # Python开始飞机数据的监听，数据存储在inReqUpdateVect列表（是否更新标志），和inReqVect列表（碰撞数据）中
        self.ue.initUE4MsgRec()
        time.sleep(0.5)
        # 创建船
        self.ue.sendUE4Pos(2, 603, 0, [10,2,0.472], [0,0,0])
        time.sleep(0.5)
        # 生成绳子
        self.ue.sendUE4ExtAct(2, [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
        time.sleep(0.5)
        # 注意：监听语句应该放到sendUE4系列语句之前，不然无法捕获创建的障碍物
        while len(self.ue.inReqVect) != 2:
            time.sleep(1)
            print("Wait for data...")
        print('所有载具数量: ', len(self.ue.inReqVect))

    
    def calculate_pos_by_rope(self, rope_info, bias_uuv):
        """
        函数功能：
        根据绳索信息和无人机的偏置计算无人机的定位
        """
        # x = rope_info[0] * math.cos(rope_info[1])
        # y = rope_info[0] * math.cos(rope_info[2])
        # z = rope_info[0] * math.cos(rope_info[3])
        x = rope_info[0] * math.cos(rope_info[1]) * math.cos(rope_info[2])
        y = rope_info[0] * math.cos(rope_info[1]) * math.sin(rope_info[2])
        z = rope_info[0] * math.sin(rope_info[1])
        x -= bias_uuv[0]
        y -= bias_uuv[1]
        z -= bias_uuv[2]
        return np.array([x, y, z])
    
    def set_original_point(self, rope_info, bias_uuv):
        """
        函数功能：
        设置定位的初始点，
        因为所有定位都是基于初始位置的，所以需要先将初始点计算出来，
        定位的时候再拿定位的结果减去初始点，才得到相对于初始点的定位信息。
        """
        self.original_point = self.calculate_pos_by_rope(rope_info, bias_uuv)
    
    def get_position(self, rope_info, bias_uuv):
        """
        函数功能：
        计算定位
        """
        if not self.is_init:  # 如果没有初始化定位点，要进行初始化
            self.set_original_point(rope_info, bias_uuv)
            self.is_init = True
        position = self.calculate_pos_by_rope(rope_info, bias_uuv)
        return position - self.original_point
    
    def euler_to_quat(self, roll_pitch_yaw, pitch=None, yaw=None):
        """
        函数功能：
        将欧拉角转换为四元数，既可以传入一个由欧拉角组成的列表，也可以分别传入三个欧拉角

        参数：可由参数名分辨
        """
        roll = roll_pitch_yaw
        if(yaw == None):
            roll = roll_pitch_yaw[0]
            pitch = roll_pitch_yaw[1]
            yaw = roll_pitch_yaw[2]
        x = math.sin(pitch/2) * math.sin(yaw/2) * math.cos(roll/2) + math.cos(pitch/2) * math.cos(yaw/2) * math.sin(roll/2)
        y = math.sin(pitch/2) * math.cos(yaw/2) * math.cos(roll/2) + math.cos(pitch/2) * math.sin(yaw/2) * math.sin(roll/2)
        z = math.cos(pitch/2) * math.sin(yaw/2) * math.cos(roll/2) - math.sin(pitch/2) * math.cos(yaw/2) * math.sin(roll/2)
        w = math.cos(pitch/2) * math.cos(yaw/2) * math.cos(roll/2) - math.sin(pitch/2) * math.sin(yaw/2) * math.sin(roll/2)
        return [w, x, y, z]
    
    def calculate_endpoint_pos(self, data, bias):
        """
        函数功能：
        根据无人机在世界坐标系下的位置和姿态，
        以及绳索端点在无人机本地坐标系下的位置，
        来计算绳索端点在世界坐标系下的位置。

        参数：
        data为无人机在世界坐标系下的位置和姿态等数据
        bias为绳索端点在无人机本地坐标系下的偏置
        
        返回值：
        绳索端点在世界坐标系下的位置
        """
        q = quaternion.from_float_array(self.euler_to_quat(data.AngEuler[0], data.AngEuler[1], data.AngEuler[2]))
        p = quaternion.from_float_array([0.0, bias[0], bias[1], bias[2]])
        p = q * p * q.conjugate()
        rope_endpoint_pos = (data.PosE[0] + p.x, data.PosE[1] + p.y, data.PosE[2] + p.z)
        return rope_endpoint_pos


    def calculate_bias(self, data, bias):
        """
        函数功能：
        根据无人潜航器的姿态和偏置，计算新的偏置

        参数：
        data为无人机在世界坐标系下的位置和姿态等数据
        bias为绳索端点在无人机本地坐标系下的偏置
        """
        q = quaternion.from_float_array(self.euler_to_quat(data.AngEuler[0], data.AngEuler[1], data.AngEuler[2]))
        p = quaternion.from_float_array([0.0, bias[0], bias[1], bias[2]])
        p = q * p * q.conjugate()
        new_bias = [p.x, p.y, p.z]
        return new_bias


    def calculate_rope_info(self, x_uuv, x_boat):
        """
        函数功能：
        计算绳子长度，计算绳子分别相对于xyz轴的偏角

        参数：
        x_uuv为系在潜航器尾巴处绳子的端点坐标
        x_boat为系在船旁处绳子的端点坐标

        返回值：
        (绳长，绳子相对于x轴的角度，绳子相对于y轴的角度，绳子相对于z轴的角度)

        注意：
        参数顺序一定要搞对，否则绳子向量的方向会反
        """
        # 检查数据
        if len(x_uuv) != 3 or len(x_uuv) != len(x_boat):
            return False
        # 转为ndarray
        x_uuv = np.array(x_uuv)
        x_boat = np.array(x_boat)
        # 计算绳子长度
        x_rope = x_uuv - x_boat  # 绳子向量
        rope_length = np.sqrt(x_rope.dot(x_rope))  # 计算模长
        # 三个轴的单位向量
        # x = np.array([1, 0, 0])
        # y = np.array([0, 1, 0])
        # z = np.array([0, 0, 1])
        rope_xy = np.array([x_rope[0], x_rope[1], 0])  # 绳索向量去掉z轴
        rope_xy_length = np.sqrt(rope_xy.dot(rope_xy))  # 计算模长
        rope_x = np.array([1, 0, 0])  # 绳索向量去掉yz轴
        rope_x_length = np.sqrt(rope_x.dot(rope_x))  # 计算模长
        # 根据向量点积公式计算分别相对xyz轴的角度
        # rope_angle = (math.acos(x.dot(x_rope)/rope_length),
        #             math.acos(y.dot(x_rope)/rope_length),
        #             math.acos(z.dot(x_rope)/rope_length))
        theta_r = math.acos(rope_xy.dot(x_rope) / (rope_length * rope_xy_length))
        theta_r = -theta_r if x_rope[2] < 0 else theta_r
        fai_r = math.acos(rope_x.dot(rope_xy) / (rope_xy_length))
        fai_r = -fai_r if x_rope[1] < 0 else fai_r

        # return [rope_length, rope_angle[0], rope_angle[1], rope_angle[2]]
        return [rope_length, theta_r, fai_r]


    def load_bias(self):
        # 绳子端点分别相对于潜航器中心和船中心的偏移，单位cm，坐标系北东地
        # x, y, z 20.495+40.5 XML文件这儿还将中心位置修改了下，所以相对位置还要加上40.5
        self.bias_uuv = np.array([-54.693, -3.480, -60.995]) / 100.0
        # x, y, z 46.486126+40.5 XML文件这儿还将中心位置修改了下，所以相对位置还要加上40.5
        self.bias_boat = np.array([-32.187225, 231.702225, -86.986126]) / 100.0
    
    
    def add_Gaussian_Noise_Pos(self, pose, uuv, k=1):
        vel = uuv.VelE
        sigma = k * math.sqrt(vel[0]**2 + vel[1]**2 + vel[2]**2)
        pose.pose.position.x += random.gauss(0, sigma)
        pose.pose.position.y += random.gauss(0, sigma)
        pose.pose.position.z += random.gauss(0, sigma)
        return pose
    

    def add_Gaussian_Noise_Rope(self, rope_info, uuv, k1=1, k2=1):
        vel = uuv.VelE
        sigma1 = k1 * math.sqrt(vel[0]**2 + vel[1]**2 + vel[2]**2)
        sigma2 = k2 * math.pi/180
        rope_info[0] += random.gauss(0, sigma1)
        rope_info[1] += random.gauss(0, sigma2)
        rope_info[2] += random.gauss(0, sigma2)
        return rope_info

    
    def start(self):
        # 绳子定位
        while True:
            # 通过getUE4Data来获取uuv
            data_uuv = self.ue.getUE4Data(self.uuvTargetCopterID)
            data_boat = self.ue.getUE4Data(self.boatTargetCopterID)
            if isinstance(data_uuv, UE4CtrlAPI.reqVeCrashData) and isinstance(data_boat, UE4CtrlAPI.reqVeCrashData):
                # 计算uuv处绳子端点的坐标
                rope_uuvpoint_pos = self.calculate_endpoint_pos(data_uuv, self.bias_uuv)
                # 计算boat处绳子端点的坐标
                rope_boatpoint_pos = self.calculate_endpoint_pos(data_boat, self.bias_boat)
                # 计算绳子信息
                rope_info = self.calculate_rope_info(rope_uuvpoint_pos, rope_boatpoint_pos)
                # 计算无人潜航器算上姿态的偏置
                bias_uuv = self.calculate_bias(data_uuv, self.bias_uuv)
                # print(rope_info[0], math.degrees(rope_info[1]), math.degrees(rope_info[2]), math.degrees(rope_info[3]))
                # 计算定位
                position = self.get_position(rope_info, bias_uuv)
                print(position[0], position[1], position[2])
            else:
                print('没有无人潜航器或船只的数据')
            time.sleep(0.0001)
    

    def start_with_ros(self):
        import rospy
        from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped
        
        rospy.init_node("rope")
        path_pub = rospy.Publisher("/rflysim/rope/path", Path, queue_size=10)
        pose_pub = rospy.Publisher("/rflysim/rope/pose", PoseStamped, queue_size=10)
        current_time = rospy.Time.now()
        path = Path()
        path.header.stamp = current_time
        path.header.frame_id = "world"

        rate = rospy.Rate(60)

        # 绳子定位
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()

            # 通过getUE4Data来获取uuv
            data_uuv = self.ue.getUE4Data(self.uuvTargetCopterID)
            data_boat = self.ue.getUE4Data(self.boatTargetCopterID)
            if isinstance(data_uuv, UE4CtrlAPI.reqVeCrashData) and isinstance(data_boat, UE4CtrlAPI.reqVeCrashData):
                # 计算uuv处绳子端点的坐标
                rope_uuvpoint_pos = self.calculate_endpoint_pos(data_uuv, self.bias_uuv)
                # 计算boat处绳子端点的坐标
                rope_boatpoint_pos = self.calculate_endpoint_pos(data_boat, self.bias_boat)
                # 计算绳子信息
                rope_info = self.calculate_rope_info(rope_uuvpoint_pos, rope_boatpoint_pos)
                rope_info = self.add_Gaussian_Noise_Rope(rope_info, data_uuv, k1=0.2, k2=0.2)
                # 计算无人潜航器算上姿态的偏置
                bias_uuv = self.calculate_bias(data_uuv, self.bias_uuv)
                # print(rope_info[0], math.degrees(rope_info[1]), math.degrees(rope_info[2]), math.degrees(rope_info[3]))
                # 计算定位
                position = self.get_position(rope_info, bias_uuv)
                # print(position[0], position[1], position[2])
                # 定义数据结构
                pose = PoseStamped()
                pose.header.stamp = current_time
                pose.header.frame_id = "rope"
                pose.pose.position.x = position[0]
                pose.pose.position.y = -position[1]
                pose.pose.position.z = -position[2]
                quat = self.euler_to_quat(data_uuv.AngEuler[0], data_uuv.AngEuler[1], data_uuv.AngEuler[2])
                pose.pose.orientation.w = quat[0]
                pose.pose.orientation.x = quat[1]
                pose.pose.orientation.y = quat[2]
                pose.pose.orientation.z = quat[3]
                # pose = self.add_Gaussian_Noise_Pos(pose, data_uuv, k=0.1)
                path.poses.append(pose)
                # 发布数据
                path_pub.publish(path)
                pose_pub.publish(pose)
            else:
                print('没有无人潜航器或船只的数据')
            
            rate.sleep()


if __name__ == '__main__':
    locator = RopeLocator("192.168.31.100")
    locator.start_with_ros()
    
