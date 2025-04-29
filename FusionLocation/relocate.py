import os
import signal
import subprocess
import time
import rospy
import numpy as np
from multiprocessing import Process, Value, Queue
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from tf.transformations import quaternion_multiply, quaternion_conjugate, quaternion_matrix
# from queue import Queue
# 两个队列是同一个东西，multiprocessing创建的就是queue的Queue

"""
https://blog.csdn.net/weixin_41362649/article/details/103066936
https://blog.csdn.net/geo_mai/article/details/83793279
"""
class Relocate:
    def __init__(self, threshold=0.9, skip=10, start_vins=False):
        self.skip = skip
        self.threshold = threshold
        self.need_restart = Value('i', 0)
        self.restarted = Value('i', 0)
        self.count = 0
        # 定义重启消息发布者
        self.restart_pub = rospy.Publisher("/vins_restart", Bool, queue_size=10)
        # 定义发布路径的话题
        self.path_pub = rospy.Publisher("/rflysim/relocate/path", Path, queue_size=10)
        self.pose_pub = rospy.Publisher("/rflysim/relocate/pose", PoseStamped, queue_size=10)
        # 订阅vins数据
        self.vins_sub = rospy.Subscriber("/vins_estimator/odometry", Odometry, callback=self.vins_callback, callback_args=[])
        # 订阅融合数据
        self.fusion_sub = rospy.Subscriber("/rflysim/fusion/pose", PoseStamped, callback=self.fusion_callback, callback_args=[])
        # 定义vins启动的命令，并执行，取消打印输出到scream
        if start_vins:
            cmd1 = "source ~/ros1_ws/devel/setup.bash"
            cmd2 = "rosrun vins vins_node ~/ros1_ws/src/VINS-Fusion/config/euroc_uuv/rflysim_stereo_imu_config.yaml"
            subprocess.Popen([cmd1 + "\n" + cmd2], shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        rospy.init_node("relocate")

        
    def start(self):
        # 定义一些数据结构。
        # 注意，这些numpy数组事实上，在其他进程中修改这些值，主进程中的值并不会被修改。
        self.poses = np.array([[0,0,0]])
        self.difference1 = np.array([[0,0,0]])
        self.difference2 = np.array([[0,0,0]])
        self.difference3 = np.array([[0,0,0]])
        # 这个队列是可以多进程中共享的，如果在其他进程中往队列中写入数据，则主进程中也会有该数据。
        self.queue = Queue()
        self.origin = Queue()

        # 定义要发布的路径
        self.path = Path()
        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = "world"

        # 循环读取队列的数据，如果有数据就发布，如果没有数据就标记为error
        print("start!")
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose_tmp = None

            if self.count > 3:
                pose.header.frame_id = "error"
                self.path.poses.append(pose)
                self.path_pub.publish(self.path)
                self.pose_pub.publish(pose)
                rate.sleep()
                continue

            if not self.queue.empty():
                pose.header.frame_id = "relocate"
                pose_tmp = self.queue.get_nowait()
                # 如果是vins数据，并且已经重启过，并且记录过原点，则需要修改vins数据的原点
                if self.restarted.value == 1:
                    if not self.origin.empty():
                        o = self.origin.get_nowait()
                        self.origin.put(o)
                        # pose_tmp = self.convert_to_world_coordinate(pose_tmp, o)
                        pose_tmp.pose.position.x += o.pose.position.x
                        pose_tmp.pose.position.y += o.pose.position.y
                        pose_tmp.pose.position.z += o.pose.position.z
                    else:
                        # 刚重启之后，本路径话题不会发布消息，所以fusion中也不会有数据
                        # 那么回调函数永远不会被调用，所以需要发送点error数据，让它发布一些数据用来更新原点
                        pose.header.frame_id = "error"
            
            if pose_tmp != None:
                pose.pose.position.x = pose_tmp.pose.position.x
                pose.pose.position.y = pose_tmp.pose.position.y
                pose.pose.position.z = pose_tmp.pose.position.z
                pose.pose.orientation.w = pose_tmp.pose.orientation.w
                pose.pose.orientation.x = pose_tmp.pose.orientation.x
                pose.pose.orientation.y = pose_tmp.pose.orientation.y
                pose.pose.orientation.z = pose_tmp.pose.orientation.z

                self.path.poses.append(pose)
                self.path_pub.publish(self.path)
                self.pose_pub.publish(pose)

                if pose.header.frame_id != "error":
                    # 计算差分
                    pose = np.array([[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]])
                    self.poses = np.vstack((self.poses, pose))
                    if len(self.poses) > 2:
                        diff = self.poses[len(self.poses) - 1] - self.poses[len(self.poses) - 2]
                        diff = np.array([diff])
                        self.difference1 = np.vstack((self.difference1, diff))
                    if len(self.difference1) > 2:
                        diff = self.difference1[len(self.difference1) - 1] - self.difference1[len(self.difference1) - 2]
                        diff = np.array([diff])
                        self.difference2 = np.vstack((self.difference2, diff))
                    if len(self.difference2) > 2:
                        diff = self.difference2[len(self.difference2) - 1] - self.difference2[len(self.difference2) - 2]
                        diff = np.array([diff])
                        self.difference3 = np.vstack((self.difference3, diff))
                    
                    rows = len(self.difference3)
                    if rows > self.skip:
                        # print("v =" + str(self.difference1[rows-1, :]))
                        # print("a =" + str(self.difference2[rows-1, :]))
                        # print("a'=" + str(self.difference3[rows-1, :]))
                        # var = np.var(self.difference3[rows-window:, :], axis=0, ddof=1)
                        # score = self.sigmoid(var)
                        score = np.abs(self.difference3[rows-1, :])
                        score = self.sigmoid(score)
                        print(score)
                        if score[0] > self.threshold or score[1] > self.threshold or score[2] > self.threshold:
                            self.restarted.value = 1
                            self.data_clear()
                            self.restart()
            
            rate.sleep()

    def restart(self):
        self.count += 1
        sig = Bool()
        sig.data = True
        self.restart_pub.publish(sig)
        print("restarted!")
    
    def data_clear(self):
        self.poses = np.array([[0,0,0]])
        self.difference1 = np.array([[0,0,0]])
        self.difference2 = np.array([[0,0,0]])
        self.difference3 = np.array([[0,0,0]])
        self.queue = Queue()
        self.origin = Queue()
        self.path = Path()
        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = "world"
    
    def sigmoid(self, x):
        y = 1 / (1 + np.exp(-x))
        y = (y - 0.5) * 2
        return y
    
    def vins_callback(self, data, args):
        pose = PoseStamped()
        pose.pose = data.pose.pose
        pose.header = data.header
        # 订阅vins的回调函数，读取的数据放入到队列中
        self.queue.put(pose)
        
    def fusion_callback(self, data, args):
        pose = data
        if self.origin.empty() and pose.header.frame_id == "rope":
            print("reset o")
            self.origin.put(pose)
            self.origin.put(pose)
            self.origin.put(pose)
            self.origin.put(pose)


if __name__ == "__main__":
    relocate = Relocate(threshold=0.5, skip=15)
    relocate.start()
