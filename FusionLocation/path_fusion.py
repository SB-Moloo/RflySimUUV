import rospy
import message_filters
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from kf import KalmanFilterForPaper as KF

class PathFusion:
    def __init__(self):
        rospy.init_node("path_fusion")
        self.path_pub = rospy.Publisher("/rflysim/fusion/path", Path, queue_size=10)
        self.pose_pub = rospy.Publisher("/rflysim/fusion/pose", PoseStamped, queue_size=10)
        self.path = Path()
        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = "world"
        R = 100  # 预测过程中的高斯噪声的协方差矩阵
        Q = 30  # 观测过程中的高斯噪声的协方差矩阵
        self.kf = KF(Q_k=Q, R_k=R)
        # self.vins = np.array([[]])
        # self.rope = np.array([[]])


    def ts_callback(self, vins_pose, rope_pose):
        quat = rope_pose.pose.orientation
        rope = rope_pose.pose.position
        vins = vins_pose.pose.position
        rope = np.array([[rope.x, rope.y, rope.z]]).T
        vins = np.array([[vins.x, vins.y, vins.z]]).T
        ans = None
        pose = PoseStamped()
        if vins_pose.header.frame_id != "error":
            ans = self.kf.calculate(vins, rope)
            pose.header.frame_id = "fusion"
        else:
            ans = (rope.T)[0]
            pose.header.frame_id = "rope"
        # 定义数据结构
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = ans[0]
        pose.pose.position.y = ans[1]
        pose.pose.position.z = ans[2]
        pose.pose.orientation.w = quat.w
        pose.pose.orientation.x = quat.x
        pose.pose.orientation.y = quat.y
        pose.pose.orientation.z = quat.z
        self.path.poses.append(pose)
        # 发布数据
        self.path_pub.publish(self.path)
        self.pose_pub.publish(pose)


    def data_synchronization(self):
        self.vins_sub = message_filters.Subscriber('/rflysim/relocate/pose', PoseStamped)
        self.rope_sub = message_filters.Subscriber('/rflysim/rope/pose', PoseStamped)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.vins_sub, self.rope_sub], 100, 0.5, allow_headerless=True)
        self.ts.registerCallback(self.ts_callback)


    def start(self):
        self.data_synchronization()
        rate = rospy.Rate(200)
        while not rospy.is_shutdown():
            rospy.spin()
            rate.sleep()


if __name__ == '__main__':
    path_fusion = PathFusion()
    path_fusion.start()
