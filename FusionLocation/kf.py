import numpy as np

class KalmanFilter(object):
    # _k 表示在k时刻
    A_k = None  # 状态转移矩阵
    C_k = None  # 观测转移矩阵
    x_pre = None  # 状态预测值
    x_post = np.mat(np.zeros((3, 3)))  # 状态估计值
    R_k = None  # 预测过程中的高斯噪声的协方差矩阵
    Q_k = None  # 观测过程中的高斯噪声的协方差矩阵
    K = None  # 卡尔曼滤波增益矩阵
    P_k_pre = None  # 协方差矩阵的预测值
    P_k_post = np.mat(np.eye(3))  # 协方差矩阵的预测值

    def __init__(self, A_k=None, C_k=None, R_k=None, Q_k=None, x_post=None, P_k_post=None):
        self.A_k = A_k  # 状态转移矩阵
        self.C_k = C_k  # 观测转移矩阵
        self.R_k = R_k  # 预测过程中的高斯噪声的协方差矩阵
        self.Q_k = Q_k  # 观测过程中的高斯噪声的协方差矩阵
        # self.x_pre = x_pre  # 状态预测值
        self.x_post = x_post if x_post is not None else self.x_post  # 状态估计值
        # self.K  # 卡尔曼滤波增益矩阵
        # self.P_k_pre  # 协方差矩阵的预测值
        self.P_k_post = P_k_post if P_k_post is not None else self.P_k_post  # 协方差矩阵的预测值
        pass
    
    def predict(self):
        self.x_pre = self.A_k @ self.x_post
        self.P_k_pre = self.A_k @ self.P_k_post @ self.A_k.T + self.R_k
    
    def update(self, measurement):
        self.K = self.P_k_pre @ self.C_k.T @ (self.C_k @ self.P_k_pre @ self.C_k.T + self.Q_k).I
        self.x_post = self.x_pre + self.K @ (measurement - self.C_k @ self.x_pre)
        self.P_post = self.P_k_pre - self.K @ self.C_k @ self.P_k_pre

    def calculate(self, measurement):
        self.predict()
        self.update(measurement)
        return self.x_post


class KalmanFilterForPaper(object):
    # _k 表示在k时刻
    A_k = np.mat(np.eye(3))  # 状态转移矩阵
    C_k = np.mat(np.eye(3))  # 观测转移矩阵
    x_pre = None  # 状态预测值
    x_post = np.mat(np.zeros((3, 3)))  # 状态估计值
    R_k = None  # 预测过程中的高斯噪声的协方差矩阵
    Q_k = None  # 观测过程中的高斯噪声的协方差矩阵
    K = None  # 卡尔曼滤波增益矩阵
    P_k_pre = None  # 协方差矩阵的预测值
    P_k_post = np.mat(np.eye(3))  # 协方差矩阵的预测值

    def __init__(self, A_k=None, C_k=None, R_k=None, Q_k=None, x_post=None, P_k_post=None):
        self.A_k = A_k if A_k is not None else self.A_k  # 状态转移矩阵
        self.C_k = C_k if C_k is not None else self.C_k  # 观测转移矩阵
        self.R_k = R_k if R_k is not None else self.R_k  # 预测过程中的高斯噪声的协方差矩阵
        self.Q_k = Q_k if Q_k is not None else self.Q_k  # 观测过程中的高斯噪声的协方差矩阵
        self.x_post = x_post if x_post is not None else self.x_post  # 状态估计值
        self.P_k_post = P_k_post if P_k_post is not None else self.P_k_post  # 协方差矩阵的预测值
        pass
    
    def predict(self, prediction):
        self.x_pre = prediction
        self.P_k_pre = self.A_k @ self.P_k_post @ self.A_k.T + self.R_k
    
    def update(self, measurement):
        self.K = self.P_k_pre @ self.C_k.T @ (self.C_k @ self.P_k_pre @ self.C_k.T + self.Q_k).I
        self.x_post = self.x_pre + self.K @ (measurement - self.C_k @ self.x_pre)
        self.P_post = self.P_k_pre - self.K @ self.C_k @ self.P_k_pre

    def calculate(self, prediction, measurement):
        self.predict(prediction)
        self.update(measurement)
        return np.array(self.x_post)