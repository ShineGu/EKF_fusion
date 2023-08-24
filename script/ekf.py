#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import os
import numpy as np
import matplotlib.pyplot as plt
import math

class Quaternion:
    """
    四元数
    """
    def __init__(self) -> None:
        self.w = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
    
    def set_q(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def transform(self):
        """
        四元数->欧拉角
        """
        angles = EulerAngles()
        # roll
        sinr_cosp = 2 * (self.w * self.x + self.y * self.z)
        cosr_cosp = 1 - 2 * (self.x * self.x + self.y * self.y)
        angles.roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch
        sinp = 2 * (self.w * self.y - self.z * self.x)
        if np.abs(sinp) >= 1:
            angles.pitch = math.copysign(np.pi / 2, sinp)
        else:
            angles.pitch = math.asin(sinp)

        # yaw
        siny_cosp = 2 * (self.w * self.z + self.x * self.y)
        cosy_cosp = 1 - 2 * (self.y * self.y + self.z * self.z)
        angles.yaw = math.atan2(siny_cosp, cosy_cosp)

        return angles
    
    def rotation(self):
        """
        四元数->旋转矩阵
        """
        r = [1 - 2 * self.y ** 2 - 2 * self.z ** 2, 2 * self.x * self.y - 2 * self.z * self.w, 2 * self.x * self.z + 2 * self.y * self.w, 
             2 * self.x * self.y + 2 * self.z * self.w, 1 - 2 * self.x ** 2 - 2 * self.z ** 2, 2 * self.y * self.z - 2 * self.x * self.w, 
             2 * self.x * self.z - 2 * self.y * self.w, 2 * self.y * self.z + 2 * self.x *self.w, 1 - 2 * self.x ** 2 - 2 * self.y ** 2]
        r = np.matrix(r).reshape(3, 3)
        return r
    
class EulerAngles:
    """
    欧拉角
    """
    def __init__(self) -> None:
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def set_e(self, roll, pitch, yaw):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def transform(self):
        """
        欧拉角->四元数
        """
        q = Quaternion()
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        cp = math.cos(self.pitch * 0.5)
        sp = math.sin(self.pitch * 0.5)
        cr = math.cos(self.roll * 0.5)
        sr = math.sin(self.roll * 0.5)

        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr;
        q.y = sy * cp * sr + cy * sp * cr;
        q.z = sy * cp * cr - cy * sp * sr;

        return q

    def rotation(self):
        """
        欧拉角->旋转矩阵
        """
        r = np.matrix([0, 0, 0, 0, 0, 0, 0, 0, 0]).reshape(3, 3)
        r_x = np.matrix([1, 0, 0, 0, math.cos(self.roll), -math.sin(self.roll), 0, math.sin(self.roll), math.cos(self.roll)]).reshape(3, 3)
        r_y = np.matrix([math.cos(self.pitch), 0, math.sin(self.pitch), 0, 1, 0, -math.sin(self.pitch), 0, math.cos(self.pitch)]).reshape(3, 3)
        r_z = np.matrix([math.cos(self.yaw), -math.sin(self.yaw), 0, math.sin(self.yaw), math.cos(self.yaw), 0, 0, 0, 1]).reshape(3, 3)
        r = np.dot(np.dot(r_z, r_y), r_x)
        return r
    
def CsvData(path: str):
    """
    params: csv_path
    return: data_float
    """
    data_float = []
    with open(path, "r") as f:
        data = f.read()
        data = data.split("\n")
        data = data[1 : -1]
        for i in data:
            data_float.append(np.float128(i.split(" ")))
    f.close()
    return data_float

if __name__ == "__main__":
    pwd = os.path.abspath(os.path.dirname(os.path.dirname(__file__)))
    data_float = CsvData(pwd + "/csv/imu.csv")
    data_float_gps = CsvData(pwd + "/csv/gps_xyz.csv")
    P_docker = []
    x_k_docker = []
    z_docker = []
    z_k_docker = data_float_gps
    q_list_docker = data_float
    u_k_list_docker = data_float
    # print(u_k_list_docker)
    q = Quaternion()
    angles = EulerAngles()
    # 1/hz
    dt = 1 / 100
    # 状态x, y, z, vx, vy, vz
    x_k = np.matrix([0, 0, 0, 0, 0, 0]).reshape(6, 1)
    # [1, 0, 0, dt, 0, 0
    #  0, 1, 0, 0, dt, 0
    #  0, 0, 1, 0, 0, dt
    #  0, 0, 0, 1, 0, 0
    #  0, 0, 0, 0, 1, 0
    #  0, 0, 0, 0, 0, 1]
    A = np.matrix([1, 0, 0, dt, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1]).reshape(6, 6)
    # [0.5dt^2, 0, 0
    #  0, 0.5dt^2, 0
    #  0, 0, 0.5dt^2
    #  dt, 0, 0
    #  0, dt, 0
    #  0, 0, dt]
    B = np.matrix([0.5 * dt ** 2, 0, 0, 0, 0.5 * dt ** 2, 0, 0, 0, 0.5 * dt ** 2, dt, 0, 0, 0, dt, 0, 0, 0, dt]).reshape(6, 3)
    # Q矩阵
    Q = np.matrix(0.1 * np.eye(6, 6))
    # R矩阵
    R = np.matrix(np.eye(3, 3))
    # [1, 0, 0, 0, 0, 0
    #  0, 1, 0, 0, 0, 0
    #  0, 0, 1, 0, 0, 0]
    H = np.matrix([1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]).reshape(3, 6)
    P = np.matrix(np.eye(6, 6))
    K = 0
    I = np.matrix(np.eye(6, 6))

    plt.ion()

    for i in range(15964):
        # imu位姿
        q_list = q_list_docker[i][6 : ]
        # print(q_list)
        # imu加速度计
        u_k_list = u_k_list_docker[i][3 : 6]
        # print(u_k_list)
        q.set_q(q_list[0], q_list[1], q_list[2], q_list[3])
        angles = q.transform()
        u_k = np.dot(angles.rotation(), u_k_list).reshape(3, 1)
        # 加速度计数据 ax, ay, az
        # u_k = np.matrix([1, 1, 1]).reshape(3, 1)
        # # gps数据 x, y, z
        # z_k = np.matrix(z_k_docker[i]).reshape(3, 1)

        # 预测方程
        # x_k/k-1 = A * x_k-1/k-1 + B * u_k
        # P_k/k-1 = A * P_k-1/k-1 * A^T + Q
        x_k = np.dot(A, x_k) + np.dot(B, u_k)
        P = np.dot(np.dot(A, P), A.T) + Q
        # hz(imu) / hz(gps)
        if i % 21 == 0:
    # 更新方程
    # K_k = P_k/k-1 * H^T / (H * P_k/k-1 * H^T + R)
    # x_k/k = x_k/k-1 + K_k(z_k - H * x_k/k-1)
    # P_k/k = (I - K_k * H)P_k/k-1
            z_k = np.matrix(z_k_docker[i // 21]).reshape(3, 1)
            z_docker.append(z_k[1].tolist()[0][0])
            K = np.dot(np.dot(P, H.T), np.linalg.inv(np.dot(np.dot(H, P), H.T) + R))
            x_k = x_k + np.dot(K, z_k - np.dot(H, x_k))
            P = np.dot(I - np.dot(K, H), P)
        x_k_docker.append(x_k[1].tolist()[0][0])
        # P_docker.append(P[0].tolist()[0][0])
    
    # print(z_docker)
        fig = plt.figure(1)
        plt.clf()    
        ax1 = fig.add_subplot(1,1,1)
        z_ax = [i * 21 for i in range(len(z_docker))]
        x_ax = [j for j in range(len(x_k_docker))]
        # print(z_ax, z_docker, x_ax, x_k_docker)
        ax1.plot(z_ax, z_docker, 'k-', label="GPS")
        ax1.plot(x_ax, x_k_docker, 'b-', label="Kalman Filter")
        # ax1.plot(x_ax, P_docker, 'b-', label="cov")
        # plt.cla()
        plt.legend()
        plt.pause(0.0001)
        # plt.show()
