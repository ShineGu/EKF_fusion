#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
import pandas as pd
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
"""
gps:topic: /gps 
    msg: sensor_msgs/NavSatFix

    latitude
    longitude
    altitude
    position_covariance 

imu:topic: /imu/data
    msg: sensor_msgs/Imu 

    angular_velocity: x, y, z
    angular_velocity_covariance
    linear_acceleration: x, y, z
    linear_acceleration_covariance
    orientation: w, x, y, z
    orientation_covariance

    topic: /imu/mag
    msg: sensor_msgs/MagneticField

    magnetic_field: x, y, z
    magnetic_field_covariance

"""
class GPS:
    """
    GPS数据
    """
    def __init__(self) -> None:
        self.lng = []
        self.lat = []
        self.hight = []
        self.p_cov = []

class xyz:
    """
    xyz类型数据
    """
    def __init__(self) -> None:
        self.x = []
        self.y = []
        self.z = []

class ori:
    """
    四元数数据
    """
    def __init__(self) -> None:
        self.w = []
        self.x = []
        self.y = []
        self.z = []

class IMU:
    """
    IMU数据
    """
    def __init__(self) -> None:
        self.angular_vel = xyz()
        self.linear_acc = xyz()
        self.orientation = ori()
        # self.angular_vel_cov = []
        # self.linear_acc_cov = []
        # self.orientation_cov = []

def GPS_Callback(msg):
    """
    收集GPS数据
    """
    global flag
    flag = False
    if msg.status.status != -1:
        rospy.loginfo("start gps")
        flag = True
        global gps
        gps.lng.append(msg.longitude)
        gps.lat.append(msg.latitude)
        gps.hight.append(msg.altitude)
        gps.p_cov.append(msg.position_covariance)
    else:
        rospy.loginfo("wait for gps")

def IMU_Callback(msg):
    """
    收集IMU数据
    """
    global flag
    if flag:
        rospy.loginfo("start imu")
        global imu
        imu.angular_vel.x.append(msg.angular_velocity.x)
        imu.angular_vel.y.append(msg.angular_velocity.y)
        imu.angular_vel.z.append(msg.angular_velocity.z)
        imu.linear_acc.x.append(msg.linear_acceleration.x)
        imu.linear_acc.y.append(msg.linear_acceleration.y)
        imu.linear_acc.z.append(msg.linear_acceleration.z)
        imu.orientation.w.append(msg.orientation.w)
        imu.orientation.x.append(msg.orientation.x)
        imu.orientation.y.append(msg.orientation.y)
        imu.orientation.z.append(msg.orientation.z)
        # imu.angular_vel_cov.append(msg.angular_velocity_covariance)
        # imu.linear_acc_cov.append(msg.linear_acceleration_covariance)
        # imu.orientation_cov.append(msg.orientation_covariance)

def GPS_XYZ_Callback(msg):
    """
    收集GPS转XYZ数据
    """
    global flag
    rospy.loginfo("eeeeeeeeeee")
    if flag:
        rospy.loginfo("start gps_xyz")
        global gps_xyz
        gps_xyz.x.append(msg.pose.position.x)
        gps_xyz.y.append(msg.pose.position.y)
        gps_xyz.z.append(msg.pose.position.z)

class Write_CSV:
    """
    将数据转为CSV文件
    """
    def __init__(self) -> None:
        self.str = "test.csv"
        self.dict = {}
    
    def write(self):
        dataFrame = pd.DataFrame(self.dict)
        dataFrame.to_csv(self.str, index=False, sep=' ')

if __name__ == "__main__":
    rospy.init_node("topic_data")
    global flag
    flag = False
    # gps
    global gps
    gps = GPS()
    global gps_xyz
    gps_xyz = xyz()
    # imu
    global imu
    imu = IMU()
    # sub
    while not rospy.is_shutdown():
        gps_sub = rospy.Subscriber("/fix", NavSatFix, callback=GPS_Callback)
        gps_xyz_sub = rospy.Subscriber("/gps_pose", PoseStamped, callback=GPS_XYZ_Callback)        
        imu_sub = rospy.Subscriber("/imu/data", Imu, callback=IMU_Callback)
        rospy.spin()
    # class to dict
    # dict gps
    gps_dict = {}
    gps_xyz_dict = {}
    gps_dict["lng"] = gps.lng
    gps_dict["lat"] = gps.lat
    gps_dict["hight"] = gps.hight
    gps_dict["p_cov"] = gps.p_cov
    gps_xyz_dict["x"] = gps_xyz.x
    gps_xyz_dict["y"] = gps_xyz.y
    gps_xyz_dict["z"] = gps_xyz.z
    # dict imu
    imu_dict = {}
    imu_dict["angular_vel.x"] = imu.angular_vel.x
    imu_dict["angular_vel.y"] = imu.angular_vel.y
    imu_dict["angular_vel.z"] = imu.angular_vel.z
    imu_dict["linear_acc.x"] = imu.linear_acc.x
    imu_dict["linear_acc.y"] = imu.linear_acc.y
    imu_dict["linear_acc.z"] = imu.linear_acc.z
    imu_dict["orientation.w"] = imu.orientation.w
    imu_dict["orientation.x"] = imu.orientation.x
    imu_dict["orientation.y"] = imu.orientation.y
    imu_dict["orientation.z"] = imu.orientation.z
    # imu_dict["angular_vel_cov"] = imu.angular_vel_cov
    # imu_dict["linear_acc_cov"] = imu.linear_acc_cov
    # imu_dict["orientation_cov"] = imu.orientation_cov

    # write_csv
    write = Write_CSV()
    write.str = rospy.get_param("/topic_data/gps_csv")
    write.dict = gps_dict
    write.write()

    write.str = rospy.get_param("/topic_data/gps_xyz_csv")
    write.dict = gps_xyz_dict
    write.write()

    write.str = rospy.get_param("/topic_data/imu_csv")
    write.dict = imu_dict
    write.write()
    # rospy.spin()
