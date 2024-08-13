#!/usr/bin/env python3
## -*- coding: utf-8 -*-
# @Copyright: TIAN-JI Intell. Driv. Lab.
# @Author: yangkai
# @File: gps.py
# @Project: Auto-Driving System
# @CreateTime: 2022/11/01
# @Description: ***

import sys
import os
import rclpy
import time
from rclpy.node import Node
from scipy.interpolate import griddata
from car_interfaces.msg import GpsInterface
from car_interfaces.msg import CarOriInterface
from car_interfaces.msg import PidInterface
import math
import copy
import numpy as np
import threading
from threading import Thread

ros_node_name = "sim_model"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/"%(ros_node_name, ros_node_name))
file_name = os.getcwd() + "/src/basic/sim_model/config/" + "calibration_table.txt"

def GetLonData(file_name):
    """
    获取纵向模型数据

    Args:
        file_name (_type_): 映射表存放地址

    Returns:
        _type_: x包含速度和油门或刹车,y是加速度
    """
    speed = []
    acceleration = []
    command = []
    with open(file_name, 'r') as file:
        # 逐行读取文件内容
        for line in file:
            # 去除首尾空白字符
            line = line.strip()
            # 如果当前行不是空行且不是 calibration 数据块的开头
            if not line.startswith("calibration {") and not line.startswith("}"):
                # 解析字段名和字段值
                field_name, field_value = line.split(": ")
                field_name = field_name.strip()
                if field_name == 'speed':
                    speed.append(float(field_value))
                elif field_name == 'acceleration':
                        acceleration.append(float(field_value))
                elif field_name == 'command':
                    if(float(field_value) > 0):
                        command.append(float(field_value)/80.0)
                    else:
                        command.append(float(field_value)/35.0)
    # 使用列表推导式将 speed 和 command 合并成 x
    x = [[xi, xj] for xi, xj in zip(speed, command)]
    y = acceleration
    return x, y
            
class SimModel(Node):
    def __init__(self, name):
        super().__init__(name)
        self.fixed_time = 0.05  # gps 发布频率
        self.msgGps = GpsInterface()
        self.msgCarOri = CarOriInterface()
        
        
        """east_to_west初始化参数"""
        self.declare_parameter('x', 314.0)
        self.declare_parameter('y', -158.0)
        self.declare_parameter('yaw', 3.16)
        self.declare_parameter('speed_now', 0.0)
        self.declare_parameter('delta', 0.0)
        
        self.msgGps.x = self.get_parameter('x').value
        self.msgGps.y = self.get_parameter('y').value
        self.msgGps.yaw = self.get_parameter('yaw').value  
        self.msgGps.northvelocity = self.get_parameter('speed_now').value  # 纵向速度
        self.msgCarOri.steerangle = self.get_parameter('delta').value

        self.msgGps.yaw = self.msgGps.yaw/math.pi*180 #转换成角度
        self.yaw_radian = self.msgGps.yaw / 180 * math.pi #转化成弧度
        
        self.L = 1.923
        self.msgPid = None


        # define publishers
        self.pubGps = self.create_publisher(GpsInterface, 'gps_data', 10)
        self.pubCarOri = self.create_publisher(CarOriInterface, 'car_ori_data', 10)  
        
        # define subscriber
        self.subPid = self.create_subscription(PidInterface, 'pid_data', self.CallbackPid, 10)
        
        # define timer
        self.timerGps = self.create_timer(self.fixed_time, self.UpdataState)

        self.output_flag = 1 #输出标志位，用于降低输出频率
        
        self.x, self.y = GetLonData(file_name)
        pass

    def UpdataState(self):
        """更新车辆状态信息
        仿真中northvelocity作为车辆速度,ax为车辆加速度
        """
        self.msgGps.timestamp = time.time()
        self.msgGps.id = 0 #id
        
        speed_now_ = self.msgGps.northvelocity
        if self.msgPid is not None:
            self.delta = (self.msgPid.angle - 0.028)  # 获取前轮转角
            if self.msgPid.throttle_percentage >= 0:
                command_ = float(self.msgPid.throttle_percentage)/100.0
            else:
                command_ = -float(self.msgPid.braking_percentage)/125.0
            now_ts = time.time()
            inter_val_ = griddata(self.x, self.y, np.array([[speed_now_, command_]]), method='linear')
            process_time = time.time() - now_ts
            self.get_logger().info("process_time = {:.4f}".format(process_time))
            self.msgGps.ax = inter_val_[0]
            
            speed_now_ += self.fixed_time * self.msgGps.ax
            speed_now_ = max(speed_now_, 0.0)  # 不考虑速度为正的情况
            self.get_logger().info("self.msgGps.ax = {:.2f}\t speed_nwo = {:.2f}".format(self.msgGps.ax, speed_now_))
            self.yaw_radian += self.fixed_time  * (speed_now_ * math.tan(self.delta) / self.L) #偏航角
            self.msgGps.x += self.fixed_time  * speed_now_ * math.cos(self.yaw_radian)    #加速度x
            self.msgGps.y += self.fixed_time  * speed_now_ * math.sin(self.yaw_radian)    #加速度y
        
            self.msgGps.yaw = self.yaw_radian / math.pi * 180.0
            self.msgGps.northvelocity = speed_now_
            if(self.msgGps.yaw > 360):
                self.msgGps.yaw = self.msgGps.yaw - 360
            if(self.msgGps.yaw < 0):
                self.msgGps.yaw = self.msgGps.yaw + 360
                
                
            self.msgCarOri.carspeed  = speed_now_
            self.msgCarOri.steerangle = self.msgCarOri.steerangle + self.fixed_time * (-self.msgCarOri.steerangle / 0.9 + 550.0 * self.msgPid.angle / 0.9) 
            
        else:
            pass
        

        if self.output_flag%10==0:
            self.get_logger().info("msgGps.x = {:.2f}\tmsgGps.y= {:.2f}\tmsgGps.yaw = {:.2f}".format(self.msgGps.x, self.msgGps.y, self.msgGps.yaw))
            self.output_flag = 1
        else:
            self.output_flag += 1
    
        self.pubGps.publish(self.msgGps)
        self.pubCarOri.publish(self.msgCarOri)
        
        
    def CallbackPid(self, msgPid:PidInterface):
        """
        Callback of subscriber (timer), subscribe the topic 'pid_data'.
        接受pid消息
        """
        self.msgPid = msgPid
        
        

def main():
    rclpy.init()
    rosNode = SimModel(name=ros_node_name)
    try:
        rclpy.spin(rosNode)
    except KeyboardInterrupt:
        rclpy.shutdown()