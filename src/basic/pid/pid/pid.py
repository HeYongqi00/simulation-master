#!/usr/bin/env python3
## -*- coding: utf-8 -*-
# @Copyright: TIAN-JI Intell. Driv. Lab.
# @Author: ***
# @File: pid.py
# @Project: Auto-Driving System
# @CreateTime: 2022/11/01
# @Description: ***

import sys
import os
import rclpy
import numpy as np
from rclpy.node import Node
import time
import yaml
import math
import datetime
from easydict import EasyDict
from car_interfaces.msg import PidInterface
from car_interfaces.msg import LocalPathPlanningInterface
from car_interfaces.msg import GpsInterface
from car_interfaces.msg import CarOriInterface
import random

ros_node_name = "pid"
sys.path.append(os.getcwd() + "/src/basic/utils/")
sys.path.append(os.getcwd() + "/src/basic/%s/%s/"%(ros_node_name, ros_node_name))

class Pid(Node):
    def __init__(self, name):
        super().__init__(name)
    
        # define publishers
        self.fixed_time = 0.05
        self.pubPid = self.create_publisher(PidInterface, 'pid_data', 10)
        self.timerPid = self.create_timer(self.fixed_time, self.pub_callback_pid)
        
        # define subscribers
        self.subLocalPathPlanning = self.create_subscription(LocalPathPlanningInterface, 'local_path_planning_data', self.sub_callback_local_path_planning, 10)
        self.subGPS = self.create_subscription(GpsInterface, 'gps_data', self.sub_callback_gps, 10)
        self.subCarOri = self.create_subscription(CarOriInterface, "car_ori_data", self.sub_callback_car_ori, 10)
        
        
        
        root_path = os.getcwd() + "/src/basic/pid/config/"  # 读取控制参数
        yaml_file = root_path + 'config.yaml'
        
        f = open(yaml_file)
        config = yaml.load(f)
        self.yaml_data = EasyDict(config)
        print(self.yaml_data)


        self.GpsState = None
        self.RefState = None
        self.CanState = None

        self.TD_refSpeed = 0.0
        self.TD_refSpeedLast = 0.0
        self.TD_nowAccelerationLast = 0.0
        self.TD_nowAcceleration = 0.0
        self.errorSpeed_sum = 0.0
        self.braking_distance = 0.0
        self.eylast = 0
        self.ephilast = 0
        self.delta_ed_last = 0
        self.control_value_last = 0
        self.ed_sum = 0 #横向误差积分
   
   
        self.speed_now = 0
        self.speed_last = 0

        self.cout_time = 0
        
    ###############################################################
        #数据存贮
        file_name_ = os.getcwd() + '/data/results/' + datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S') +'.txt'
        self.saveData = open(file_name_,"w")
        #0, 1, 2, 3  为  refspeed, nowspeed, throttle, braking
        #4, 5, 6, 7, 8, 9 为 ref_x, ref_y, ref_yaw, nowx, nowy, nowyaw
        #10, 11, 12 为 erroryaw, errordistance, steerangle

    ###############################################################
        self.output_flag = 1 #用于降低输出频率
    def pub_callback_pid(self):
        """
        Callback of publisher (timer), publish the topic 'pid_data'.
        :param None.
        """
        #检查gps,local,can是否受到消息
        if self.GpsState is None:
            self.get_logger().warn("The GPS node is not enabled")
            return
        if self.RefState is None:
            self.get_logger().warn("The path_planner node is not enabled")
            return
        if self.CanState is None:
            self.get_logger().warn("The Can node is not enabled")
            return
        #检查轨迹信息是否正确
        if len(self.RefState.y) == 0:
            self.get_logger().error("Reference Route Error")
            return
        
        
        msgPid = PidInterface()
        msgPid.timestamp = time.time()
        
        msgPid.throttle_percentage, msgPid.braking_percentage = self.longitudinal_controller()
        
        msgPid.angle = self.lateral_controller()
        if self.output_flag%5==0:
            self.get_logger().info("longitudinal_controller: throttle = {:.2f}\tbrake = {:.2f}".
                            format(msgPid.throttle_percentage, msgPid.braking_percentage))
            self.get_logger().info("lateral_controller: angle = {:.2f}".format(msgPid.angle))
            self.output_flag = 1
        else:
            self.output_flag += 1
        #倒车相关
        if self.RefState.speed[0]<0:
            self.backoff_flag=True
            msgPid.gear=4
        elif self.RefState.speed[0]>=0:
            self.backoff_flag=False
            msgPid.gear=3

        self.pubPid.publish(msgPid)

        pass
        

    def longitudinal_controller(self):
        
        refSpeed = abs(self.RefState.speed[0])
        # nowSpeed = abs(self.CanState.carspeed)

        # 给定参考信号    self.cout_time * 0.01
        # if (self.cout_time) * self.fixed_time < 20:
        #     refSpeed = 1.5
        # elif (self.cout_time) * self.fixed_time >= 20 and (self.cout_time) * self.fixed_time < 60:
        #     refSpeed = 2.0
        # else:
        #     refSpeed = 2.0
        # self.cout_time += 1

        nowSpeed = self.speed_now
        #停车部分开环控制
        if refSpeed < 0.05:
            if nowSpeed < 0.1:
                refThrottle = 0
                refBraking = 30
            else:
                self.braking_distance = self.braking_distance + nowSpeed * self.fixed_time
                refThrottle = 0
                refBraking = int(self.yaml_data.longitudinal_dec_open_loop * self.braking_distance 
                                 + self.yaml_data.longitudinal_dec_open_loop_start)
        ###############################################################

        ###############################################################
        #加减速PID控制        
        else:
            self.braking_distance = 0

            ##################################################
            #TD算法
            TD_s = 9
            self.TD_refSpeed = self.TD_refSpeedLast + self.fixed_time*self.TD_nowAccelerationLast
            self.TD_nowAcceleration = self.TD_nowAccelerationLast + self.fixed_time*(-TD_s*TD_s*(self.TD_refSpeedLast - refSpeed) 
                                                                          - 2*TD_s*self.TD_nowAccelerationLast)
            if(self.TD_nowAcceleration < -2):
                self.TD_nowAcceleration = -2
            self.TD_refSpeedLast = self.TD_refSpeed
            self.TD_nowAccelerationLast = self.TD_nowAcceleration

            errorSpeed = nowSpeed - self.TD_refSpeed
            
            self.get_logger().info("self.speed_ref = {}\tself.speed_now = {}".format(refSpeed, self.speed_now))

            
            if(errorSpeed > 0.5):    #减速，P控制
                refBraking = errorSpeed * self.yaml_data.longitudinal_dec_kp
                refThrottle = 0
            
            else:                   #加速，PI控制,积分分离
                if(abs(errorSpeed) < 1.0):                                                                                               
                    self.errorSpeed_sum = self.errorSpeed_sum + errorSpeed
                else:
                    self.errorSpeed_sum = 0
                
                if self.GpsState.pitch > 2:#判断是否上坡，采用不同的参数
                    accPID_kp = self.yaml_data.longitudinal_acc_slope_kp
                    accPID_ki = self.yaml_data.longitudinal_acc_slope_ki                 
                else:
                    accPID_kp = self.yaml_data.longitudinal_acc_plain_kp
                    accPID_ki = self.yaml_data.longitudinal_acc_plain_ki

                refThrottle = -100*(errorSpeed * accPID_kp + self.errorSpeed_sum * accPID_ki)
                refBraking = 0
        ###############################################################

        ###############################################################
        #控制量标准化
        refThrottle = int(refThrottle)
        refBraking = int(refBraking) 
        #控制量限幅
        refThrottle = max(0, min(refThrottle, 100))#油门0-100
        refBraking = max(0, min(refBraking, 125))#刹车0-125
        ###############################################################  

        ############################################################### 
        #数据存储 #0, 1, 2, 3  为  refspeed, nowspeed, throttle, braking
        #4, 5, 6, 7, 8, 9 为 ref_x, ref_y, ref_yaw, nowx, nowy, nowyaw
        #10, 11, 12 为 erroryaw, errordistance, steerangle
        self.saveData.write("{}\t{}\t{}\t{}\t".format(refSpeed, nowSpeed, refThrottle, refBraking)) # 0,1,2,3
        self.saveData.write("{}\t{}\t{}\t".format(self.TD_refSpeed, self.TD_nowAcceleration, self.speed_now)) # 4,5,6,7,8
        ###############################################################  
        return refThrottle, refBraking

    def lateral_controller(self):
        nowSpeed = abs(self.CanState.carspeed)

        ###############################################################
        #获得车辆在参考轨迹上最近点,规划层给的第一个点就是距离车最近的点
        nearest_point_index = 0
        ###############################################################
        
        ###############################################################
        #根据速度选取预瞄点
        if(nowSpeed<3.5):
            advance_point = nearest_point_index + self.yaml_data.advance_distance_1
        elif(nowSpeed<5.5):
            advance_point = nearest_point_index + self.yaml_data.advance_distance_2
        elif(nowSpeed<7.5):
            advance_point = nearest_point_index + self.yaml_data.advance_distance_3
        else:
            advance_point = nearest_point_index + self.yaml_data.advance_distance_4
        ###############################################################

        ###############################################################
        #计算误差航向角和误差距离
        errorYaw, errorDistance = self.get_ephi_ed(advance_point)
        ############################################################### 

        ############################################################### 
        #计算方向盘转角
        if (nowSpeed < 3.5):
            pidEphi_kp = self.yaml_data.lateral_ephi_kp1
            pidEd_kp = self.yaml_data.lateral_ed_kp1
            pidEd_ki = self.yaml_data.lateral_ed_ki1
            pidEphi_kd = self.yaml_data.lateral_ephi_kd1
            pidEd_kd = self.yaml_data.lateral_ed_kd1
        elif (nowSpeed < 5.5):
            pidEphi_kp = self.yaml_data.lateral_ephi_kp2
            pidEd_kp = self.yaml_data.lateral_ed_kp2
            pidEd_ki = self.yaml_data.lateral_ed_ki2
            pidEphi_kd = self.yaml_data.lateral_ephi_kd2
            pidEd_kd = self.yaml_data.lateral_ed_kd2
        elif (nowSpeed < 6.5):
            pidEphi_kp = self.yaml_data.lateral_ephi_kp3
            pidEd_kp = self.yaml_data.lateral_ed_kp3
            pidEd_ki = self.yaml_data.lateral_ed_ki3
            pidEphi_kd = self.yaml_data.lateral_ephi_kd3
            pidEd_kd = self.yaml_data.lateral_ed_kd3
        else:
            pidEphi_kp = self.yaml_data.lateral_ephi_kp4
            pidEd_kp = self.yaml_data.lateral_ed_kp4
            pidEd_ki = self.yaml_data.lateral_ed_ki4
            pidEphi_kd = self.yaml_data.lateral_ephi_kd4
            pidEd_kd = self.yaml_data.lateral_ed_kd4


        #积分分离
        if abs(errorDistance)>0.45:
            self.ed_sum = 0
        else:
            self.ed_sum = self.ed_sum + errorDistance
        if self.ed_sum>3:
            self.ed_sum=3
        if self.ed_sum<-3:
            self.ed_sum=-3
            
        controlValue = (pidEphi_kp*errorYaw + pidEd_kp*errorDistance 
                        + pidEphi_kd*(errorYaw-self.ephilast)+pidEd_kd*(errorDistance-self.eylast) 
                        + pidEd_ki*self.ed_sum)*180/math.pi    #转化成角度输出
        
        
        controlValue = controlValue/30.0+0.028
        # 一阶惯性滤波
        # inertia_factor = 0.3
        # controlValue = inertia_factor * controlValue + (1 - inertia_factor) * self.control_value_last
        # self.control_value_last = controlValue
        self.eylast = errorDistance
        self.ephilast = errorYaw

        if(controlValue > 1):
            controlValue = 1.0
        if(controlValue < -1):
            controlValue = -1.0
        ############################################################### 

        ############################################################### 
        #数据存储 #0, 1, 2, 3  为  refspeed, nowspeed, throttle, braking
        #4, 5, 6, 7, 8, 9 为 ref_x, ref_y, ref_yaw, nowx, nowy, nowyaw
        #10, 11, 12 为 erroryaw, errordistance, steerangle
        # self.saveData.write("{}\t{}\t{}\t{}\t{}\n".format(errorYaw, errorDistance, controlValue, self.TD_nowAcceleration, self.TD_refSpeed)) #10-14
        ###############################################################  
        
        return controlValue      

    def get_ephi_ed(self, advance_point):
        
        #越界保护
        if advance_point > len(self.RefState.x) - 1:
            advance_point = len(self.RefState.x) - 1
        #------------------------#

        refX = self.RefState.x[advance_point]
        refY = self.RefState.y[advance_point]
        refYaw = self.RefState.angle[advance_point]
        
        
        nowX = self.GpsState.x
        nowY = self.GpsState.y  
        nowYaw = self.GpsState.yaw
        nowSpeed = abs(self.CanState.carspeed)
     
        errorYaw =  refYaw - nowYaw


        if(errorYaw > math.pi):
            errorYaw = errorYaw - 2*math.pi
        if(errorYaw < -math.pi):
            errorYaw = errorYaw + 2*math.pi

        errorDistance = ((refY - nowY) * math.cos(refYaw) - (refX - nowX) * math.sin(refYaw))
        if self.output_flag%5==0:
            self.get_logger().info("nowX = {:.2f}\t nowY = {:.2f}\t nowYaw = {:.2f}".format(nowX, nowY, nowYaw))
            self.get_logger().info("refX = {:.2f}\t refY = {:.2f}\t refYaw = {:.2f}".format(refX, refY, refYaw))
            self.get_logger().info("errorDistance = {:.2f}\terrorYaw = {:.2f}".format(errorDistance,errorYaw))

        ############################################################### 
        #数据存储 #0, 1, 2, 3  为  refspeed, nowspeed, throttle, braking
        #4, 5, 6, 7, 8, 9 为 refX, refY, refYaw, nowx, nowy, nowyaw
        #10, 11, 12 为 erroryaw, errordistance, steerangle
        # self.saveData.write("{}\t{}\t{}\t{}\t{}\t{}\t".format(refX, refY, refYaw, nowX, nowY, nowYaw)) # 4-9
        self.saveData.write("{}\t{}\t{}\t{}\t{}\t{}\n".format(refX, refY, refYaw, nowX, nowY, nowYaw)) # 9-14
        ############################################################### 
        if nowSpeed < 6:
            if errorDistance > 2.5:
                errorDistance = 2.5
            if errorDistance < -2.5:
                errorDistance = -2.5
        else:
            if errorDistance > 2:
                errorDistance = 2
            if errorDistance < -2:
                errorDistance = -2

        return errorYaw, errorDistance



    def sub_callback_local_path_planning(self, msgLocalPathPlanning:LocalPathPlanningInterface):
        """
        Callback of subscriber, subscribe the topic 'local_path_planning_data'.
        :param msgLocalPathPlanning: The message heard by the subscriber.
        """
        self.RefState = msgLocalPathPlanning
        

    def sub_callback_gps(self, msgGps:GpsInterface):
        self.GpsState = msgGps
        self.GpsState.yaw = msgGps.yaw/180*math.pi  #角度转化弧度

        self.pitch = self.GpsState.pitch
        
        self.speed_now = pow(pow(self.GpsState.northvelocity,2)+pow(self.GpsState.eastvelocity,2), 0.5) #由GPS解出速度和加速度
        
        

    def sub_callback_car_ori(self, msgCarOri:CarOriInterface):
        """
        Callback of subscriber, subscribe the topic car_ori_data.
        :param msgCarOri: The message heard by the subscriber.
        """
        self.CanState = msgCarOri



        
def main():
    rclpy.init()
    rosNode = Pid(name='pid')
    rclpy.spin(rosNode)
    rclpy.shutdown()
