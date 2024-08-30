#!/usr/bin/env python3
## -*- coding: utf-8 -*-
# @Copyright: TIAN-JI Intell. Driv. Lab.
# @Author: HeYongqi
# @File: longitudinal_mpc.py
# @Project: Auto-Driving System
# @CreateTime: 2022/12/05
# @Description: The upper longitudinal controller is based on MPC, 
# the bottom longitudinal controller is based on feedforward and PID,
# the lateral controller is based on PID.

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
import casadi as ca


ros_node_name = "longitudinal_mpc"

sys.path.append(os.getcwd() + "/src/extension/utils/")
sys.path.append(os.getcwd() + "/src/extension/%s/%s/"%(ros_node_name, ros_node_name))

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
        
        root_path = os.getcwd() + "/src/extension/longitudinal_mpc/config/"  # 读取控制参数
        yaml_file = root_path + 'config.yaml'
        f = open(yaml_file)
        config = yaml.load(f)
        self.yaml_data = EasyDict(config)
        print(self.yaml_data)


        self.GpsState = None
        self.RefState = None
        self.CanState = None
        
        self.eylast = 0
        self.ephilast = 0
        self.delta_ed_last = 0
        self.control_value_last = 0
        self.ed_sum = 0 #横向误差积分
        
        self.refspeed = 0
        self.speed_now = 0.0
        self.speed_last = 0.0
        self.acceleration = np.zeros([10,1])
        self.acceleration_now = 0.0
        self.acceleration_sum = 0.0
        self.speed_sum = 0.0
        self.refacceleration = 0.0

        self.cout_time = 0
        
        
        ### 上层MPC控制器参数
        self.Q_1 = np.array([1.50])
        self.Q_2 = np.array([0.2])
        self.R = np.array([0.7])
        self.horizon = 10                    #预测步长

        ### MPC 模型参数
        self.tau = 0.35
        self.Km = 1.0
        self.a_1_1 = 1.0
        self.a_1_2 = self.fixed_time
        self.a_2_2 = (1 - self.fixed_time / self.tau)
        self.b_2 = self.Km * self.fixed_time / self.tau   
        ### 底层 前馈 + PI控制参数    
        self.e_acc_kp = 0.40
        self.e_acc_ki = 0.040
        
        
    ###############################################################
        #数据存贮
        file_name_ = os.getcwd() + '/data/results/' + datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S') +'.txt' 
        self.saveData = open(file_name_,"w")
        
        # 数据存储 # 1, 2, 3, 4 为refspeed, speed_now, refacceleration, acceleration_now
        # 5, 6, 7  为 refThrottle, refBraking, process_time
        # 8, 9, 10 为 扩展纵向数据
        # 11, 12, 13, 14, 15, 16  为 refX, refY, refYaw, nowX, nowY, nowYaw
        # 17, 18, 19, 20 为 errorYaw, errorDistance, controlValue, process_time


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
        
        msgPid.throttle_percentage, msgPid.braking_percentage = self.longitudinal_mpc_controller()    #纵向MPC控制器

        msgPid.angle = self.lateral_controller()                                                      #横向PID控制器
        
        if self.output_flag%5==0:
            self.get_logger().info("longitudinal_controller: throttle = {:.2f}\tbrake = {:.2f}".
                            format(msgPid.throttle_percentage, msgPid.braking_percentage))
            self.get_logger().info("lateral_controller: angle = {:.2f}".format(msgPid.angle))
            self.output_flag = 1
        else:
            self.output_flag += 1
        #倒车相关
        print("speed:::::::::::::",self.RefState.speed[0])
        if self.RefState.speed[0]<0:
            self.backoff_flag=True
            msgPid.gear=4
        elif self.RefState.speed[0]>=0:
            self.backoff_flag=False
            msgPid.gear=3

        self.pubPid.publish(msgPid)

        pass

    def longitudinal_mpc_controller(self):
        self.start_time = time.time()  #计时
        # self.refspeed = self.RefState.speed[0]  #获取参考速度

        # 给定参考信号    self.cout_time * 0.01
        if (self.cout_time) * self.fixed_time < 20:
            self.refspeed = 1.5
        elif (self.cout_time) * self.fixed_time >= 20 and (self.cout_time) * self.fixed_time < 60:
            self.refspeed = 2.0
        else:
            self.refspeed = 2.0
        self.cout_time += 1    
        
        
        refspeed_array_ = self.refspeed * np.ones([self.horizon + 1,1]) #MPC参考速度序列
         
        opti = ca.Opti()
        ca_v = opti.variable(self.horizon + 1, 1)
        ca_a = opti.variable(self.horizon + 1, 1)
        ca_ades = opti.variable(self.horizon, 1)
        
        opti.subject_to(ca_v[0] == self.speed_now)
        opti.subject_to(ca_a[0] == self.acceleration_now)
        
        for i in range(0, self.horizon):      #模型
            opti.subject_to(ca_v[i+1] == self.a_1_1 * ca_v[i] + self.a_1_2 * ca_a[i])
            opti.subject_to(ca_a[i+1] == self.a_2_2 * ca_a[i] + self.b_2 * ca_ades[i])
        
        for i in range(0, self.horizon):          #输入约束
            opti.subject_to(ca_ades[i] >= -1.0)
            opti.subject_to(ca_ades[i] <= 1.0)
            
        # 标准MPC代价函数
        cost = 0
        Q_1 = np.diag(self.Q_1)
        Q_2 = np.diag(self.Q_2)
        R_ = np.diag(self.R)

        cost += ca.mtimes([(refspeed_array_ - ca_v).T, Q_1, (refspeed_array_ - ca_v)])
        cost += ca.mtimes([ca_a.T, Q_2, ca_a])
        cost += ca.mtimes([ca_ades.T, R_, ca_ades])
        opti.minimize(cost)


        # 求解
        opts_setting = {'ipopt.max_iter':100, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6}
        opti.solver('ipopt', opts_setting)
        sol = opti.solve()
        self.refacceleration = sol.value(ca_ades[0])
        print("ades = {}".format(self.refacceleration))

        # 加速度限幅
        if self.refacceleration > 1.0:
            self.refacceleration = 1.0
        if self.refacceleration < -1.0:
            self.refacceleration = -1.0


        ##################底层控制器#######################
        e_acc = self.refacceleration - self.acceleration_now
        
        # 积分分离
        if e_acc > -0.08 and e_acc < 0.08:
            self.acceleration_sum = self.acceleration_sum + e_acc
        else:
            self.acceleration_sum = 0
        
        # 前馈控制参数选择
        if self.speed_now > 3.0:
            feed_forward_kp = 0.75
        elif self.speed_now < 2.0:
            feed_forward_kp = 1.65
        else:
            feed_forward_kp = 0.75

        refThrottle = 15 + feed_forward_kp * self.speed_now + 100 * (self.e_acc_kp * e_acc + self.e_acc_ki * self.acceleration_sum)  ## 底层PI控制器 
        refBraking = 0

        if refThrottle < 0:     #制动减速
            refBraking = abs(refThrottle)
            refThrottle = 0

        #控制量标准化
        refThrottle = int(refThrottle)
        refBraking = int(refBraking)
        

        #控制量限幅
        refThrottle = max(0, min(refThrottle, 100))#油门0-100
        refBraking = max(0, min(refBraking, 125))#刹车0-125

        self.end_time = time.time()
        process_time = self.end_time - self.start_time
        print("longitudinal controller runing time = {}".format(process_time))
        
        # 数据存储 # 1, 2, 3, 4 为refspeed, speed_now, refacceleration, acceleration_now
        # 5, 6, 7  为 refThrottle, refBraking, process_time
        # 8, 9, 10 为 扩展纵向数据
        # 11, 12, 13, 14, 15, 16  为 refX, refY, refYaw, nowX, nowY, nowYaw
        # 17, 18, 19, 20 为 errorYaw, errorDistance, controlValue, process_time
        
        self.saveData.write("{}\t{}\t{}\t{}\t".format(self.refspeed, self.speed_now, self.refacceleration, self.acceleration_now)) # 1, 2, 3, 4
        self.saveData.write("{}\t{}\t".format(refThrottle, refBraking, process_time)) # 5, 6, 7

        return refThrottle, refBraking
                 


    def lateral_controller(self):
        self.start_time = time.time()  #计时模块
        nowSpeed = abs(self.speed_now)

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
        if abs(errorDistance) > 0.45:
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

        self.eylast = errorDistance
        self.ephilast = errorYaw

        if(controlValue > 1):
            controlValue = 1.0
        if(controlValue < -1):
            controlValue = -1.0
        ###############################################################
        self.end_time = time.time()
        process_time = self.end_time - self.start_time
        ############################################################### 
        # 数据存储 # 1, 2, 3, 4 为refspeed, speed_now, refacceleration, acceleration_now
        # 5, 6, 7  为 refThrottle, refBraking, process_time
        # 8, 9, 10 为 扩展纵向数据
        # 11, 12, 13, 14, 15, 16  为 refX, refY, refYaw, nowX, nowY, nowYaw
        # 17, 18, 19, 20 为 errorYaw, errorDistance, controlValue, process_time
        self.saveData.write("{}\t{}\t{}\t{}\n".format(errorYaw, errorDistance, controlValue, process_time)) # 17, 18, 19, 20
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
        
        nowSpeed = abs(self.speed_now)
     
        errorYaw =  refYaw - nowYaw


        if(errorYaw > math.pi):
            errorYaw = errorYaw - 2*math.pi
        if(errorYaw < -math.pi):
            errorYaw = errorYaw + 2*math.pi

        errorDistance = ((refY - nowY) * math.cos(refYaw) - (refX - nowX) * math.sin(refYaw))

        ############################################################### 
        # 数据存储 # 1, 2, 3, 4 为refspeed, speed_now, refacceleration, acceleration_now
        # 5, 6, 7  为 refThrottle, refBraking, process_time
        # 8, 9, 10 为 扩展纵向数据
        # 11, 12, 13, 14, 15, 16  为 refX, refY, refYaw, nowX, nowY, nowYaw
        # 17, 18, 19, 20 为 errorYaw, errorDistance, controlValue, process_time
        self.saveData.write("{}\t{}\t{}\t{}\t{}\t{}\t".format(refX, refY, refYaw, nowX, nowY, nowYaw)) # 11, 12, 13, 14, 15, 16
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
        
        self.speed_now = pow(pow(self.GpsState.northvelocity,2)+pow(self.GpsState.eastvelocity,2), 0.5) #由GPS解出速度

        
        for i in range(0, self.acceleration.shape[0] - 1):
            self.acceleration[i] = self.acceleration[i + 1]
        self.acceleration[-1] = (self.speed_now - self.speed_last) / self.fixed_time           #差分计算加速度
        self.acceleration_now = np.average(self.acceleration)
    
        self.speed_last = self.speed_now
        
        # self.get_logger().info("self.speed_now = {}\t self.acceleration_now  = {}".format(self.speed_now, self.acceleration_now))
    
    def sub_callback_car_ori(self, msgCarOri:CarOriInterface):
        """
        Callback of subscriber, subscribe the topic car_ori_data.
        :param msgCarOri: The message heard by the subscriber.
        """
        self.CanState = msgCarOri


def main():
    rclpy.init()
    rosNode = Pid(name=ros_node_name)
    rclpy.spin(rosNode)
    rclpy.shutdown()
