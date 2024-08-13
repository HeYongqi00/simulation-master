#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
# @Author: He Yongqi 
# @Email yongqihe@tju.edu.cn
# @File: hmi.py
# @Date: 2023/07/19
# @Description: 全局参考路径，规划路径，和车辆位置可视化
"""
import os
import sys
import time
import math
import numpy as np
import cv2
import rclpy                                     
from rclpy.node import Node                     
from car_interfaces.msg import GpsInterface
from car_interfaces.msg import LocalPathPlanningInterface
from car_interfaces.msg import GlobalPathPlanningInterface
from car_interfaces.msg import CarOriInterface
from car_interfaces.msg import PidInterface
from car_interfaces.msg import LocalPredictionInterface

ros_node_name = "hmi"
sys.path.append(os.getcwd() + "/src/%s/%s/"%(ros_node_name, ros_node_name))


def cv2AddText(canvas, text_name, text_pose_x, text_pose_y, text_data, font):
    """文本和数字输出

    Args:
        canvas (_type_): 画布
        text_name (_type_): 文本矩阵
        text_pose_x (_type_): 文本x坐标矩阵
        text_pose_y (_type_): 文本y坐标矩阵
        text_data (_type_): 文本数据
        font (_type_): 字体
    """
    for i in range(0,text_name.shape[0]):
        for j in range(0,text_name.shape[1]):
            if text_name[i][j] == 'Ref' or text_name[i][j] == 'Actual':
                text_string = text_name[i][j]
            else:
                text_string = text_name[i][j] + ':' + '{:.2f}'.format(text_data[i][j])
            cv2.putText(canvas, text_string, (text_pose_x[j],text_pose_y[i]), font, 0.60, (0, 0, 255), 1)
    pass


def world_to_canvas(world_auto_pose, canvas_auto_pose, world_path_pose, scale):
    """坐标转换，从位置坐标转换到可视化像素坐标

    Args:
        world_auto_pose (_type_): 世界坐标系下自动驾驶车辆位置和姿态信息[x y yaw] yaw为弧度
        canvas_auto_pose (_type_): 画布坐标系下自动驾驶车辆位置信息[x y]
        world_path_pose (_type_): 世界坐标系下路径位置信息[x y]
        scale (_type_): _世界坐标系到画布坐标系的距离扩大系数 [scale_x scale_y]

    Returns:
        _type_: 返回画布坐标系下参考路径位置
    """

    n = np.size(world_path_pose, 0)
    
    world_delta_ = np.zeros((n,2), dtype=float)
    local_delta_ = np.zeros((n,2), dtype=float)
    canvas_path_pose = np.zeros((n,2), dtype=int)
    for i in range(0, n):
        world_delta_[i,0] = world_path_pose[i,0] - world_auto_pose[0]  #世界坐标系下差值
        world_delta_[i,1] = world_path_pose[i,1] - world_auto_pose[1]
        
        local_delta_[i,0] = world_delta_[i,0]*math.cos(world_auto_pose[2]) + world_delta_[i,1]*math.sin(world_auto_pose[2])
        local_delta_[i,1] = - world_delta_[i,0]*math.sin(world_auto_pose[2]) + world_delta_[i,1]*math.cos(world_auto_pose[2])

        canvas_path_pose[i,0] = -local_delta_[i,1]*scale[0] + canvas_auto_pose[0]
        canvas_path_pose[i,1] = -local_delta_[i,0]*scale[1] + canvas_auto_pose[1]
    return canvas_path_pose


class Hmi(Node):
    def __init__(self, name):
        super().__init__(name)                     # ROS2节点父类初始化

        ############################ 初始化默认参数 #########################
        self.autoPose = None  # 车辆自身状态信息
        self.planPath = None # 初始化planpath的msg，规划路径点
        self.globalPath = None
        self.CanData = None
        self.poseData = None
        self.predictionPath = None
        self.msgPid = None

        ############################ 订阅话题，检查话题名称 #########################
        self.autoPose_subscriber = self.create_subscription(GpsInterface, "gps_data", self.autoPose_callback, 10)
        self.planPath_subscriber = self.create_subscription(LocalPathPlanningInterface, "local_path_planning_data", self.planPath_callback, 10)
        self.globalPath_subscriber = self.create_subscription(GlobalPathPlanningInterface, "global_path_planning_data", self.globalPath_callback, 10)
        self.subCarOri = self.create_subscription(CarOriInterface, "car_ori_data", self.sub_callback_car_ori, 10)
        self.subPid = self.create_subscription(PidInterface, "pid_data", self.sub_callback_pid, 10)
        self.subPrediction =self.create_subscription(LocalPredictionInterface, "local_prediction_data", self. sub_callback_prediction, 10)
    
        ############################ 定时器，绘制画布进行可视化 ######################
        self.planningVis_timer = self.create_timer(0.10, self.hmi_callback)


        self.text_name = np.array([['x','y','yaw','v'],['ex','ey','eyaw','ev'],
                                   ['Ref','Thr','Bra','Steer'],['Actual','Thr','Bra','Steer']])
        self.text_pose_x = np.array([5,105,215,335])
        self.text_pose_y = np.array([715,740,765,790])
        self.text_data = np.zeros((4,4), dtype=float)
    
    def autoPose_callback(self, msg:GpsInterface):
        """
        存入车辆当前位置信息
        """
        self.autoPose = msg
        self.auto_pose_flag = 0
        self.autoPose.yaw = self.autoPose.yaw/180*math.pi
        self.text_data[0,0] = self.autoPose.x
        self.text_data[0,1] = self.autoPose.y
        self.text_data[0,2] = self.autoPose.yaw
        self.text_data[0,3] = pow(pow(self.autoPose.northvelocity,2)+pow(self.autoPose.eastvelocity,2), 0.5)  #gps解算速度

        # self.text_data[0,3] = self.CanData.carspeed
    
    
    def planPath_callback(self, msg:LocalPathPlanningInterface):
        """
        存入局部路径信息
        """
        self.planPath = msg
        if self.planPath is not None and self.autoPose is not None:
            
            dx = self.planPath.x[0] - self.autoPose.x
            dy = self.planPath.y[0] - self.autoPose.y
            error_yaw_ = self.planPath.angle[0] - self.autoPose.yaw
            
            error_y_ = dy * math.cos(self.planPath.angle[0]) - dx * math.sin(self.planPath.angle[0])
            error_x_ = dy * math.sin(self.planPath.angle[0]) + dx * math.cos(self.planPath.angle[0])
            if(error_yaw_ > math.pi):
                error_yaw_ = error_yaw_ - 2*math.pi
            if(error_yaw_ < -math.pi):
                error_yaw_ = error_yaw_ + 2*math.pi
            # if(self.msgPid is None):
            self.text_data[1,0] = error_x_
            self.text_data[1,1] = error_y_
            self.text_data[1,2] = error_yaw_
            self.text_data[1,3] = self.planPath.speed[0] - pow(pow(self.autoPose.northvelocity,2)+pow(self.autoPose.eastvelocity,2), 0.5)
    
    def sub_callback_pid(self, msg:PidInterface):
        """
        存入控制信息
        """
        self.msgPid = msg
        self.text_data[2,1] = self.msgPid.throttle_percentage
        self.text_data[2,2] = self.msgPid.braking_percentage
        self.text_data[2,3] = self.msgPid.angle
    
    def sub_callback_prediction(self, msg:LocalPredictionInterface):
        """
        存入预测路径信息
        """
        self.predictionPath = msg
        self.text_data[1,0] = self.predictionPath.error_x
        self.text_data[1,1] = self.predictionPath.error_y
        self.text_data[1,2] = self.predictionPath.error_yaw
        self.text_data[1,3] = self.predictionPath.error_speed
    
    def globalPath_callback(self, msg:GlobalPathPlanningInterface):
        """
        全局路径信息
        """
        self.globalPath = msg
        world_global_pose_x_ = np.array([])
        world_global_pose_y_ = np.array([])
        for i in range(0, len(self.globalPath.routedata)):
            if i%2==0:
                world_global_pose_x_ = np.append(world_global_pose_x_, self.globalPath.routedata[i])
            else:
                world_global_pose_y_ = np.append(world_global_pose_y_, self.globalPath.routedata[i])
        self.world_global_pose = np.concatenate([world_global_pose_x_.reshape(-1,1), world_global_pose_y_.reshape(-1,1)], axis=1)
    def sub_callback_car_ori(self, msgCarOri:CarOriInterface):
        """
        Callback of subscriber, subscribe the topic car_ori_data.
        :param msgCarOri: The message heard by the subscriber.
        """
        self.CanData = msgCarOri
        self.text_data[3,1] = self.CanData.throttle_percentage
        self.text_data[3,2] = self.CanData.braking_percentage
        self.text_data[3,3] = self.CanData.steerangle/550.0
        

    def hmi_callback(self):
        ########################## 画布属性配置 ###############################
        start_time = time.time()
        cv2.namedWindow("hmi", cv2.WINDOW_AUTOSIZE)                         #名称
        font = cv2.FONT_HERSHEY_TRIPLEX                                     #设置字体
        scale = np.array([45, 15])                                          #scale 相对距离1m代表多少个像素
        canvas_height, canvas_width = 800, 800                              #画布长宽
        canvas = np.ones((canvas_height, canvas_width, 3), np.uint8) * 255  #画布底色为白色 (255, 255, 255)
        canvas_auto_pose = (int(canvas_width/2),int(3*canvas_height/5))     #画布中车辆的固定位置
        
        if self.autoPose is not None:
            cv2.circle(canvas, canvas_auto_pose, 5, (255,0,0), -1)  #画出车辆当前位置 蓝色
            if self.poseData is None:                               #历史轨迹
                self.poseData = np.append([[]], [[self.autoPose.x, self.autoPose.y]], axis=1)
            else:
                self.poseData = np.append(self.poseData, [[self.autoPose.x, self.autoPose.y]], axis=0)
                if np.size(self.poseData, 0) > 40:
                    self.poseData =  np.delete(self.poseData, 0, axis=0)
            if self.planPath is not None:                       
                ############################ 旋转点当前位置和参考航向角 ###############################
                world_auto_pose_ = np.array([self.autoPose.x, self.autoPose.y, self.planPath.angle[0]])   
                ############################ 画出历史路径 ###############################
                history_auto_pose_ = world_to_canvas(world_auto_pose_, canvas_auto_pose, self.poseData, scale)
                for i in range(0, np.size(history_auto_pose_, 0)):
                    canvas_point_pose_ = (history_auto_pose_[i,0], history_auto_pose_[i,1])
                    cv2.circle(canvas, canvas_point_pose_, 1, (0,0,0), -1)     #黑色
                if  self.globalPath is not None:
                ############################ 画出全局参考路径 ###############################
                    self.canvas_global_pose = world_to_canvas(world_auto_pose_, canvas_auto_pose, self.world_global_pose, scale)
                    for i in range(0, np.size(self.canvas_global_pose, 0)):
                        canvas_point_pose_ = (self.canvas_global_pose[i,0], self.canvas_global_pose[i,1])
                        cv2.circle(canvas, canvas_point_pose_, 1, (0,0,255), -1) # 红色
                else:
                    self.get_logger().warn("检查全局规划节点")
                ############################ 画出规划参考路径 ###############################
                if self.planPath is not None:
                    world_path_pose_ = np.concatenate([np.array(self.planPath.x).reshape(-1,1), np.array(self.planPath.y).reshape(-1,1)], axis=1)
                    canvas_path_pose_ = world_to_canvas(world_auto_pose_, canvas_auto_pose, world_path_pose_, scale)
                    for i in range(0, np.size(canvas_path_pose_, 0)):
                        canvas_point_pose_ = (canvas_path_pose_[i,0], canvas_path_pose_[i,1])
                        cv2.circle(canvas, canvas_point_pose_, 1, (0,255,0), -1)
                else:
                    self.get_logger().warn("检查局部规划节点")
                if self.predictionPath is not None:
                    prediction_path_pose_ = np.concatenate([np.array(self.predictionPath.x).reshape(-1,1), np.array(self.predictionPath.y).reshape(-1,1)], axis=1)
                    canvas_path_pose_ = world_to_canvas(world_auto_pose_, canvas_auto_pose, prediction_path_pose_, scale)
                    for i in range(0, np.size(canvas_path_pose_, 0)):
                        canvas_point_pose_ = (canvas_path_pose_[i,0], canvas_path_pose_[i,1])
                        cv2.circle(canvas, canvas_point_pose_, 1, (255,0,0), -1)
                else:
                    self.get_logger().warn("检查MPC控制节点")                   
            else:
                self.get_logger().warn("检查局部规划节点")        
        else:
            self.get_logger().warn("检查定位节点")
        self.get_logger().info("Process time {}".format(time.time() - start_time))
        cv2AddText(canvas, self.text_name, self.text_pose_x, self.text_pose_y, self.text_data, font)
        cv2.imshow("hmi", canvas)        
        cv2.waitKey(100)

def main():                                           # ROS2节点主入口main函数
    rclpy.init()                                      # ROS2 Python接口初始化
    rosNode = Hmi(name=ros_node_name)                 # 创建ROS2节点对象并进行初始化
    rclpy.spin(rosNode)                               # 循环等待ROS2退出
    rclpy.shutdown()                                  # 关闭ROS2 Python接口
