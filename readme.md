## <center> Simulation
基于**ROS2**搭建**自动驾驶仿真平台**，所用接口在实际车辆接口上进行改动。主要实现功能：
- 可视化显示（hmi节点）：包含**车辆位置**，**参考线**以及**局部规划轨迹/路径**，采用cv2库实现。
- 自动驾驶车辆仿真模型（sim_model节点）：提供**方向盘、油门和刹车**接口，期望转角和实际转角建立为**一阶惯性环节**，纵向油门、刹车和加速度之间的关系根据**标定表**进行线性插值拟合。
- 规划模块：根据参考线进行**局部轨迹/路径规划**，包含**参考线优化模块（eigen库）**，**匹配点和投影点计算**。
- 控制模块：包含**横纵向PID路径和速度跟踪模块**，**横向MPC路径跟踪模块**，**纵向MPC模块**，**MPC轨迹跟踪模块**。
### 1. 功能包介绍
- `car_interfaces`：定义的msg包，定义各种话题数据。
- `hmi`：可视化包，用于显示车辆当前位置，参考线和参考路径/轨迹。
- `pid`：pid路径跟踪控制器，包含横向路径跟踪和纵向速度跟踪控制器。
- `planner`：规划器，包含参考线优化和局部路径/轨迹规划。
- `sim_model`：仿真模型，提供**方向盘、油门和刹车**接口，期望转角和实际转角建立为**一阶惯性环节**，纵向油门、刹车和加速度之间的关系根据**标定表**进行线性插值拟合。
- `start_simulation`：启动节点，包含各种配置信息。
- `lateral_mpc`：横向MPC路径跟踪控制器，纵向采用PID跟踪速度。
- `longitudinal_mpc`：上层采用MPC控制器计算期望加速度，下层采用复合控制器跟踪期望加速度。
- `trajectory_mpc`：MPC轨迹跟踪控制器，MPC控制器得到前轮偏角和速度控制量，速度控制采用PID控制器。
### 2. 节点启动
- 仿真节点启动
```
ros2 launch start_simulation start_simulation.launch.py
```
文件`src/basic/start_simulation/config/simulation.yaml`用于仿真节点`sim_model`和规划器`planner`参数配置。
- 控制程序启动（任选一个）
```
ros2 launch pid pid.launch.py    // 启动PID控制节点
ros2 launch lateral_mpc lateral_mpc.launch.py // 启动横向MPC控制节点
ros2 launch longitudinal_mpc longitudinal_mpc.launch.py  // 启动纵向MPC控制节点
ros2 launch trajectory_mpc trajectory_mpc.launch.py // 启动MPC轨迹跟踪节点
```