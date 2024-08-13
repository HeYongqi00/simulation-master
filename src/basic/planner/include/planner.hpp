#include <rclcpp/rclcpp.hpp>

#include <cstdio>

#include <vector>

#include <cmath>

#include <unistd.h>

#include <fstream>

#include <OsqpEigen/OsqpEigen.h>

#include <Eigen/Dense>

#include <car_interfaces/msg/global_path_planning_interface.hpp>

#include <car_interfaces/msg/local_path_planning_interface.hpp>

#include <car_interfaces/msg/gps_interface.hpp>

#include "planner_tools.hpp"

using namespace std;

using namespace car_interfaces::msg;

using namespace math;


namespace planning {
    class PathPlanner: public rclcpp::Node {

        protected:
            // ROS timer 
            rclcpp::TimerBase::SharedPtr spin_timer_;

            // ROS publishers
            rclcpp::Publisher<GlobalPathPlanningInterface>::SharedPtr global_path_pub_;
            rclcpp::Publisher<LocalPathPlanningInterface>::SharedPtr local_path_pub_;
            
            // ROS subscribers
            rclcpp::Subscription<GpsInterface>::SharedPtr gps_data_sub_;

            GpsInterface::SharedPtr msg_gps_data = NULL;

            CarState car_state;
            vector<double> smooth_cost_weight;  // 路径平滑代价
            vector<PathPoint> global_path;     //存放全局规划路径
            vector<PathPoint> reference_line;  //存放参考线
            vector<PathPoint> local_path;      //局部路径
            double dt;

            string file_name;
            int plan_horizon;
            double ref_speed;
            int choice;
            int trajectory_plan;

        
        public:
            /**
             * @brief constructor
             * @param node_name node name
             * @param loop_hz control frequency
             * @param options node configuration
             */
            explicit PathPlanner(const string node_name, const double loop_hz, const rclcpp::NodeOptions& options);

            virtual ~PathPlanner() = default;

            /**
             * @brief get parameters from launch file
            */
           void GetParameter();

           /**
            * @brief get the global path from file
            * @param file_name file name
            * @param choice global planning choice
            * @param ref_speed planned speed
            * */
            bool GetPathFromFile(const string file_name, const int choice, const double ref_speed);

            /**
             * @brief 参考线平滑
             * @param reference_line 粗糙参考线
             * @param cost_weight 代价权重 3*1 分别为 与原路经相似代价， 平滑代价， 紧凑代价
             */
            bool SmoothReferenceLine(vector<PathPoint>& reference_line, 
                                     const vector<double>& smooth_cost_weight);


            /**
             * @brief get reference line
             * @param nearest_index index of the nearest point to the global path
             * @return reference line
            */
            vector<PathPoint> GetReferenceLine(const vector<PathPoint>& global_path,
                                                 const CarState& car_state);
            /**
             * @brief 根据速度进行轨迹规划，两点之间距离由纵向速度计算
             * @param reference_line 参考线
             * @param car_state 车辆状态
             * @param plan_horizon 规划时域
             * 
            */
            void TrajectoryPlan(const vector<PathPoint> reference_line,
                                 const CarState car_state,
                                 const int plan_horizon);
            /**
             * @brief publish local path 
             * @param nearest_index index of the nearest point to the reference path
             * @param point_interval planning interval on reference path
             * @param plan_horizon plan horizon
            */
            void LocalPathPlan(const vector<PathPoint> reference_line,
                               const CarState car_state,
                               const int plan_horizon);

            /**
             * @brief 获取局部规划起始点坐标
             * @param local_path 上次的规划的局部路径
             * @param car_state 车辆位置信息
             * @return 规划起始点
            */
            PathPoint GetStartPoint(const vector<PathPoint>& reference_line, 
                                    const vector<PathPoint>& local_path, 
                                    const CarState& car_state);

            /**
             * @brief publish global path
             * @param nearest_index index of the nearest point to the reference path
            */
           void GlobalPathPlan(const vector<PathPoint> reference_line);

           /**
            * @brief get GPS messages
           */
            void GpsDataCallback(const GpsInterface::SharedPtr msg);           
            /**
             * @brief find the nearest point index
             * @param global_path global Path
             * @param car_state the state of the car
             * @param last_index the index of the last nearest point
            */

           /**
            * @brief Find the nearest point on the reference path
            * @param path_points path points
            * @param car_state car state
           */
            int MatchToPath(const vector<PathPoint>& path_points, 
                            const CarState& car_state, 
                            int nearest_index);
        private:
            void TimerCallback();
    };
}