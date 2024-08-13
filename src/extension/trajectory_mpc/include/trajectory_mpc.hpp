/**
 * @file lateral_mpc.hpp
 * @brief Defines the MPCController class.
*/

#include <rclcpp/rclcpp.hpp>

#include <cstdio>

#include <cmath>

#include <yaml-cpp/yaml.h>

#include <eigen3/Eigen/Core>

#include <cppad/cppad.hpp>

#include <cppad/ipopt/solve.hpp>

#include <chrono>

#include <fstream>

#include "car_interfaces/msg/gps_interface.hpp"

#include "car_interfaces/msg/car_ori_interface.hpp"

#include "car_interfaces/msg/local_path_planning_interface.hpp"

#include "car_interfaces/msg/pid_interface.hpp"

#include "car_interfaces/msg/local_prediction_interface.hpp"

using namespace std;

using namespace car_interfaces::msg;

using CppAD::AD;



typedef CPPAD_TESTVECTOR(double) Dvector;

struct CarState {
    double x_now;
    double y_now;
    double yaw_now;
    double delta_now;
    double pitch_now;
    double lon_speed_now;
    double lat_speed_now;
    double lon_acc_now;
    double lat_acc_now;
};

struct MpcParameters {
    size_t prediction_horizon;
    int preview_point;
    double dt;
    double stage_cost_error_x;
    double stage_cost_error_y;
    double stage_cost_error_v;
    double stage_cost_error_w;
    double terminal_cost_error_x;
    double terminal_cost_error_y;

    double stage_cost_delta;

    double Ks = 3.0;
    double tau_s = 3.0;

    double L = 2.0;

    size_t x_s;
    size_t y_s;
    size_t theta_s;
    size_t delta_s;
    size_t v_s;
    size_t desired_delta_s;

    size_t num_vars;
    size_t num_constraints;
};

struct ErrorState {
    double error_y;
    double error_x;
    double error_yaw;
    double error_speed;
    double error_acc;
};

namespace control{
    /**
     * @class MPCControl
     * 
     * @brief MPCControl, lateral mpc controller
    */
    class MPCController: public rclcpp::Node {

    protected:
        CarState car_state;
        MpcParameters control_param;
        ErrorState error_state;
        YAML::Node pid_param;

        // ROS publishers
        rclcpp::Publisher<PidInterface>::SharedPtr control_pub_;
        rclcpp::Publisher<LocalPredictionInterface>::SharedPtr prediction_pub_;

        // ROS subscriptions
        rclcpp::Subscription<GpsInterface>::SharedPtr gps_data_sub_;
        rclcpp::Subscription<CarOriInterface>::SharedPtr car_ori_sub_;
        rclcpp::Subscription<LocalPathPlanningInterface>::SharedPtr local_plan_sub_;
        // ROS timer 
        rclcpp::TimerBase::SharedPtr spin_timer_;
        GpsInterface::SharedPtr msg_gps_data = NULL;
        CarOriInterface::SharedPtr msg_car_ori = NULL;
        LocalPathPlanningInterface::SharedPtr msg_local_plan = NULL;
        PidInterface pid_control_msg;

        // out file
        ofstream out_file;
    public:
        
        /**
         * @brief constructor
         * @param node_name node name
         * @param loop_hz control frequency
         * @param options node configuration
         */
        explicit MPCController(const string node_name, const double loop_hz, const rclcpp::NodeOptions &options);

        /**
         * @brief Set control parameters
        */
        void ControlParameterSet();

        ofstream CreateOutputFile();

        /**
         * @brief 轨迹跟踪控制器
         * @param pid_control_msg control message
         * @param error_state error state of the car
        */
        void TrajectoryCpontroller(PidInterface& pid_control_msg, const ErrorState& error_state);

        /**
         * @brief longitudinal pid controller
         * @param pid_control_msg control message
         * @param error_state error state of the car
        */
       void LongitudinalPidController(PidInterface& pid_control_msg, const double ref_speed);

        /**
         * @brief construction model
         * @param error_state error state of the car
         * @return optimal control sequence
        */
        vector<vector<double>> GetSolution(CarState& car_state);

        /**
         * @brief calculate lateral and longitudinal error
         * @param msg_local_plan the local plan data
         * @param car_state error state of the car
        */
        ErrorState ComputeErrors(const LocalPathPlanningInterface& msg_local_plan, const CarState& car_state); 

        /**
         * @brief Calculate predicted trajectory
         * @param car_state car state
         * @param control_solutions control sequence
         * @return error state
        */
       void ComputePredictions(const CarState& car_state, const vector<vector<double>>& control_solutions);
        
        /**
         * @brief gps data callback
         * @param msg a pointer of gps data
        */
        void GpsDataCallback(const GpsInterface::SharedPtr msg);

        /**
         * @brief car ori data callback
         * @param msg a pointer of orin data
        */
        void OriDataCallback(const CarOriInterface::SharedPtr msg);
        
        /**
         * @brief local plan data callback
         * @param msg a pointer of local plan data
        */
        void PlanDataCallback(const LocalPathPlanningInterface::SharedPtr msg);

        /**
         * @brief timer callback function
        */
        void TimerCallback();

        /**
         * @brief destructor
        */
        virtual ~MPCController() = default;

    };
    /**
     * @class FG_eval
     * 
     * @brief construct model of model predictive controller
    */
    class FG_eval {
        public:
            /**
             * @brief constructor
             * @param control_param the parameters of mpc
             * @param car_state the state of car
            */
            FG_eval(MpcParameters* control_param, CarState* car_state, LocalPathPlanningInterface::SharedPtr msg_local_plan);

            /**
             * @brief destructor
            */
            ~FG_eval() = default;
            typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

            void operator()(ADvector& fg, const ADvector& vars);
        protected:
            MpcParameters* control_param = NULL;
            CarState* car_state = NULL;
            LocalPathPlanningInterface::SharedPtr msg_local_plan = NULL;
    };
}


