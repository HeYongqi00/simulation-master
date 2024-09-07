/**
 * @file linear_mpc.cpp
 * @brief 线性MPC轨迹跟踪控制
 * @author HeYongqi
 * @date 2023-09-13
 */

#include "linear_mpc.hpp"

namespace control {

    MPCController::MPCController(const string node_name, const double loop_hz, 
    const rclcpp::NodeOptions& options) : Node(node_name, options) {
        control_param.dt = 1 / loop_hz;
        control_pub_ = create_publisher<PidInterface>("pid_data", 1);
        prediction_pub_ = create_publisher<LocalPredictionInterface>("local_prediction_data", 1);
        gps_data_sub_ = create_subscription<GpsInterface>(
            "gps_data", 1, bind(&MPCController::GpsDataCallback, this, placeholders::_1));
        car_ori_sub_ = create_subscription<CarOriInterface>(
            "car_ori_data", 1, bind(&MPCController::OriDataCallback, this, placeholders::_1));
        local_plan_sub_ = create_subscription<LocalPathPlanningInterface>(
            "local_path_planning_data", 1, bind(&MPCController::PlanDataCallback, this, placeholders::_1));
        
        spin_timer_ = create_wall_timer(chrono::milliseconds(int(1000/loop_hz)), bind(&MPCController::TimerCallback, this));
        ControlParameterSet();
        out_file = CreateOutputFile();
    }

    void MPCController::ControlParameterSet() {
        
        char floder_path[100] = {0};
        char file_name[256] = {0};
        getcwd(floder_path, sizeof(floder_path));
        sprintf(file_name, "%s%s", floder_path, "/src/extension/trajectory_mpc/config/config.yaml");
        pid_param =  YAML::LoadFile(file_name);
        
        // 声明参数
        this->declare_parameter<double>("stage_cost_error_x", 0.4);
        this->declare_parameter<double>("stage_cost_error_y", 0.4);
        this->declare_parameter<double>("stage_cost_error_v", 0.1);
        this->declare_parameter<double>("stage_cost_error_w", 0.1);
        this->declare_parameter<double>("terminal_cost_error_x", 0.5);
        this->declare_parameter<double>("terminal_cost_error_y", 0.5);
        this->declare_parameter<int>("prediction_horizon", 10);
        this->declare_parameter<int>("preview_point", 0);

        this->declare_parameter<double>("Ks", 1.0);
        this->declare_parameter<double>("tau_s", 0.9);
        this->declare_parameter<double>("L", 2.0);

        this->declare_parameter<double>("stage_cost_delta", 0.2);


        control_param.stage_cost_error_x =  this->get_parameter("stage_cost_error_x").as_double();
        control_param.stage_cost_error_y = this->get_parameter("stage_cost_error_y").as_double();
        control_param.stage_cost_error_v = this->get_parameter("stage_cost_error_v").as_double();
        control_param.stage_cost_error_w = this->get_parameter("stage_cost_error_w").as_double();
        control_param.terminal_cost_error_x = this->get_parameter("terminal_cost_error_x").as_double();
        control_param.terminal_cost_error_y = this->get_parameter("terminal_cost_error_y").as_double();
        control_param.prediction_horizon = this->get_parameter("prediction_horizon").as_int();
        control_param.preview_point = this->get_parameter("preview_point").as_int();

        control_param.Ks = this->get_parameter("Ks").as_double();
        control_param.tau_s = this->get_parameter("tau_s").as_double();
        control_param.L = this->get_parameter("L").as_double();

        control_param.stage_cost_delta = this->get_parameter("stage_cost_delta").as_double();
        

        control_param.x_s = 0;
        control_param.y_s = control_param.x_s + control_param.prediction_horizon + 1;
        control_param.theta_s = control_param.y_s + control_param.prediction_horizon + 1;
        control_param.delta_s = control_param.theta_s + control_param.prediction_horizon + 1;

        control_param.v_s = control_param.delta_s + control_param.prediction_horizon + 1;
        control_param.desired_delta_s = control_param.v_s + control_param.prediction_horizon;

        control_param.num_vars = 4 * (control_param.prediction_horizon + 1) + 2 * control_param.prediction_horizon;   // 变量数量
        control_param.num_constraints = (control_param.prediction_horizon + 1) * 4;
    }

    ofstream MPCController::CreateOutputFile() {
        
        char floder_path[100] = {0};
        char file_name[256] = {0};
        char time_name[100] = {0};
        time_t now_time = time(NULL);
        struct tm *ptr_time;
        ofstream out_file;
        ptr_time = gmtime(&now_time);
        sprintf(time_name, "%d_%.2d_%.2d_%.2d_%.2d_%.2d.txt", 1900+ptr_time->tm_year, 1+ptr_time->tm_mon, ptr_time->tm_mday, 
                8+ptr_time->tm_hour, ptr_time->tm_min, ptr_time->tm_sec);
        getcwd(floder_path, sizeof(floder_path));
        sprintf(file_name, "%s%s%s", floder_path, "/data/results/", time_name);
        out_file.open(file_name);
        return out_file;
    }

    void MPCController::TrajectoryCpontroller(PidInterface& pid_control_msg, const ErrorState& error_state) {
        
        auto start = std::chrono::high_resolution_clock::now();
        vector<vector<double>> control_solutions;
        control_solutions = GetSolution(car_state);
        ComputePredictions(car_state, control_solutions);
        // double L = 1.923;
        cout << "delta = " << car_state.delta_now << endl;
        cout << "error_x = " << error_state.error_x << endl;
        cout << "error_y = " << error_state.error_y << endl;
        cout << "error_yaw = " << error_state.error_yaw << endl;
        cout << "desired_speed = " << control_solutions[0][0] << endl;
        cout << "expected_angle = " << control_solutions[0][1] << endl;
        // pid_control_msg.angle =  atan(control_solutions.at(0) * L / (2.0 * 30)) + 0.028;
        // pid_control_msg.angle =  atan(control_solutions[0][1]) + 0.028;
        double steer_value = control_solutions[0][1] + 0.028;
        if(steer_value > 1)
            steer_value = 1.0;
        if(steer_value < -1)
            steer_value = -1.0;
        pid_control_msg.angle = steer_value;
        // pid_control_msg.angle = control_solutions.at(0) + 0.028;
        // pid_control_msg.angle = control_solutions.at(0);

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;
        double process_time = duration.count();

        cout << "process_time = " << process_time << endl;


        out_file << msg_local_plan->x[control_param.preview_point] << "\t" 
                 << msg_local_plan->y[control_param.preview_point] << "\t"
                 << msg_local_plan->angle[control_param.preview_point] << "\t"
                 << car_state.x_now << "\t" << car_state.y_now << "\t"
                 << car_state.yaw_now << "\t" << error_state.error_yaw << "\t"
                 << error_state.error_x << "\t" << error_state.error_y << "\t"
                 << pid_control_msg.angle << "\t" << process_time << "\t" 
                 << car_state.delta_now << "\t" << control_solutions[1][1] << "\n";
        double ref_speed = control_solutions[0][0];
        // double ref_speed = 0.5;

        LongitudinalPidController(pid_control_msg, ref_speed);
        
    }

    void MPCController::LongitudinalPidController(PidInterface& pid_control_msg, const double ref_speed) {
        
        auto start = std::chrono::high_resolution_clock::now();
        static double bracking_distance = 0;
        static double error_speed_sum = 0;
        
        // ref_speed = msg_local_plan->speed[control_param.preview_point];  // 
        double ref_acc = 0;
        double ref_throttle, ref_braking;
        double acc_kp, acc_ki;
        // double error_speed = error_state.error_speed;
        double error_speed = ref_speed - car_state.lon_speed_now;
        
        // 档位
        if (ref_speed < 0) {
            pid_control_msg.gear = 4;
        }
        else if (ref_speed >= 0) {
            pid_control_msg.gear = 3;
        }
        // 停车开环控制
        if (ref_speed < 0.05) {  
            if (car_state.lon_speed_now < 0.1) {
                ref_throttle = 0;
                ref_braking = 30;
            }
            else {
                bracking_distance += car_state.lon_speed_now * control_param.dt;
                ref_throttle = 0;
                ref_braking = pid_param["longitudinal_dec_open_loop"].as<double>() * bracking_distance 
                              + pid_param["longitudinal_dec_open_loop_start"].as<double>();
            }
        }
        // 加减速PID
        else {
            bracking_distance = 0;
            if (error_speed < -0.5) {  // 减速 P控制
                ref_braking = error_speed * pid_param["longitudinal_dec_kp"].as<double>();
                ref_throttle = 0;
            }
            else {                                 // 加速 PI控制，积分分离
                if (abs(error_speed) < 1.0)
                    error_speed_sum += error_speed;
                else
                    error_speed_sum = 0;
                
                if (car_state.pitch_now > 2) {
                    acc_kp = pid_param["longitudinal_acc_slope_kp"].as<double>();
                    acc_ki = pid_param["longitudinal_acc_slope_ki"].as<double>();
                }
                else {
                    acc_kp = pid_param["longitudinal_acc_plain_kp"].as<double>();;
                    acc_ki = pid_param["longitudinal_acc_plain_ki"].as<double>();
                }
                ref_throttle = 100 * (acc_kp * error_speed + acc_ki * error_speed_sum);
                ref_braking = 0;
                if (ref_throttle < 0) {
                    ref_braking = abs(ref_throttle);
                    ref_throttle = 0;
                }
                    
            }
        }--
        ref_throttle = max(0.0, min(ref_throttle, 100.0));
        ref_braking = max(0.0, min(ref_braking, 125.0));

        pid_control_msg.throttle_percentage = (int)(ref_throttle);
        pid_control_msg.braking_percentage = (int)(ref_braking);

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;
        double process_time = duration.count();

        out_file << ref_speed << "\t" << car_state.lon_speed_now << "\t"; // 1-2
        out_file << ref_acc << "\t" << car_state.lat_acc_now << "\t";     // 3-4
        out_file << ref_braking << "\t" << ref_braking << "\t";           // 5-6
        out_file << process_time << "\t";                                 // 7
        out_file << 0 << "\t" << 0 << "\t" << 0 << "\t";                  // 8-10
    }

    vector<vector<double>> MPCController::GetSolution(CarState& car_state) {
        Eigen::VectorXd a(3);  // 创建大小为3的动态向量
        a << 1, 2, 3;          // 初始化向量的元素

        // 使用()运算符访问向量的元素
        std::cout << a(0) << " " << a(1) << " " << a(2) << std::endl;
        
        Dvector vars(control_param.num_vars);
        // 变量初始化
        for (size_t i = 0; i < control_param.num_vars; i++) {
            vars[i] = 0;
        }

        // 初始状态约束
        vars[control_param.x_s] = car_state.x_now;
        vars[control_param.y_s] = car_state.y_now;
        vars[control_param.theta_s] = car_state.yaw_now;
        // vars[control_param.delta_s] = car_state.delta_now;

        // 变量上下界
        Dvector vars_lowerbound(control_param.num_vars), vars_upperbound(control_param.num_vars);
        for (size_t i = 0; i < control_param.num_vars; i++) {
            vars_lowerbound[i] = -1000;
            vars_upperbound[i] = 1000;
        }
        for (size_t i = control_param.v_s; i < control_param.num_vars; i++) {
            vars_lowerbound[i] = -3;
            vars_upperbound[i] = 3;
        }
        for (size_t i = control_param.desired_delta_s; i < control_param.num_vars; i++) {
            vars_lowerbound[i] = -1;
            vars_upperbound[i] = 1;
        }
        
        // 等式约束
        Dvector constraints_lowerbound(control_param.num_constraints);
        Dvector constraints_upperbound(control_param.num_constraints);
        for (size_t i = 0; i < control_param.num_constraints; i++) {
            constraints_lowerbound[i] = 0;
            constraints_upperbound[i] = 0;
        }
        constraints_lowerbound[control_param.theta_s] = car_state.yaw_now;
        constraints_lowerbound[control_param.delta_s] = car_state.delta_now;
        constraints_lowerbound[control_param.x_s] = car_state.x_now;
        constraints_lowerbound[control_param.y_s] = car_state.y_now;


        constraints_upperbound[control_param.x_s] = car_state.x_now;
        constraints_upperbound[control_param.y_s] = car_state.y_now;
        constraints_upperbound[control_param.theta_s] = car_state.yaw_now;
        constraints_upperbound[control_param.delta_s] = car_state.delta_now;
        
        // 求解器设置
        string options;
        options += "Integer print_level  0\n";
        options += "Sparse  true        forward\n";
        options += "Sparse  true        reverse\n";
        // options += "Integer max_iter     100\n";
        // options += "max_cpu_time    0.05\n";
        CppAD::ipopt::solve_result<Dvector> solution;
        
        FG_eval fg_eval(&control_param, &car_state, msg_local_plan);

        CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound, 
                                                constraints_upperbound, fg_eval, solution);
        bool ok = true;
	    if(ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success)
            cout << "solve success!" << endl;
        else
            cout << "solve failed!" << "solution.status = " << solution.status << endl;
        
        vector<vector<double>> control_solution(control_param.prediction_horizon, vector<double>(2,0));
        for (size_t i = 0; i < control_param.prediction_horizon; i++) {
            control_solution[i][0] = solution.x[control_param.v_s + i];
            control_solution[i][1] = solution.x[control_param.desired_delta_s + i];
        }
        // for (size_t i = 0; i < 2*control_param.prediction_horizon; i++)
        //     control_solution.push_back(solution.x[control_param.v_s + i]);
        return control_solution;
    }

    ErrorState MPCController::ComputeErrors(const LocalPathPlanningInterface& msg_local_plan, const CarState& car_state) {
        double ref_x, ref_y, ref_yaw, ref_speed;
        double now_x, now_y, now_yaw;
        ErrorState error_state;

        // ref_x = msg_local_plan.x.at(0);
        // ref_y = msg_local_plan.y[control_param.preview_point];
        // ref_yaw = msg_local_plan.angle[control_param.preview_point];
        // ref_speed = msg_local_plan.speed[control_param.preview_point];
        ref_x = msg_local_plan.x.at(0);
        ref_y = msg_local_plan.y.at(0);
        ref_yaw = msg_local_plan.angle.at(0);
        ref_speed = msg_local_plan.speed.at(0);

        now_yaw = car_state.yaw_now;
        now_x = car_state.x_now;
        now_y = car_state.y_now;
        

        // const double dx = now_x - ref_x;
        // const double dy = now_y - ref_y;
        // const double cross_rd_nd = cos(ref_yaw) * dy - sin(ref_yaw) * dx;
        
        error_state.error_yaw = ref_yaw - now_yaw;
        if (error_state.error_yaw > M_PI)
            error_state.error_yaw -= 2 * M_PI;
        else if (error_state.error_yaw < -M_PI)
            error_state.error_yaw += 2 * M_PI;

        error_state.error_x = cos(now_yaw) * (ref_x - now_x) + sin(now_yaw) * (ref_y - now_y);
        
        error_state.error_y =  - sin(now_yaw) * (ref_x - now_x) + cos(now_yaw) * (ref_y - now_y);
        // error_state.error_y = std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd); // apollo 计算横向误差
        
        error_state.error_speed = ref_speed - car_state.lon_speed_now;

        return error_state;
    }

    void MPCController::ComputePredictions (const CarState& car_state, const vector<vector<double>>& control_solutions) {
        LocalPredictionInterface local_prediction_;
        local_prediction_.error_x = error_state.error_x;
        local_prediction_.error_y = error_state.error_y;
        local_prediction_.error_yaw = error_state.error_yaw;
        local_prediction_.error_speed = error_state.error_speed;

        local_prediction_.x.push_back(car_state.x_now);
        local_prediction_.y.push_back(car_state.y_now);
        local_prediction_.yaw.push_back(car_state.yaw_now);
        double x_next_ = car_state.x_now;
        double y_next_ = car_state.y_now;
        double yaw_next_ = car_state.yaw_now;
        // double speed_now_ = car_state.lon_speed_now;
        double ref_speed = msg_local_plan->speed[0];

        for (size_t i = 0; i < control_solutions.size(); i++) {
            x_next_ += control_param.dt * ref_speed * cos(yaw_next_);
            y_next_ += control_param.dt * ref_speed * sin(yaw_next_);
            yaw_next_ += control_param.dt * control_solutions[i][1];

            local_prediction_.x.push_back(x_next_);
            local_prediction_.y.push_back(y_next_);
            local_prediction_.yaw.push_back(yaw_next_);
        }
        prediction_pub_->publish(local_prediction_);
    }
    void MPCController::GpsDataCallback(const GpsInterface::SharedPtr msg) {
        msg_gps_data = msg;
        car_state.x_now = msg_gps_data->x;
        car_state.y_now = msg_gps_data->y;
        car_state.yaw_now = msg_gps_data->yaw/180*M_PI;     // From angle to radian
        car_state.pitch_now = msg_gps_data->pitch;
        car_state.lon_speed_now = pow(pow(msg_gps_data->northvelocity, 2) + pow(msg_gps_data->eastvelocity, 2) ,0.5);
    }

    void MPCController::OriDataCallback(const CarOriInterface::SharedPtr msg) {
        msg_car_ori = msg;
        car_state.delta_now = (msg->steerangle / 550.0); // 方向盘转角
    }

    void MPCController::PlanDataCallback(const LocalPathPlanningInterface::SharedPtr msg) {
        msg_local_plan = msg;
    }

    void MPCController::TimerCallback() {
        if (!msg_gps_data)
            RCLCPP_WARN(get_logger(),"Check the gps node!");
        else if (!msg_local_plan)
            RCLCPP_WARN(get_logger(),"Check the local plan node!");
        else if (!msg_car_ori)
            RCLCPP_WARN(get_logger(),"Check the car orin node!");
        else {
            error_state =  ComputeErrors(*msg_local_plan, car_state);

            TrajectoryCpontroller(pid_control_msg, error_state);
  
            RCLCPP_INFO(get_logger(),"longitudinal controller: throttle = %d\tbrake = %d", 
                                pid_control_msg.throttle_percentage, pid_control_msg.braking_percentage);
            RCLCPP_INFO(get_logger(),"lateral controller: angle = %f", pid_control_msg.angle);
            control_pub_->publish(pid_control_msg);
        }
    }

    FG_eval::FG_eval(MpcParameters* control_param , CarState* car_state, LocalPathPlanningInterface::SharedPtr msg_local_plan) {
        this->control_param = control_param;
        this->car_state = car_state;
        this->msg_local_plan = msg_local_plan;
    }

    void FG_eval::operator()(ADvector& fg, const ADvector& vars) {
        
        // The cost is stored is the first element of `fg`.
        fg[0] = 0;
        // Stage cost
        for (size_t i = 0; i < control_param->prediction_horizon; i++)
        {
            AD<double> theta_now = vars[control_param->theta_s + i];
            AD<double> delta_now = vars[control_param->delta_s + i];
            AD<double> v_now = vars[control_param->v_s + i];
            AD<double> x_now = vars[control_param->x_s + i];
            AD<double> y_now = vars[control_param->y_s + i];


            double x_r = msg_local_plan->x.at(i);
            double y_r = msg_local_plan->y.at(i + control_param->preview_point);
            double theta_r = msg_local_plan->angle.at(i + control_param->preview_point);
            double v_r = msg_local_plan->speed.at(i + control_param->preview_point);

            AD<double> delta_desiged = vars[control_param->desired_delta_s + i];

            AD<double> e_x = CppAD::cos(theta_now) * (x_r - x_now) + CppAD::sin(theta_now) * (y_r - y_now);
            AD<double> e_y = -CppAD::sin(theta_now) * (x_r - x_now) + CppAD::cos(theta_now) * (y_r - y_now);
            AD<double> e_theta = theta_r - theta_now;
            AD<double> v_e = -v_now + v_r * CppAD::cos(e_theta);
            // AD<double> w_e = -v_now * CppAD::tan(delta_desiged) + v_r * CppAD::sin(e_theta);  // 期望转角
            // AD<double> w_e = -v_now * CppAD::tan(delta_now) + v_r * CppAD::sin(e_theta);     // 实际转角
            AD<double> w_e = v_r * CppAD::sin(e_theta);     // 实际转角

            // fg[0] += 5.0 * CppAD::pow(e_theta, 2);

            fg[0] += control_param->stage_cost_error_x * CppAD::pow(e_x, 2);
            fg[0] += control_param->stage_cost_error_y * CppAD::pow(e_y, 2);
            fg[0] += control_param->stage_cost_error_v * CppAD::pow(v_e, 2);
            fg[0] += control_param->stage_cost_error_w * CppAD::pow(w_e, 2);

            
        }
        // 输入 期望转角代价
        for (size_t i = 0; i < control_param->prediction_horizon; i++)
            fg[0] += control_param->stage_cost_delta * CppAD::pow(vars[control_param->desired_delta_s + i], 2);
        
        // Terminal cost
        AD<double> theta_now = vars[control_param->theta_s + control_param->prediction_horizon];
        AD<double> x_now = vars[control_param->x_s + control_param->prediction_horizon];
        AD<double> y_now = vars[control_param->y_s + control_param->prediction_horizon];
        

        double x_r = msg_local_plan->x.at(control_param->prediction_horizon);
        double y_r = msg_local_plan->y.at(control_param->prediction_horizon + control_param->preview_point);

        AD<double> e_x = CppAD::cos(theta_now) * (x_r - x_now) + CppAD::sin(theta_now) * (y_r - y_now);
        AD<double> e_y = -CppAD::sin(theta_now) * (x_r - x_now) + CppAD::cos(theta_now) * (y_r - y_now);

        fg[0] += control_param->terminal_cost_error_x * CppAD::pow(e_x, 2);
        fg[0] += control_param->terminal_cost_error_y * CppAD::pow(e_y, 2);

        


        // Setup Constraints
        fg[1 + control_param->x_s] = vars[control_param->x_s];
        fg[1 + control_param->y_s] = vars[control_param->y_s];
        fg[1 + control_param->theta_s] = vars[control_param->theta_s];
        fg[1 + control_param->delta_s] = vars[control_param->delta_s];


        for (size_t i = 0; i < control_param->prediction_horizon; i++) 
        {
            // The state at time k + 1.
            AD<double> x_next_ = vars[control_param->x_s + i + 1];
            AD<double> y_next_ = vars[control_param->y_s + i + 1];
            AD<double> theta_next_ = vars[control_param->theta_s + i + 1];
            AD<double> delta_next_ = vars[control_param->delta_s + i + 1];

            // The state at time k.
            AD<double> x_now_ = vars[control_param->x_s + i];
            AD<double> y_now_ = vars[control_param->y_s + i];
            AD<double> theta_now_ = vars[control_param->theta_s + i];
            AD<double> delta_now_ = vars[control_param->delta_s + i];

            AD<double> v_now_ = vars[control_param->v_s + i];
            AD<double> desired_delta_s = vars[control_param->desired_delta_s + i];



            // fg[2 + control_param->x_s + i] = control_param->dt * (v_now_ * CppAD::cos(theta_now_) - v_now_ * CppAD::sin(theta_now_) * CppAD::tan(delta_now_)) 
            //                                     + x_now_ - x_next_;

            // fg[2 + control_param->y_s + i] = control_param->dt * (v_now_ * CppAD::sin(theta_now_) + v_now_ * CppAD::cos(theta_now_) * CppAD::tan(delta_now_)) 
            //                                     + y_now_ - y_next_;

            fg[2 + control_param->x_s + i] = control_param->dt * (v_now_ * CppAD::cos(theta_now_)) 
                                                + x_now_ - x_next_;

            fg[2 + control_param->y_s + i] = control_param->dt * (v_now_ * CppAD::sin(theta_now_)) 
                                                + y_now_ - y_next_;
            
            fg[2 + control_param->theta_s + i] = control_param->dt * (delta_now_)
                                                    + theta_now_ - theta_next_;

            fg[2 + control_param->delta_s + i] = control_param->dt * (-delta_now_ / control_param->tau_s + control_param->Ks * desired_delta_s / control_param->tau_s) 
                                                    + delta_now_ - delta_next_;
            // fg[2 + control_param->delta_s + i] = desired_delta_s - delta_now_;

            // fg[2 + control_param->error_yaw_start + i] = control_param->dt * (-omega_now_ - 2.0 * msg_local_plan->kappa[i]) 
            //                                                             + error_yaw_now_ - error_yaw_next_;
        }
    }
}