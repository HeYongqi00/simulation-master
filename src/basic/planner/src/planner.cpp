#include "planner.hpp"

namespace planning {
    PathPlanner::PathPlanner(const string node_name, const double loop_hz, 
    const rclcpp::NodeOptions &options) : Node(node_name, options) {
        dt = 1 / loop_hz;
        spin_timer_ = create_wall_timer(chrono::milliseconds(int(1000/loop_hz)), bind(&PathPlanner::TimerCallback, this));
        global_path_pub_ = create_publisher<GlobalPathPlanningInterface>("global_path_planning_data", 1);
        local_path_pub_ = create_publisher<LocalPathPlanningInterface>("local_path_planning_data", 1);
        gps_data_sub_ = create_subscription<GpsInterface>(
            "gps_data", 1, bind(&PathPlanner::GpsDataCallback, this, placeholders::_1));
        
        GetParameter();

        if (GetPathFromFile(file_name, choice, ref_speed))
            RCLCPP_INFO(get_logger(), file_name + " %s", "open success !");
        else
            RCLCPP_ERROR(get_logger(), file_name + " %s", "open failed !");
        RCLCPP_INFO(get_logger(), "ref_speed = %f", ref_speed);
        
    }

    void PathPlanner::GetParameter() {
        declare_parameter("file_name", "lane_change.txt");
        get_parameter("file_name", file_name);
        declare_parameter("plan_horizon", 20);
        get_parameter("plan_horizon", plan_horizon);
        declare_parameter("ref_speed", 1.5);
        get_parameter("ref_speed", ref_speed);
        declare_parameter("choice", 0);
        get_parameter("choice", choice);
        declare_parameter("trajectory_plan", 0);
        get_parameter("trajectory_plan", trajectory_plan);

        smooth_cost_weight = {1.0, 2.0, 0.5};
        car_state.lon_speed_now = 0.0;
    }


    bool PathPlanner::GetPathFromFile(const string file_name, const int choice, const double ref_speed) {
        char char_file_name[100];
        char floder_path[100] = {0};
        char file_path_name[256] = {0};
        ifstream in_file;
        double x, y, yaw, s = 0, lat, lon, speed;
        sprintf(char_file_name, "%s", file_name.c_str()); // string to char
        getcwd(floder_path, sizeof(floder_path));
        sprintf(file_path_name, "%s%s%s", floder_path, "/data/maps/", char_file_name);
        in_file.open(file_path_name);
        if (!in_file.is_open())
            return false;
        else {
            while (!in_file.eof()) {
                in_file >> x >> y >> yaw >> lat >> lon >> speed;
            
                if (choice == 0) //速度规划为匀速
                    speed = ref_speed;
        
                PathPoint pose = {x, y, yaw, s, speed};
                if (!global_path.empty()) {
                    double dx = global_path.back().x - pose.x;
                    double dy = global_path.back().y - pose.y;

                    s += sqrt(dx * dx + dy * dy);
                    pose.s = s;

                    global_path.push_back(pose);
                }
                else {
                    global_path.push_back(pose);
                }
            }
            global_path.back().speed = 0;
            in_file.close();
            return true;
        }
    }

    void PathPlanner::TimerCallback() {
        
        if (!msg_gps_data)
            RCLCPP_WARN(get_logger(),"Check the gps node!");
        else {
            auto start = std::chrono::high_resolution_clock::now();
            reference_line = GetReferenceLine(global_path, car_state); //获取参考线
            
            GlobalPathPlan(reference_line);
            
            // if (!SmoothReferenceLine(reference_line, smooth_cost_weight))  // 参考线平滑
            //     RCLCPP_ERROR(get_logger(),"Check the smooth reference line functinon!");
            if(trajectory_plan)
                TrajectoryPlan(reference_line, car_state, plan_horizon);  // 轨迹规划
            else
                LocalPathPlan(reference_line, car_state, plan_horizon);  // 局部路径规划
             

            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> duration = end - start;
            double process_time = duration.count();
            RCLCPP_INFO(get_logger(), "Planning time = %f", process_time);
        }
    }

    bool PathPlanner::SmoothReferenceLine(vector<PathPoint>& reference_line, 
                                          const vector<double>& smooth_cost_weight) {
        size_t n = reference_line.size();
        size_t i, j;
        Eigen::SparseMatrix<double>A1(2*n, 2*n - 4);
        Eigen::SparseMatrix<double>A2(2*n, 2*n - 2);
        Eigen::SparseMatrix<double>A3(2*n, 2*n);
        
        Eigen::SparseMatrix<double> hessian(2*n, 2*n);  // H矩阵
        Eigen::VectorXd gradient(2*n);                    // f向量
        Eigen::SparseMatrix<double> linearMatrix(2*n, 2*n); //  A: m*n矩阵,必须为稀疏矩阵SparseMatrix
        Eigen::VectorXd lowerBound = Eigen::VectorXd::Zero(2*n);                  //L: m*1下限向量
        Eigen::VectorXd upperBound = Eigen::VectorXd::Zero(2*n);                  //U: m*1上限向量

        for (j = 0; j < 2*n - 4; j++) {   //列索引
            A1.insert(j,j) = 1;
            A1.insert(j+2,j) = -2;
            A1.insert(j+4,j) = 1;
        }
        
        for (j = 0; j < 2*n - 2; j++) {
            A2.insert(j,j) = 1;
            A2.insert(j+2,j) = -1;
        }
        for (j = 0; j < 2*n; j++) {
            A3.insert(j,j) = 1;
        }
        for (i = 0; i < 2*n; i++) {
            if (i%2 == 0)
                gradient(i) = -2 * reference_line.at(i/2).x;
            else
                gradient(i) = -2 * reference_line.at(i/2).y;
        }

        hessian = 2 * (smooth_cost_weight.at(0) * A1 * A1.transpose() 
                        + smooth_cost_weight.at(1) * A2 * A2.transpose() 
                        + smooth_cost_weight.at(2) * A3 * A3.transpose());
        gradient = smooth_cost_weight.at(2) * gradient;

        // instantiate the solver
        OsqpEigen::Solver solver;
 
        // settings
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);

        solver.data()->setNumberOfVariables(2*n);   //变量数2*n
        solver.data()->setNumberOfConstraints(2*n); //约束数2*n


        if (!solver.data()->setHessianMatrix(hessian))
            return false;
        if (!solver.data()->setGradient(gradient))
            return false;
        if (!solver.data()->setLinearConstraintsMatrix(linearMatrix))
            return false;
        if (!solver.data()->setLowerBound(lowerBound))
            return false;
        if (!solver.data()->setUpperBound(upperBound))
            return false;

        // instantiate the solver
        if (!solver.initSolver())
            return false;
        
        Eigen::VectorXd smooth_reference_line;


        // solve the QP problem
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
            return false;
    
        smooth_reference_line = solver.getSolution();
        for (i = 0; i < 2*n; i++) {
            if (i%2 == 0)
                reference_line.at(i/2).x = smooth_reference_line(i);
            else
                reference_line.at(i/2).y = smooth_reference_line(i);
        }
        
        double x_last = reference_line.at(0).x;
        double y_last = reference_line.at(0).y;
        double s = reference_line.at(0).s;
        for (i = 0; i < n; i++) {
            double dx = reference_line.at(i).x - x_last;
            double dy = reference_line.at(i).y - y_last;
            s += sqrt(dx * dx + dy * dy);
            reference_line.at(i).s = s;
            x_last = reference_line.at(i).x;
            y_last = reference_line.at(i).y;
        }
        return true;

    }
    void PathPlanner::GlobalPathPlan(const vector<PathPoint> reference_line) {
        GlobalPathPlanningInterface global_path_msg;
        for (size_t i = 0; i < reference_line.size(); i++) {
            global_path_msg.routedata.push_back(reference_line.at(i).x);
            global_path_msg.routedata.push_back(reference_line.at(i).y);
        }
        global_path_pub_->publish(global_path_msg);
    }

    vector<PathPoint> PathPlanner::GetReferenceLine(const vector<PathPoint>& global_path,
                                                      const CarState& car_state) {
        static int nearest_index = -1;
        int start_index, end_index;
        nearest_index = MatchToPath(global_path, car_state, nearest_index);  // 在参考线上找到匹配点下标
        start_index = max(nearest_index - 40, int(0));
        end_index = min(nearest_index + 70, int(global_path.size() - 1));
        vector<PathPoint> reference_line;
        reference_line.assign(global_path.begin() + start_index, global_path.begin() + end_index);
        return reference_line;
    }

    PathPoint PathPlanner::GetStartPoint(const vector<PathPoint>& reference_line, 
                                         const vector<PathPoint>& local_path, 
                                         const CarState& car_state) {
        PathPoint start_point;
        int nearest_index = -1;
        if (local_path.empty()) {  // 局部路径为空
            nearest_index = MatchToPath(reference_line, car_state, nearest_index);  //从参考线上寻找最近点
            int start_index, end_index;

            start_index = (nearest_index == 0) ? nearest_index : nearest_index - 1;
            end_index = (nearest_index + 1 == int(reference_line.size())) ? nearest_index : nearest_index + 1;
 
            // 获得在参考线上的投影点
            PathPoint point = FindProjectionPoint(reference_line.at(start_index), reference_line.at(end_index), car_state);

            start_point.x = car_state.x_now;
            start_point.y = car_state.y_now;
            start_point.yaw = car_state.yaw_now;
            start_point.s = point.s;
            start_point.speed = point.speed;
            return start_point;
        }
        else {
            nearest_index = MatchToPath(local_path, car_state, nearest_index);
            return local_path.at(nearest_index);
        }
    }

    void PathPlanner::LocalPathPlan(const vector<PathPoint> reference_line,
                                    const CarState car_state,
                                    const int plan_horizon) {
        static int nearest_index = -1;
        nearest_index = MatchToPath(reference_line, car_state, nearest_index);
        int start_index, end_index;

        start_index = (nearest_index == 0) ? nearest_index : nearest_index - 1;
        end_index = (nearest_index + 1 == int(reference_line.size())) ? nearest_index : nearest_index + 1;  
        
        PathPoint point =  FindProjectionPoint(reference_line.at(start_index), reference_line.at(end_index), car_state);

        LocalPathPlanningInterface local_path_msg; 
        double delta_s = reference_line.at(nearest_index).speed * dt; //根据参考速度计算规划间隔
        double next_s = point.s;;
 
        local_path_msg.x.push_back(point.x);
        local_path_msg.y.push_back(point.y);
        local_path_msg.angle.push_back(point.yaw);
        local_path_msg.speed.push_back(reference_line.at(nearest_index).speed);


        for (int i = 0; i < plan_horizon - 1; i++) {
            next_s += delta_s;
            // 下一个点的所在区间
            for (int j = start_index; j < int(reference_line.size()) - 2; j++) {
                if (reference_line.at(j).s >= next_s) {
                    start_index = (j == 0) ? j : j - 1;
                    break;
                }
            }
            end_index = start_index + 1;
            point = InterpolateUsingLinearApproximation(reference_line.at(start_index), reference_line.at(end_index), next_s);

            local_path_msg.x.push_back(point.x);
            local_path_msg.y.push_back(point.y);
            local_path_msg.angle.push_back(point.yaw);
            local_path_msg.speed.push_back(reference_line.at(nearest_index).speed);
        }
        for (int i = 0; i < plan_horizon - 2; i++) {
            local_path_msg.kappa.push_back(CalculateCurvature(local_path_msg.x[i+1], local_path_msg.y[i+1], local_path_msg.angle[i+1],
                                                              local_path_msg.x[i], local_path_msg.y[i], local_path_msg.angle[i]));
        }
        local_path_msg.kappa.push_back(local_path_msg.kappa.back());

        local_path_pub_->publish(local_path_msg);
        RCLCPP_INFO(get_logger(), "Now pose(%.2f,%.2f,%.2f)\tPlan pose(%.2f,%.2f,%.2f)", car_state.x_now, car_state.y_now, 
                    car_state.yaw_now, local_path_msg.x.front(), local_path_msg.y.front(), local_path_msg.angle.front());
    }

    void PathPlanner::TrajectoryPlan(const vector<PathPoint> reference_line,
                                      const CarState car_state,
                                      const int plan_horizon) {
        // 得到在参考线上的投影点
        static int nearest_index = -1;
        nearest_index = MatchToPath(reference_line, car_state, nearest_index);
        int start_index, end_index;

        start_index = (nearest_index == 0) ? nearest_index : nearest_index - 1;
        end_index = (nearest_index + 1 == int(reference_line.size())) ? nearest_index : nearest_index + 1;  
        PathPoint point =  FindProjectionPoint(reference_line.at(start_index), reference_line.at(end_index), car_state);

        static LocalPathPlanningInterface local_path_msg;
        // double delta_s = reference_line.at(nearest_index).speed * dt; //根据参考速度计算规划间隔
        double ref_speed = min(car_state.lon_speed_now + 0.5, reference_line.at(nearest_index).speed);
        double delta_s =  ref_speed * dt;

        static double next_s;
        int i;
        if (car_state.lon_speed_now < 0.1 || local_path_msg.x.size() == 0) {   // 低速下找最近点开始规划
            local_path_msg.x.clear();
            local_path_msg.y.clear();
            local_path_msg.angle.clear();
            local_path_msg.speed.clear();

            next_s = point.s;
            local_path_msg.x.push_back(point.x);
            local_path_msg.y.push_back(point.y);
            local_path_msg.angle.push_back(point.yaw);
            local_path_msg.speed.push_back(ref_speed);
            i = 0;
        }
        else {                          // 只规划最后一个点
            i = plan_horizon - 1;
        }
        for ( ; i < plan_horizon; i++) {
            next_s += delta_s;
            // 下一个点的所在区间
            for (int j = start_index; j < int(reference_line.size()) - 2; j++) {
                if (reference_line.at(j).s >= next_s) {
                    start_index = (j == 0) ? j : j - 1;
                    break;
                }
            }
            end_index = start_index + 1;
            point = InterpolateUsingLinearApproximation(reference_line.at(start_index), reference_line.at(end_index), next_s);
            local_path_msg.x.push_back(point.x);
            local_path_msg.y.push_back(point.y);
            local_path_msg.angle.push_back(point.yaw);
            local_path_msg.speed.push_back(ref_speed);
        }
        if (car_state.lon_speed_now == 0) { 
            local_path_msg.x.assign(local_path_msg.x.begin() , local_path_msg.x.end() - 1);
            local_path_msg.y.assign(local_path_msg.y.begin(), local_path_msg.y.end() - 1);
            local_path_msg.angle.assign(local_path_msg.angle.begin(), local_path_msg.angle.end() - 1);
            local_path_msg.speed.assign(local_path_msg.speed.begin(), local_path_msg.speed.end() - 1);
        }
        else {
            local_path_msg.x.assign(local_path_msg.x.begin() + 1, local_path_msg.x.end());
            local_path_msg.y.assign(local_path_msg.y.begin() + 1, local_path_msg.y.end());
            local_path_msg.angle.assign(local_path_msg.angle.begin() + 1, local_path_msg.angle.end());
            local_path_msg.speed.assign(local_path_msg.speed.begin() + 1, local_path_msg.speed.end());
        }
        local_path_pub_->publish(local_path_msg);
        RCLCPP_INFO(get_logger(), "Now pose(%.2f,%.2f,%.2f)\tPlan pose(%.2f,%.2f,%.2f)", car_state.x_now, car_state.y_now, 
                    car_state.yaw_now, local_path_msg.x.front(), local_path_msg.y.front(), local_path_msg.angle.front());
    }
    
    void PathPlanner::GpsDataCallback(const GpsInterface::SharedPtr msg) {
        msg_gps_data = msg;
        car_state.x_now = msg_gps_data->x;
        car_state.y_now = msg_gps_data->y;
        car_state.yaw_now = msg_gps_data->yaw/180*M_PI;     // From angle to radian
        car_state.pitch_now = msg_gps_data->pitch;
        car_state.lon_speed_now = pow(pow(msg_gps_data->northvelocity, 2) + pow(msg_gps_data->eastvelocity, 2) ,0.5);
    }

    int PathPlanner::MatchToPath(const vector<PathPoint>& path_points, 
                                 const CarState& car_state, 
                                 int nearest_index) {
        if (nearest_index < 0) { // 默认为负，进行全局搜索
            double dist_min = EuclideanDistance(path_points.front(), car_state.x_now, car_state.y_now);
            nearest_index = 0;
            for (size_t i = 0; i < path_points.size(); i++) {
                double dist_temp = EuclideanDistance(path_points.at(i), car_state.x_now, car_state.y_now);
                if (dist_temp < dist_min) {
                    nearest_index = i;
                    dist_min = dist_temp;
                }
            }            
        }
        else {
            double dist_min = EuclideanDistance(path_points.at(nearest_index), car_state.x_now, car_state.y_now);
            int start_index, end_index;
            start_index = max(int(0), nearest_index - 10);                  //往后数10个点作为搜索起始点
            end_index= min(int(path_points.size()), nearest_index + 10);
            for(int i = start_index; i < end_index; i++) {
                double dist_temp = EuclideanDistance(path_points.at(i), car_state.x_now, car_state.y_now);
                if (dist_temp < dist_min) {
                    nearest_index = i;
                    dist_min = dist_temp;
                }
            }
        }
        return nearest_index;
    }
}