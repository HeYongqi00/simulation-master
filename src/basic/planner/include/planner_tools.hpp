#include <cmath>

#include <vector>

using namespace std;

struct CarState {
    double x_now;
    double y_now;
    double yaw_now;
    double pitch_now;
    double lon_speed_now;
    double lat_speed_now;
    double lon_acc_now;
    double lat_acc_now;
};

struct PathPoint {
    double x;
    double y;
    double yaw;
    double s;
    double speed;
};

namespace math {
    /**
     * @brief 根据车辆位置返回投影点
     * @param p0 距离车辆最近点的前一个点
     * @param p1 距离车辆最近点的后一个点
     * @return 在参考线上的投影点
    */
    PathPoint FindProjectionPoint(const PathPoint& p0, const PathPoint& p1, const CarState& car_state);

    /**
     * @brief 计算欧拉距离
     * @return 欧拉距离 d = sqrt(dx^2 + dy^2)
    */
    double EuclideanDistance(const PathPoint& point, const double x, const double y);

    /**
    * @brief 计算道路曲率
    * @return kappa = d(theta) / sqrt(dx^2 + dy^2)
    */
    double CalculateCurvature(double x1, double y1, double theta1,
                              double x2, double y2, double theta2);
    
    /**
     * @brief 两点之间线性插值
     * @return 插值点
    */
    PathPoint InterpolateUsingLinearApproximation(const PathPoint& p0, const PathPoint& p1, const double s);

    /**
     * @brief 航向角插值 需要考虑航向角的切换
     * @return 插值航向角
    */
    double slerp(const double a0, const double t0, const double a1, const double t1, const double s);
}