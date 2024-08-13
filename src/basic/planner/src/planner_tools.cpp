#include "planner_tools.hpp"

using namespace math;

double math::EuclideanDistance(const PathPoint& point, const double x, const double y) {
    double dx = point.x - x;
    double dy = point.y - y;
    return dx * dx + dy * dy;
}

double math::CalculateCurvature(double x1, double y1, double theta1,
                                        double x2, double y2, double theta2) {
    while (theta1 - theta2 > M_PI) {
        theta1 -= 2*M_PI;
    }
    while (theta2 - theta1 > M_PI) {
        theta2 -= 2*M_PI;
    }
    return (theta1 - theta2) / sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

PathPoint math::InterpolateUsingLinearApproximation(const PathPoint& p0, const PathPoint& p1, const double s) {
    double s0 = p0.s;
    double s1 = p1.s;

    PathPoint path_point;
    double weight = (s - s0) / (s1 - s0);
    double x = (1 - weight) * p0.x + weight * p1.x;
    double y = (1 - weight) * p0.y + weight * p1.y;
    double yaw = slerp(p0.yaw, p0.s, p1.yaw, p1.s, s);
    
    path_point.x = x;
    path_point.y = y;
    path_point.yaw = yaw;
    path_point.s = s;
    return path_point;
}

double math::slerp(const double a0, const double t0, const double a1, const double t1, const double t) {
    if (abs(t1 - t0) <= 0.001f) {
        return a0;
    }
    double d = a1 - a0;
    if (d > M_PI)
        d = d - 2 * M_PI;
    else if (d < -M_PI)
        d = d + 2 * M_PI;

    const double r = (t - t0) / (t1 - t0);
    const double a = a0 + d * r;
    return a;
}

PathPoint math::FindProjectionPoint(const PathPoint& p0, const PathPoint& p1, const CarState& car_state) {
    
    double v0x = car_state.x_now - p0.x;
    double v0y = car_state.y_now - p0.y;

    double v1x = p1.x - p0.x;
    double v1y = p1.y - p0.y;

    double v1_norm = sqrt(v1x * v1x + v1y * v1y);
    double dot = v0x * v1x + v0y * v1y;

    double delta_s = dot / v1_norm;
    return InterpolateUsingLinearApproximation(p0, p1, p0.s + delta_s);
}