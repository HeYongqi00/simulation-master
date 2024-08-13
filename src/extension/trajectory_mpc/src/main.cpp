#include "trajectory_mpc.hpp"
int main(int argc, char **argv)
{
    double loop_hz = 20;  //
    std::string node_name = "trajectory_mpc";

    // Force flush of the stdout buffer
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Init ROS
    rclcpp::init(argc, argv);
    // Create single-threaded executor
    rclcpp::executors::SingleThreadedExecutor executor;


    // Allow running in-process communication
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(false);
    // Create and add driver node
    auto node = std::make_shared<control::MPCController>(node_name, loop_hz, options);

    // Add node to the executor
    executor.add_node(node);
    // Spin until rclcpp::ok() returns false
    executor.spin();

    // Shut down ROS
    rclcpp::shutdown();

}