#include <ekf_slam/slam.h>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Create the MPCController node
    auto node = std::make_shared<Slam>("ekf_slam");

    // Start spinning the node
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
