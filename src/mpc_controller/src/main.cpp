#include <rclcpp/rclcpp.hpp>
#include <mpc_controller/mpccontroller.h>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // Create the MPCController node
    auto node = std::make_shared<mpc_controller::MPCController>("mpc_controller");

    // Start spinning the node
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}