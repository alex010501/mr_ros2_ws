#include <rclcpp/rclcpp.hpp>
#include <simple_planner/planner.h>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node_options = rclcpp::NodeOptions();
    auto planner = std::make_shared<simple_planner::Planner>(node_options);
    rclcpp::spin(planner);
    rclcpp::shutdown();
    return 0;
}
