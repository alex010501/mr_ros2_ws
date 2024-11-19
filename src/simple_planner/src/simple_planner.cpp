#include <rclcpp/rclcpp.hpp>
#include "planner.h"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<simple_planner::Planner>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  return 0;
}
