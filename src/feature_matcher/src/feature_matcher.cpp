#include <rclcpp/rclcpp.hpp>
#include "matcher.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Matcher>();

  rclcpp::spin(node);
  return 0;
}



