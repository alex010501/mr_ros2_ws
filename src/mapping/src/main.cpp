#include <mapping/map.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Map>());
    rclcpp::shutdown();
    return 0;
}
