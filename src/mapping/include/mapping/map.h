#pragma once

#include <vector>
#include <string>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class Map : public rclcpp::Node
{
public:
    Map();

private:
    void prepareMapMessage();
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    bool determineScanTransform(geometry_msgs::msg::TransformStamped& scan_transform,
                                const rclcpp::Time& stamp,
                                const std::string& laser_frame);
    float log2p(float p);
    float calc_p(float l);
    int get_map(float l);

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string map_frame_;
    double map_resolution_;
    int map_width_;
    int map_height_;
    nav_msgs::msg::OccupancyGrid map_msg_;
};