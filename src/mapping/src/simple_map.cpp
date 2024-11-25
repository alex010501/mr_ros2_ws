#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <vector>

// Глобальные переменные
rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;

std::string map_frame;
double map_resolution = 0.1;
int map_width = 2000;
int map_height = 2000;
nav_msgs::msg::OccupancyGrid map_msg;

void prepareMapMessage(nav_msgs::msg::OccupancyGrid& map_msg)
{
    map_msg.header.frame_id = map_frame;
    map_msg.info.height = map_height;
    map_msg.info.width = map_width;
    map_msg.info.resolution = map_resolution;
    map_msg.info.origin.position.x = -map_width * map_resolution / 2.0;
    map_msg.info.origin.position.y = -10.0;
    map_msg.data.resize(map_height * map_width, -1);
}

float log2p(float p)
{
    return std::log(p / (1 - p));
}

float calc_p(float l)
{
    return 1 - (1 / (1 + std::exp(l)));
}

int get_map(float l)
{
    float p = calc_p(l);
    if (p < 0.4)
        return 0;
    else if (p > 0.6)
        return 100;
    else
        return 50;
}

bool determineScanTransform(geometry_msgs::msg::TransformStamped& scan_transform,
                            const rclcpp::Time& stamp,
                            const std::string& laser_frame)
{
    try
    {
        scan_transform = tf_buffer->lookupTransform(map_frame, laser_frame, stamp, rclcpp::Duration::from_seconds(0.1));
    }
    catch (tf2::TransformException& ex)
    {
        RCLCPP_WARN(rclcpp::get_logger("laser_callback"), "Failed to get transform: %s", ex.what());
        return false;
    }
    return true;
}

void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    geometry_msgs::msg::TransformStamped scan_transform;
    const auto& laser_frame = scan->header.frame_id;
    const auto& laser_stamp = scan->header.stamp;

    if (!determineScanTransform(scan_transform, laser_stamp, laser_frame))
        return;

    map_msg.header.stamp = laser_stamp;

    tf2::Vector3 zero_pose(0, 0, 0);
    tf2::Quaternion rotation(
        scan_transform.transform.rotation.x,
        scan_transform.transform.rotation.y,
        scan_transform.transform.rotation.z,
        scan_transform.transform.rotation.w
    );

    tf2::Vector3 translation(
        scan_transform.transform.translation.x,
        scan_transform.transform.translation.y,
        scan_transform.transform.translation.z
    );

    tf2::Transform scan_pose_transform(rotation, translation);

    tf2::Vector3 scan_pose = scan_pose_transform.inverse() * zero_pose;

    int y = (scan_pose.y() - map_msg.info.origin.position.y) / map_resolution;
    int x = (scan_pose.x() - map_msg.info.origin.position.x) / map_resolution;

    map_msg.data[y * map_width + x] = 0;

    int size_ranges = scan->ranges.size();
    float curr_angle = scan->angle_min;
    float delta_angle = scan->angle_increment;
    int i = 0;

    while (curr_angle < scan->angle_max)
    {
        float range = scan->ranges[i];
        if (range > scan->range_min && range < scan->range_max)
        {
            float h = scan->range_min;
            while (h <= range)
            {
                tf2::Vector3 h_pose(h * cos(curr_angle), h * sin(curr_angle), 0);
                tf2::Quaternion rotation(
                    scan_transform.transform.rotation.x,
                    scan_transform.transform.rotation.y,
                    scan_transform.transform.rotation.z,
                    scan_transform.transform.rotation.w
                );

                tf2::Vector3 translation(
                    scan_transform.transform.translation.x,
                    scan_transform.transform.translation.y,
                    scan_transform.transform.translation.z
                );

                tf2::Transform scan_pose_transform(rotation, translation);

                tf2::Vector3 scan_pose = scan_pose_transform.inverse() * zero_pose;

                int h_y = (h_pose.y() - map_msg.info.origin.position.y) / map_resolution;
                int h_x = (h_pose.x() - map_msg.info.origin.position.x) / map_resolution;

                float p = 0.5;
                if (std::abs(range - h) < 0.1) {
                    p = 1.0;
                } else if (range - h > 0.1) {
                    p = 0.0;
                }

                float log_prev = log2p(static_cast<float>(map_msg.data[h_y * map_width + h_x]) / 100);
                float log_free = log2p(0.5);
                float log_inv = log2p(p);
                float log_ti = log_inv + log_prev - log_free;

                map_msg.data[h_y * map_width + h_x] = get_map(log_ti);
                h += 0.01;
            }
        }
        i++;
        curr_angle += delta_angle;
    }

    map_pub->publish(map_msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("control_node");
    map_frame = node->declare_parameter<std::string>("map_frame", "odom");
    map_resolution = node->declare_parameter("map_resolution", map_resolution);
    map_width = node->declare_parameter("map_width", map_width);
    map_height = node->declare_parameter("map_height", map_height);

    tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    map_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>("/simple_map", 10);

    prepareMapMessage(map_msg);

    auto laser_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, laserCallback);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}