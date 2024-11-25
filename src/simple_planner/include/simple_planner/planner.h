#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <limits>
#include <queue>
#include <vector>

namespace simple_planner
{

    struct SearchNode
    {
        enum State
        {
            CLOSE,
            OPEN,
            UNDEFINED
        };
        State state = UNDEFINED;
        double g = std::numeric_limits<double>::max();
        double h = 0;
    };

    struct MapIndex
    {
        int i;
        int j;
    };

    class Planner : public rclcpp::Node
    {
    public:
        explicit Planner(const rclcpp::NodeOptions &options);

    private:
        friend class CompareSearchNodes;

        void on_pose(const nav_msgs::msg::Odometry::SharedPtr odom);
        void on_target(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
        bool update_static_map();
        void increase_obstacles(std::size_t cells);
        void calculate_path();
        double heuristic(int i, int j);
        bool indices_in_map(int i, int j);
        template <class T>
        T &map_value(std::vector<T> &data, int i, int j)
        {
            int index = j * map_.info.width + i;
            assert(index < static_cast<int>(data.size()) && index >= 0);
            return data[index];
        }
        MapIndex point_index(double x, double y)
        {
            return {
                static_cast<int>(floor((x - map_.info.origin.position.x) / map_.info.resolution)),
                static_cast<int>(floor((y - map_.info.origin.position.y) / map_.info.resolution))};
        }

        nav_msgs::msg::OccupancyGrid map_;
        nav_msgs::msg::OccupancyGrid obstacle_map_;
        std::vector<SearchNode> search_map_;
        sensor_msgs::msg::PointCloud path_msg_;

        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacle_map_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr path_publisher_;
        rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr map_server_client_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;

        geometry_msgs::msg::Pose start_pose_;
        geometry_msgs::msg::Pose target_pose_;
        double robot_radius_;
    };

}