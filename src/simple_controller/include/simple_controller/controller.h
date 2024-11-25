#pragma once

#include <list>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <angles/angles.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <simple_controller/trajectory.h>

namespace simple_controller
{
    using TrajPtr = std::shared_ptr<trajectory::TrajectorySegment>;

    class Controller : public rclcpp::Node
    {
    private:
        double robot_x = 0.0;
        double robot_y = 0.0;
        double robot_theta = 0.0;

        rclcpp::Time robot_time;
        double p_factor;
        double d_factor;
        double i_factor;
        double max_antiwindup_error;
        double error_integral;
        double last_error;

        double radius;
        double cy;
        double max_curvature;

        double current_linear_velocity = 0.0;
        double current_angular_velocity = 0.0;

        double traj_dl;
        double traj_length;

        using Trajectory = std::list<TrajPtr>;
        std::list<TrajPtr> trajectory;

        nav_msgs::msg::Path path;
        std::size_t nearest_point_index;

        std::list<TrajPtr>::iterator current_segment;
        double current_segment_length = 0.0;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_sub;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::TimerBase::SharedPtr init_timer;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr err_pub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steer_pub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

        std::string world_frame_id;

        void on_timer();
        void on_pose(const nav_msgs::msg::Odometry::SharedPtr msg);
        void on_path(const nav_msgs::msg::Path &path);

        void on_odo(const nav_msgs::msg::Odometry::SharedPtr odom);
        void initialize_controller();

        double cross_track_error();
        void update_robot_pose(double dt);
        void publish_trajectory();
        void on_odo(const nav_msgs::msg::Odometry::SharedPtr msg);
        void publish_error(double error);
        nav_msgs::msg::Path create_path() const;
        std::size_t get_nearest_path_pose_index(int start_index, std::size_t search_len);

    public:
        double get_p_factor() { return p_factor; }
        double get_d_factor() { return d_factor; }
        double get_i_factor() { return i_factor; }
        void reset();
        void reset(double p, double d, double i);
        Controller();
        virtual ~Controller();
    };
}