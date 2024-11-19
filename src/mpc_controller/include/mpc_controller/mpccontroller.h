#pragma once
#include <mpc_controller/mpc.h>
#include <mpc_controller/trajectory_segment.h>

#include <list>
#include <memory>

#include <rclcpp/rclcpp.hpp>
// #include <rclcpp/clock.hpp>

#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>
#include <acado/acado_toolkit.hpp>


namespace mpc_controller
{

    using TrajPtr = std::shared_ptr<trajectory::TrajectorySegment>;

    /*!
     *\brief robot controller
     * controls following along defined trajectory via simple pid regulator
     * angular_velocity = pid(error)
     * error is distance to trajectory
     * trajectory is list of angular and linear segments, saved as pointers to base class Trajectory
     * Trajectory is cycled
     * feedback from robot is received via ground_truth callback (real position of robot)
     * during control future trajectory is published for velocity controller
     */
    class MPCController: public rclcpp::Node
    {
    protected:
        // MPC controller
        MPC mpc;

        // ROS
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_sub;

        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Clock::SharedPtr clock;

        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr err_pub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steer_pub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vel_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr traj_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr poly_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr mpc_traj_pub;


        // ros::NodeHandle nh;

        double robot_x = 0.0;
        double robot_y = 0.0;
        double robot_theta = 0.0;
        // time of robot coordinates update
        rclcpp::Time robot_time;
        rclcpp::Time event;

        ///\ circle params
        double radius;
        ///\ second circle center
        double cy;

        // robot params
        double wheel_base;
        double max_steer_angle;
        double max_steer_rate;
        double max_velocity;
        double max_acc;

        double current_linear_velocity = 0.0;
        double current_angular_velocity = 0.0;
        double current_curvature = 0.0;
        double current_angle = 0.0;
        double control_dt;
        // discrete of publish trajectory
        double traj_dl;
        // length of published trajectory
        double traj_length;

        double kcte;
        double kepsi;
        double kev;
        double ksteer_cost;

        using Trajectory = std::list<TrajPtr>;
        /// \ container of trajectory segments
        Trajectory trajectory;

        /// \ current segment
        Trajectory::iterator current_segment;
        /// \ length of the current segment at the current point
        double current_segment_length = 0.0;

        /// \ frame_id for coordinates of controller
        std::string world_frame_id;

        double cmd_vel = 0;
        double cmd_acc = 0;
        double cmd_steer_angle = 0;
        double cmd_steer_rate = 0;
        std::vector<double> mpc_x, mpc_y;

        std::vector<tf2::Vector3> control_points;
        double control_points_dl = 2.0;
        std::size_t control_points_num = 6;
        double mpc_steps;
        double mpc_dt;
        // coefs for y = f(x) for control points
        std::vector<double> control_coefs;

        tf2::Transform robot2world;

        void on_timer();
        void on_pose(const nav_msgs::msg::Odometry::ConstSharedPtr &odom);
        
        double cross_track_error();

        void get_segment(std::list<TrajPtr>::iterator &traj_it, double &len);
        void update_robot_pose(double dt);        
        void publish_trajectory();
        void on_odo(const nav_msgs::msg::Odometry::ConstSharedPtr &odom);
        void update_trajectory_segment();
        void publish_error(double error);
        void update_control_points();        
        void convert_control_points();
        void calculate_control_coefs();
        
        void apply_control();
        void publish_poly();
        void publish_mpc_traj(std::vector<double> &x, std::vector<double> &y);

        void add_point(sensor_msgs::msg::PointCloud &msg, const tf2::Vector3 &point);

        double polyeval(double x);

    public:
        MPCController(const std::string &ns = "mpc_controller");
        virtual ~MPCController();
        
        void reset();

        // void solveMPC();
    };

}
