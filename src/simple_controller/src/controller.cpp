#include <simple_controller/controller.h>

namespace simple_controller
{
    template <class T>
    T clip(T val, T max)
    {
        return std::max(std::min(val, max), -max);
    }

    void Controller::update_robot_pose(double dt)
    {
        RCLCPP_DEBUG(this->get_logger(), "update_robot_pose %.2f v = %.2f", dt, current_linear_velocity);
        robot_x += current_linear_velocity * dt * sin(robot_theta);
        robot_y += current_linear_velocity * dt * cos(robot_theta);
        robot_theta = angles::normalize_angle(robot_theta + current_angular_velocity * dt);
        robot_time += rclcpp::Duration::from_seconds(dt);
    }

    void Controller::on_path(const nav_msgs::msg::Path &path)
    {
        RCLCPP_INFO(this->get_logger(), "Got path size: %ld", path.poses.size());
        this->path = path;
        nearest_point_index = 0;
    }

    void Controller::on_odo(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        odo_received_ = true;
        odo_sub_.reset(); // Unsubscribe after first odometry message
        RCLCPP_DEBUG(this->get_logger(), "Odometry received");
    }

    std::size_t Controller::get_nearest_path_pose_index(int start_index, std::size_t search_len)
    {
        double nearest_distance = std::numeric_limits<double>::max();
        std::size_t nearest_index;

        for (int index = start_index; index < start_index + static_cast<int>(search_len); ++index)
        {
            std::size_t real_index = index % path.poses.size();
            const auto &path_point = path.poses[real_index].pose.position;
            double dx = robot_x - path_point.x;
            double dy = robot_y - path_point.y;
            double distance_sqr = dx * dx + dy * dy;
            if (distance_sqr < nearest_distance)
            {
                nearest_distance = distance_sqr;
                nearest_index = real_index;
            }
        }
        return nearest_index;
    }

    void Controller::on_timer()
    {
        if (std::abs(current_linear_velocity) < 0.01)
            return;

        update_robot_pose((this->now() - robot_time).seconds());
        nearest_point_index = get_nearest_path_pose_index(nearest_point_index - 10, 20);
        publish_trajectory();
    }

    void Controller::on_pose(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_x = msg->pose.pose.position.x;
        robot_y = msg->pose.pose.position.y;
        robot_theta = 2 * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

        world_frame_id = msg->header.frame_id;
        robot_time = msg->header.stamp;
    }

    void Controller::publish_trajectory()
    {
        path_pub->publish(path);
    }

    void Controller::initialize_controller()
    {
        if (!odo_received_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for odometry...");
            return;
        }

        // Fetch PID parameters
        double p_factor = this->get_parameter("proportional").as_double();
        double d_factor = this->get_parameter("differential").as_double();
        double i_factor = this->get_parameter("integral").as_double();

        // Configure the controller with parameters
        controller_->reset(p_factor, d_factor, i_factor);

        RCLCPP_INFO(this->get_logger(), "Controller initialized with P: %f, D: %f, I: %f", p_factor, d_factor, i_factor);

        // Shutdown the timer as it's no longer needed
        init_timer_.reset();
    }

    Controller::Controller() : Node("simple_controller")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing Simple Controller Node");

        p_factor = this->declare_parameter("proportional", 1.0);
        d_factor = this->declare_parameter("differential", 0.0);
        i_factor = this->declare_parameter("integral", 0.0);
        
        init_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Controller::initialize_controller, this))

        odo_sub = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&Controller::on_odo, this, std::placeholders::_1));
        pose_sub = this->create_subscription<nav_msgs::msg::Odometry>("ground_truth", 10, std::bind(&Controller::on_pose, this, std::placeholders::_1));
        path_pub = this->create_publisher<nav_msgs::msg::Path>("controller_path", 10);
        timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Controller::on_timer, this));
    }

    Controller::~Controller() {}

} // namespace simple_controller
