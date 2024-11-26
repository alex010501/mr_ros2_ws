#include <simple_controller/controller.h>

namespace simple_controller
{

    template <class T>
    T clip(T val, T max)
    {
        if (val > max)
            return max;
        if (val < -max)
            return -max;
        return val;
    }

    std::size_t Controller::cal_target_index()
    {
        update_robot_pose((this->get_clock()->now() - robot_time).seconds());
        nearest_point_index = get_nearest_path_pose_index(nearest_point_index - 10, 20);

        const auto &nearest_pose = path.poses[nearest_point_index].pose;
        const tf2::Quaternion quat(nearest_pose.orientation.x, nearest_pose.orientation.y,
                                   nearest_pose.orientation.z, nearest_pose.orientation.w);
        double nearest_pose_angle = tf2::impl::getYaw(quat);

        double dx = robot_x - nearest_pose.position.x;
        double dy = robot_y - nearest_pose.position.y;
        // error is negative difference by y axe in the axis of the nereset pose
        double error = -(-dx * sin(nearest_pose_angle) + dy * cos(nearest_pose_angle));

        double m_error = abs(error);

        double l_d = lam * current_linear_velocity + c;
        while (l_d > m_error && (nearest_point_index + 1) < static_cast<int>(path.poses.size()))
        {
            const auto &nearest_pose_plus1 = path.poses[nearest_point_index + 1].pose;

            const tf2::Quaternion quat(nearest_pose_plus1.orientation.x, nearest_pose_plus1.orientation.y,
                                       nearest_pose_plus1.orientation.z, nearest_pose_plus1.orientation.w);
            double nearest_pose_angle_plus1 = tf2::impl::getYaw(quat);

            double dx_plus1 = robot_x - nearest_pose_plus1.position.x;
            double dy_plus1 = robot_y - nearest_pose_plus1.position.y;
            double m_error_plus1 = abs(-dx_plus1 * sin(nearest_pose_angle_plus1) + dy_plus1 * cos(nearest_pose_angle_plus1));
            m_error = m_error_plus1;
            nearest_point_index += 1;
        }
        return nearest_point_index;
    }

    void Controller::update_robot_pose(double dt)
    {
        // RCLCPP_INFO_STREAM(this->get_logger(), "update_robot_pose "<<dt<<" v = "<<current_linear_velocity );
        robot_x += current_linear_velocity * dt * sin(robot_theta);
        robot_y += current_linear_velocity * dt * cos(robot_theta);
        robot_theta = angles::normalize_angle(robot_theta + current_angular_velocity * dt);
        robot_time += rclcpp::Duration(std::chrono::milliseconds((int)(dt * 1000)));
        // RCLCPP_INFO_STREAM(this->get_logger(), "leaving update");
    }

    void Controller::on_path(const nav_msgs::msg::Path &path)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Got path " << path.poses.size());
        this->path = path;
        nearest_point_index = 0;
    }

    // search a pose in the pat nearest to the robot, assume path may be cyclic
    std::size_t Controller::get_nearest_path_pose_index(int start_index,
                                                        std::size_t search_len)
    {
        // RCLCPP_INFO_STREAM(this->get_logger(), "getting nearest index");
        double nearest_distance = 1e10;
        std::size_t index = start_index;
        std::size_t nearest_index;
        geometry_msgs::msg::Pose nearest_pose;
        for (int index = start_index; index < start_index + static_cast<int>(search_len); ++index)
        {
            std::size_t real_index;
            if (index >= 0 && index < static_cast<int>(path.poses.size()))
            {
                real_index = static_cast<std::size_t>(index);
            }
            if (index < 0)
            {
                real_index = (static_cast<int>(path.poses.size()) + index);
            }
            if (index >= static_cast<int>(path.poses.size()))
            {
                real_index = static_cast<std::size_t>(index) - path.poses.size();
            }

            const auto &path_point = path.poses[real_index].pose.position;
            double dx = robot_x - path_point.x;
            double dy = robot_y - path_point.y;
            double distance_sqr = dx * dx + dy * dy;
            // RCLCPP_INFO_STREAM(this->get_logger(), "robot coords " << robot_x << " " << robot_y);
            // RCLCPP_INFO_STREAM(this->get_logger(), "point coords " << path_point.x << " " << path_point.y);
            if (distance_sqr < nearest_distance)
            {
                nearest_distance = distance_sqr;
                nearest_index = real_index;
                // RCLCPP_INFO_STREAM(this->get_logger(), "real_index" << real_index);
                // RCLCPP_INFO_STREAM(this->get_logger(), "assigned_index" << nearest_index);
            }
        }
        // RCLCPP_INFO_STREAM(this->get_logger(), "index " << nearest_index);
        return nearest_index;
    }

    void Controller::on_timer()
    {
        if (std::abs(current_linear_velocity) < 0.01)
        {
            return;
        }
        // RCLCPP_INFO_STREAM(this->get_logger(), "Entered timer body");
        update_robot_pose((this->get_clock()->now() - robot_time).seconds());
        ;
        nearest_point_index = cal_target_index();

        const auto &nearest_pose = path.poses[nearest_point_index].pose;
        const tf2::Quaternion quat(nearest_pose.orientation.x, nearest_pose.orientation.y,
                                   nearest_pose.orientation.z, nearest_pose.orientation.w);
        double nearest_pose_angle = tf2::impl::getYaw(quat);
        double dx = robot_x - nearest_pose.position.x;
        double dy = robot_y - nearest_pose.position.y;
        // RCLCPP_INFO_STREAM(this->get_logger(), "target_pose " << nearest_pose.position.x << " " << nearest_pose.position.y);
        // error is negative difference by y axe in the axis of the nereset pose
        double error = -(-dx * sin(nearest_pose_angle) + dy * cos(nearest_pose_angle));

        double l_d = lam * current_linear_velocity + c;
        // alpha
        double base2nearest_pose_angle = (nearest_pose_angle - robot_theta);
        // angular
        double angular_m_cmd = atan2(2 * 1.88 * sin(base2nearest_pose_angle), l_d);
        double curvature = angular_m_cmd / current_linear_velocity;
        // send curvature as command to drives
        std_msgs::msg::Float32 cmd;
        cmd.data = clip<double>(curvature, max_curvature);
        steer_pub_->publish(cmd);

        // send trajectory for velocity controller
        publish_trajectory();

        // send error for debug proposes
        publish_error(error);
        // RCLCPP_INFO_STREAM(this->get_logger(), "steering cmd = "<<curvature);
    }

    void Controller::on_pose(const nav_msgs::msg::Odometry &odom)
    {
        robot_x = odom.pose.pose.position.x;
        robot_y = odom.pose.pose.position.y;
        robot_theta = 2 * atan2(odom.pose.pose.orientation.z,
                                odom.pose.pose.orientation.w);

        world_frame_id = odom.header.frame_id;
        robot_time = this->get_clock()->now();
        // RCLCPP_INFO_STREAM(this->get_logger(), "on_pose_time" << robot_time.seconds());
    }

    void Controller::on_odo(const nav_msgs::msg::Odometry &odom)
    {
        current_linear_velocity = odom.twist.twist.linear.x;
        current_angular_velocity = odom.twist.twist.angular.z;
        // RCLCPP_INFO_STREAM(this->get_logger(), "odom vel = "<< current_linear_velocity);
    }

    void Controller::publish_error(double error)
    {
        std_msgs::msg::Float32 err_msg;
        err_msg.data = error;
        err_pub_->publish(err_msg);
    }

    double Controller::cross_track_error()
    {
        double error = 0.0;
        if (robot_y < radius)
        {
            double rx = robot_x;
            double ry = robot_y - radius;
            error = sqrt(rx * rx + ry * ry) - radius;
        }
        else if (robot_y > cy)
        {
            double rx = robot_x;
            double ry = robot_y - cy;
            error = sqrt(rx * rx + ry * ry) - radius;
        }
        else if (robot_x > 0)
        {
            error = robot_x - radius;
        }
        else if (robot_x < 0)
        {
            error = -radius - robot_x;
        }
        return error;
    }

    void Controller::publish_trajectory()
    {
        //  ROS_DEBUG_STREAM("publish trajectory");
        path_pub_->publish(path);
    }

    void Controller::reset()
    {
        error_integral = 0.0;
        last_error = cross_track_error();
    }

    void Controller::reset(double p, double d, double i)
    {
        reset();
        p_factor = p;
        d_factor = d;
        i_factor = i;
    }

    nav_msgs::msg::Path Controller::create_path() const
    {
        // prepare path message from trajectory
        nav_msgs::msg::Path path;
        path.header.frame_id = "odom";
        path.header.stamp = robot_time;
        auto segment_it = trajectory.begin();
        double previous_segment_left = 0.0;
        std::size_t points_added = 0;
        double point_length = 0.0;

        while (segment_it != trajectory.end())
        {
            const auto segment = *segment_it;
            double segment_length = segment->get_length();
            // add  points from the segment
            while (point_length <= segment_length)
            {
                const auto point = segment->get_point(point_length);
                const auto angle = segment->get_orientation(point_length);
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "odom";
                pose.pose.position.x = point.x();
                pose.pose.position.y = point.y();
                tf2::Quaternion quat(0, 0, 0, 1);
                quat.setRPY(0, 0, angle);
                pose.pose.orientation.x = quat.getX();
                pose.pose.orientation.y = quat.getY();
                pose.pose.orientation.z = quat.getZ();
                pose.pose.orientation.w = quat.getW();
                path.poses.push_back(pose);
                point_length += traj_dl;
                points_added++;
            }
            point_length -= segment_length;
            ++segment_it;
        }
        return path;
    }

    /*!
     * \brief constructor
     * loads parameters from ns
     * proportional, differential , integral - pid factors
     * max_antiwindup_error - max error for using integral component
     * trajectory consists of two circle segments connected with two lines
     * first circle center is (0, radius), second circle center is (0, cy)
     * radius - radius of circular parts
     * cy - center of second circle
     * traj_dl - discrete of published trajectory
     * traj_length - length of published trajectory
     * timer_period - timer discrete
     */
    Controller::Controller() : Node("simple_controller"),
                               p_factor(this->declare_parameter("proportional", 1.0)),
                               d_factor(this->declare_parameter("differential", 0.0)),
                               i_factor(this->declare_parameter("integral", 0.0)),
                               max_antiwindup_error(this->declare_parameter("max_antiwindup_error", 0.5)),
                               error_integral(0.0),
                               last_error(0.0),
                               radius(this->declare_parameter("radius", 10.0)),
                               cy(this->declare_parameter("cy", 2 * radius)), // default circle is in center (0,radius)
                               max_curvature(this->declare_parameter("max_curvature", 0.2)),
                               traj_dl(this->declare_parameter("traj_dl", 0.2)),
                               traj_length(this->declare_parameter("traj_length", 5.0))
    {
        using namespace std::placeholders;
        using namespace std::chrono_literals;

        err_pub_ = this->create_publisher<std_msgs::msg::Float32>("error", 10);
        steer_pub_ = this->create_publisher<std_msgs::msg::Float32>("/steering", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("controller_path", 1);

        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("ground_truth", 1, std::bind(&Controller::on_pose, this, _1));
        odo_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1, std::bind(&Controller::on_odo, this, _1));
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>("path", 1, std::bind(&Controller::on_path, this, _1));

        timer_ = this->create_wall_timer(100ms, std::bind(&Controller::on_timer, this));

        // counter clock
        trajectory.emplace_back(std::make_shared<trajectory::CircularSegment>(1.0 / radius, 0, 0, 1.0, 0, M_PI / 2 * radius));
        trajectory.emplace_back(std::make_shared<trajectory::LinearSegment>(radius, radius, 0.0, 1.0, cy - radius));
        trajectory.emplace_back(std::make_shared<trajectory::CircularSegment>(1.0 / radius, radius, cy, 0.0, 1.0, M_PI / 2 * radius));
        trajectory.emplace_back(std::make_shared<trajectory::CircularSegment>(1.0 / radius, 0, radius + cy, -1.0, 0.0, M_PI / 2 * radius));
        trajectory.emplace_back(std::make_shared<trajectory::LinearSegment>(-radius, cy, 0.0, -1.0, cy - radius));
        trajectory.emplace_back(std::make_shared<trajectory::CircularSegment>(1.0 / radius, -radius, radius, 0.0, -1.0, M_PI / 2 * radius));

        // clock wise track
        //  trajectory.emplace_back( std::make_shared<trajectory::CircularTrajectory>( -1.0 / radius,    0,       0,    1.0,   0,   M_PI/2*radius) );
        //  trajectory.emplace_back( std::make_shared<trajectory::LinearTrajectory>  (        radius, -radius, 0.0,   -1.0,  cy - radius) );
        //  trajectory.emplace_back( std::make_shared<trajectory::CircularTrajectory>( -1.0 / radius,   radius,   -cy,   0.0,   -1.0, M_PI/2*radius ) );
        //  trajectory.emplace_back( std::make_shared<trajectory::CircularTrajectory>( -1.0 / radius,   0, -radius - cy,   -1.0,  0.0, M_PI/2*radius ) );
        //  trajectory.emplace_back( std::make_shared<trajectory::LinearTrajectory>  (       -radius, -cy,   0.0,   1.0, cy - radius) );
        //  trajectory.emplace_back( std::make_shared<trajectory::CircularTrajectory>( -1.0/ radius,   -radius, -radius, 0.0,  1.0,  M_PI/2*radius) );
        current_segment = trajectory.begin();
        const auto trajectory_path = create_path();

        robot_time = this->get_clock()->now();
        // RCLCPP_INFO_STREAM(this->get_logger(), "start_time" << robot_time.seconds());
        on_path(trajectory_path);
    }

    Controller::~Controller()
    {
        // TODO Auto-generated destructor stub
    }

}