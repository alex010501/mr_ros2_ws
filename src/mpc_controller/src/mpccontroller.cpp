#include <mpc_controller/mpccontroller.h>

namespace mpc_controller
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

    void MPCController::update_trajectory_segment()
    {
        current_segment_length = (*current_segment)->get_point_length(robot_x, robot_y);

        while (current_segment_length < 0.0)
        {
            if (current_segment == trajectory.begin())
                current_segment = trajectory.end();

            --current_segment;
            current_segment_length = (*current_segment)->get_point_length(robot_x, robot_y);
        }
        while (current_segment_length > (*current_segment)->get_length())
        {
            ++current_segment;
            if (current_segment == trajectory.end())
                current_segment = trajectory.begin();
            current_segment_length = (*current_segment)->get_point_length(robot_x, robot_y);
        }
    }

    void MPCController::update_control_points()
    {
        control_points.resize(control_points_num);
        Trajectory::iterator segment = current_segment;
        tf2::Vector3 pose(robot_x, robot_y, 0);
        RCLCPP_DEBUG(this->get_logger(), "control points");
        double control_point_distance = (*segment)->get_point_length(pose.x(), pose.y());
        for (std::size_t i = 0; i < control_points_num; ++i)
        {
            control_point_distance += i * control_points_dl;
            while (control_point_distance > (*segment)->get_length())
            {
                control_point_distance -= (*segment)->get_length();
                ++segment;
                if (segment == trajectory.end())
                    segment = trajectory.begin();
            }
            control_points[i] = (*segment)->get_point(control_point_distance);
            RCLCPP_DEBUG(this->get_logger(), "%lu: %f %f", i, control_points[i].x(), control_points[i].y());
        }
    }

    // converts control points into robot coordinate frame
    void MPCController::convert_control_points()
    {
        tf2::Transform world2robot = robot2world.inverse();
        RCLCPP_DEBUG(this->get_logger(), "control points in robot coordinates ");
        for (auto &point : control_points)
        {
            point = world2robot(point);
            RCLCPP_DEBUG(this->get_logger(), "%f %f", point.x(), point.y());
        }
    }

    // calculates polynom coefficients
    void MPCController::calculate_control_coefs()
    {
        const int order = 3; // we have 4 coefficients
        assert(order <= control_points.size() - 1);
        Eigen::MatrixXd A(control_points.size(), order + 1);

        for (int i = 0; i < control_points.size(); ++i)
        {
            A(i, 0) = 1.0;
        }
        Eigen::VectorXd yvals(control_points.size());
        for (int j = 0; j < control_points.size(); j++)
        {
            yvals(j) = control_points[j].y();
            for (int i = 0; i < order; i++)
            {
                A(j, i + 1) = A(j, i) * control_points[j].x();
            }
        }
        auto Q = A.householderQr();
        Eigen::VectorXd result = Q.solve(yvals);
        control_coefs.assign(result.data(), result.data() + result.size());
        RCLCPP_DEBUG(this->get_logger(), "coefs: %f %f %f %f", control_coefs[0], control_coefs[1], control_coefs[2], control_coefs[3]);
    }

    double MPCController::polyeval(double x)
    {
        double result = control_coefs[0];
        double ax = 1.0;
        for (int i = 1; i < control_coefs.size(); ++i)
        {
            ax *= x;
            result += ax * control_coefs[i];
        }
        return result;
    }

    /// \ update robot pose to current time based on last pose and velocities
    void MPCController::update_robot_pose(double dt)
    {
        //  ROS_DEBUG_STREAM("update_robot_pose "<<dt<<" v = "<<current_linear_velocity );
        robot_x += current_linear_velocity * dt * cos(robot_theta);
        robot_y += current_linear_velocity * dt * sin(robot_theta);
        robot_theta = angles::normalize_angle(robot_theta + current_angular_velocity * dt);
        robot_time += rclcpp::Duration::from_seconds(dt);
        robot2world.setOrigin(tf2::Vector3(robot_x, robot_y, 0));
        tf2::Quaternion q;
        q.setRPY(0, 0, robot_theta); // Roll = 0, Pitch = 0, Yaw = robot_theta
        robot2world.setRotation(q);
        // robot2world.setRotation(tf2::createQuaternionFromYaw(robot_theta));
    }

    void MPCController::apply_control()
    {
        cmd_vel += cmd_acc * control_dt;
        cmd_steer_angle += cmd_steer_rate * control_dt;
        cmd_steer_angle = clip<double>(cmd_steer_angle, max_steer_angle);
        // send curvature as command to drives
        std_msgs::msg::Float32 cmd;
        cmd.data = cmd_steer_angle;
        steer_pub->publish(cmd);
        // send velocity as command to drives
        cmd.data = cmd_vel;
        vel_pub->publish(cmd);
        RCLCPP_DEBUG(this->get_logger(), "cmd v = %f angle = %f", cmd_vel, cmd_steer_angle);
    }

    void MPCController::on_timer()
    {
        rclcpp::Time current_time = clock->now();

        apply_control();
        if (current_time.get_clock_type() == robot_time.get_clock_type())
            update_robot_pose((current_time - robot_time).seconds() + control_dt);
        else
            RCLCPP_ERROR(this->get_logger(), "Different time sources");
        update_trajectory_segment();

        update_control_points();
        convert_control_points();
        calculate_control_coefs();

        double error = control_coefs[0];
        RCLCPP_DEBUG(this->get_logger(), "error = %f", error);

        rclcpp::Time start_solve = clock->now();
        // const auto start_solve = std::chrono::steady_clock::now();
        mpc.solve(current_linear_velocity, cmd_steer_angle, control_coefs, cmd_steer_rate, cmd_acc, mpc_x, mpc_y);
        double solve_time = (clock->now() - start_solve).seconds();
        RCLCPP_DEBUG(this->get_logger(), "solve time = %f", solve_time);
        RCLCPP_ERROR(this->get_logger(), "Solve time too big %f", solve_time);
        publish_trajectory();
        publish_poly();
        // send error for debug proposes
        publish_error(cross_track_error());
        publish_mpc_traj(mpc_x, mpc_y);
    }

    void MPCController::on_pose(const nav_msgs::msg::Odometry::ConstSharedPtr &odom)
    {
        robot_x = odom->pose.pose.position.x;
        robot_y = odom->pose.pose.position.y;
        robot_theta = 2 * atan2(odom->pose.pose.orientation.z,
                                odom->pose.pose.orientation.w);

        world_frame_id = odom->header.frame_id;
        robot_time = clock->now();
    }

    void MPCController::on_odo(const nav_msgs::msg::Odometry::ConstSharedPtr &odom)
    {
        current_linear_velocity = odom->twist.twist.linear.x;
        current_angular_velocity = odom->twist.twist.angular.z;
        if (std::abs(current_linear_velocity) < 0.01)
        {
            current_curvature = current_angular_velocity / current_linear_velocity;
            current_angle = atan(current_curvature * wheel_base);
        }
        // else left the same as before
        RCLCPP_DEBUG(this->get_logger(), "odom vel = %f w = %f angle = %f", current_linear_velocity, current_angular_velocity, current_angle);
    }

    void MPCController::publish_error(double error)
    {
        std_msgs::msg::Float32 err_msg;
        err_msg.data = error;
        err_pub->publish(err_msg);
    }

    /*
     *@brief calculates feedback error for trajectory
     *@return feedback error
    */
    double MPCController::cross_track_error()
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

    /// \ returns iterator to segment and current length  of trajectory belonging to current position
    void MPCController::get_segment(std::list<TrajPtr>::iterator &traj_it, double &len)
    {
        traj_it = trajectory.end();

        if (robot_y < radius)
        {
            if (robot_x >= 0)
            {
                traj_it = trajectory.begin();
            }
        }
    }

    void MPCController::add_point(sensor_msgs::msg::PointCloud &msg, const tf2::Vector3 &point)
    {
        geometry_msgs::msg::Point32 p;
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
        msg.points.push_back(p);
    }

    /*
     * \brief publishes trajectory as pointcloud message
     */
    void MPCController::publish_trajectory()
    {
        // prepare pointcloud message
        auto msg = std::make_shared<sensor_msgs::msg::PointCloud>();
        msg->header.frame_id = world_frame_id;
        msg->header.stamp = clock->now();

        double trajectory_points_quantity = traj_length / traj_dl + 1;
        int points_left = trajectory_points_quantity;
        msg->points.reserve(trajectory_points_quantity);
        double publish_len = 0;
        Trajectory::iterator it = current_segment;
        double start_segment_length = current_segment_length;

        while (points_left)
        {
            double segment_length = (*it)->get_length();
            // add points from the segment
            int segment_points_quantity = std::min<int>(points_left, std::floor((segment_length - start_segment_length) / traj_dl));
            
            for (int i = 0; i <= segment_points_quantity; ++i)
            {
                add_point(*msg, (*it)->get_point(start_segment_length + i * traj_dl));
            }
            points_left -= segment_points_quantity;
            // switch to next segment
            if (points_left)
            {
                // start point for next segment
                start_segment_length += (segment_points_quantity + 1) * traj_dl - segment_length;
                
                ++it;
                if (it == trajectory.end())
                    it = trajectory.begin();
            }
        }
        traj_pub->publish(*msg);
    }

    void MPCController::publish_poly()
    {
        // prepare pointcloud message
        auto msg = std::make_shared<sensor_msgs::msg::PointCloud>();
        msg->header.frame_id = world_frame_id;
        msg->header.stamp = clock->now();

        double xrange = control_points_dl * control_points_num * 1.5;
        int trajectory_points_quantity = xrange / traj_dl;
        msg->points.reserve(trajectory_points_quantity);

        for (int i = 0; i < trajectory_points_quantity; ++i)
        {
            double x = i * traj_dl;
            tf2::Vector3 point = robot2world(tf2::Vector3(x, polyeval(x), 0));
            add_point(*msg, point);
        }
        poly_pub->publish(*msg);
    }

    void MPCController::publish_mpc_traj(std::vector<double> &x, std::vector<double> &y)
    {
        if (x.empty())
            return;
        
        auto msg = std::make_shared<sensor_msgs::msg::PointCloud>();
        msg->header.frame_id = world_frame_id;
        msg->header.stamp = clock->now();
        
        msg->points.reserve(x.size());
        for (int i = 0; i < x.size(); ++i)
        {
            add_point(*msg, robot2world(tf2::Vector3(x[i], y[i], 0)));
        }
        mpc_traj_pub->publish(*msg);
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
    MPCController::MPCController(const std::string &ns): Node(ns),
                                                         radius(this->declare_parameter<double>("radius", 25.0)),
                                                         cy(this->declare_parameter<double>("cy", 70.0)),
                                                         wheel_base(this->declare_parameter<double>("wheel_base", 1.88)),
                                                         max_steer_angle(this->declare_parameter<double>("max_steer_angle", 0.3)),
                                                         max_steer_rate(this->declare_parameter<double>("max_steer_rate", 0.3)),
                                                         max_velocity(this->declare_parameter<double>("max_velocity", 10.0)),
                                                         max_acc(this->declare_parameter<double>("max_acc", 0.8)),
                                                         traj_dl(this->declare_parameter<double>("traj_dl", 0.2)),
                                                         traj_length(this->declare_parameter<double>("traj_length", 50)),
                                                         control_dt(this->declare_parameter<double>("timer_period", 0.05)),
                                                         mpc_steps(this->declare_parameter<int>("mpc_steps", 6)),
                                                         mpc_dt(this->declare_parameter<double>("mpc_dt", 0.15)),
                                                         kcte(this->declare_parameter<double>("kcte", 1.1)),
                                                         kepsi(this->declare_parameter<double>("kepsi", 2.0)),
                                                         kev(this->declare_parameter<double>("kev", 0.1)),
                                                         ksteer_cost(this->declare_parameter<double>("ksteer_cost", 5.0)),
                                                         mpc()                                                      
    {
        mpc.init(mpc_steps, mpc_dt, max_velocity, max_acc, max_steer_angle, max_steer_rate, wheel_base, kcte, kepsi, kev, ksteer_cost);

        // Set the clock type to system time
        clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

        // Subscriptions
        pose_sub = this->create_subscription<nav_msgs::msg::Odometry>("ground_truth", 1, std::bind(&MPCController::on_pose, this, std::placeholders::_1));
        odo_sub = this->create_subscription<nav_msgs::msg::Odometry>("odom", 1, std::bind(&MPCController::on_odo, this, std::placeholders::_1));

        // Publishers
        err_pub = this->create_publisher<std_msgs::msg::Float32>("error", 10);
        steer_pub = this->create_publisher<std_msgs::msg::Float32>("/steering", 1);
        vel_pub = this->create_publisher<std_msgs::msg::Float32>("/velocity", 1);
        traj_pub = this->create_publisher<sensor_msgs::msg::PointCloud>("trajectory", 1);
        poly_pub = this->create_publisher<sensor_msgs::msg::PointCloud>("poly", 1);
        mpc_traj_pub = this->create_publisher<sensor_msgs::msg::PointCloud>("mpc_traj", 1);

        // Timer
        timer = this->create_wall_timer(std::chrono::duration<double>(control_dt), std::bind(&MPCController::on_timer, this));
        
        RCLCPP_INFO(this->get_logger(), "MPC controller started");
        RCLCPP_INFO(this->get_logger(), "radius = %f", radius);
        RCLCPP_INFO(this->get_logger(), "cy = %f", cy);
        RCLCPP_INFO(this->get_logger(), "wheel_base = %f", wheel_base);
        RCLCPP_INFO(this->get_logger(), "max_steer_angle = %f", max_steer_angle);
        RCLCPP_INFO(this->get_logger(), "max_steer_rate = %f", max_steer_rate);
        RCLCPP_INFO(this->get_logger(), "max_velocity = %f", max_velocity);
        RCLCPP_INFO(this->get_logger(), "max_acc = %f", max_acc);
        RCLCPP_INFO(this->get_logger(), "traj_dl = %f", traj_dl);
        RCLCPP_INFO(this->get_logger(), "traj_length = %f", traj_length);
        RCLCPP_INFO(this->get_logger(), "control_dt = %f", control_dt);
        RCLCPP_INFO(this->get_logger(), "mpc_steps = %f", mpc_steps);
        RCLCPP_INFO(this->get_logger(), "mpc_dt = %f", mpc_dt);
        RCLCPP_INFO(this->get_logger(), "kcte = %f", kcte);
        RCLCPP_INFO(this->get_logger(), "kepsi = %f", kepsi);
        RCLCPP_INFO(this->get_logger(), "kev = %f", kev);
        RCLCPP_INFO(this->get_logger(), "ksteer_cost = %f", ksteer_cost);

        // counter clock
        trajectory.emplace_back(std::make_shared<trajectory::CircularSegment>(1.0 / radius,       0,           0,  1.0,           0, M_PI / 2 * radius));
        trajectory.emplace_back(std::make_shared<trajectory::LinearSegment>  (      radius,  radius,         0.0,  1.0, cy - radius));
        trajectory.emplace_back(std::make_shared<trajectory::CircularSegment>(1.0 / radius,  radius,          cy,  0.0,         1.0, M_PI / 2 * radius));
        trajectory.emplace_back(std::make_shared<trajectory::CircularSegment>(1.0 / radius,       0, radius + cy, -1.0,         0.0, M_PI / 2 * radius));
        trajectory.emplace_back(std::make_shared<trajectory::LinearSegment>  (     -radius,      cy,         0.0, -1.0, cy - radius));
        trajectory.emplace_back(std::make_shared<trajectory::CircularSegment>(1.0 / radius, -radius,      radius,  0.0,        -1.0, M_PI / 2 * radius));

        current_segment = trajectory.begin();
    }

    

    MPCController::~MPCController()
    {
        // TODO Auto-generated destructor stub
    }

    /*void MPCController::solveMPC()
    {
        ACADO::DifferentialState x, y, fi, delta, vel;
        ACADO::Control delta_rate, acc;

        const double t_start = 0;
        const double t_end = mpc_steps * mpc_dt;
        double &a0 = control_coefs[0];
        double &a1 = control_coefs[1];
        double &a2 = control_coefs[2];
        double &a3 = control_coefs[3];

        ACADO::DiscretizedDifferentialEquation f(mpc_dt);

        // discrete time system
        f << next(x) == x + vel * cos(fi) * mpc_dt;
        f << next(y) == y + vel * sin(fi) * mpc_dt;
        f << next(fi) == fi + vel * tan(delta) / wheel_base;
        f << next(delta) == delta + delta_rate * mpc_dt;
        f << next(vel) == vel + acc * mpc_dt;

        // optimal control problem
        ACADO::OCP ocp(t_start, t_end, mpc_steps);
        ocp.subjectTo(f);
        ocp.subjectTo(ACADO::AT_START, x == 0);
        ocp.subjectTo(ACADO::AT_START, y == 0);
        ocp.subjectTo(ACADO::AT_START, fi == 0);
        ocp.subjectTo(-max_acc <= acc <= max_acc);
        ocp.subjectTo(vel <= max_velocity);
        ocp.subjectTo(delta_rate <= max_steer_rate);
        ocp.subjectTo(delta <= max_steer_angle);

        ACADO::Expression cte = pow(y - a0 - a1 * x - a2 * x * x - a3 * x * x * x, 2);
        ACADO::Expression epsi = pow(fi - atan(a1 + a2 * x + a3 * x * x), 2);
        ocp.minimizeMayerTerm(cte + epsi);

        ACADO::OptimizationAlgorithm alg(ocp);
        RCLCPP_INFO_STREAM(this->get_logger(), "start solving mpc");
        alg.solve();
        RCLCPP_INFO_STREAM(this->get_logger(), "finished solving mpc");
        ACADO::VariablesGrid controls;
        alg.getControls(controls);
        controls.print();
    }*/
}
