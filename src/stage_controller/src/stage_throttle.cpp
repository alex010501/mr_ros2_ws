#include <algorithm>
#include <chrono>
#include <random>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rclcpp/rclcpp.hpp>

double car_length = 1.5;

double steering = 0;
double cmd_steering = 0.0;
double max_steering_rate = 1.0;
double max_steering = 0.5;

double throttle = 0;
double velocity = 0;
double cmd_throttle = 0;
double max_throttle_rate = 200.0;
double max_throttle = 100;
double max_velocity = 20;

double kMass = 500;
double kFriction = 1.0;
double kWindFriction = 0.01;
double kBrake = 3.0;
double kThrottle = 2.5;
double kVelExp = 0.8;
double velocity_noise = 0.0;

rclcpp::Time last_timer_time{};

std::default_random_engine noise_generator;
std::normal_distribution<double> noise_distr;

void on_steering(const std_msgs::msg::Float32 &msg)
{
    cmd_steering = msg.data;
}

void on_throttle(const std_msgs::msg::Float32 &msg)
{
    cmd_throttle = msg.data;
}

double clamp(double cmd, double max_value)
{
    return copysign(std::min(std::abs(cmd), max_value), cmd);
}

double clamp(double value, double cmd, double max_value, double max_rate, double dt)
{
    cmd = clamp(cmd, max_value);
    auto diff = (cmd - value);
    diff = clamp(diff, max_rate * dt);
    return value + diff;
}

void publish(rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub, double data)
{
    std_msgs::msg::Float32 msg;
    msg.data = data;
    pub->publish(msg);
}

double acc_from_throttle(double dt, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr real_throttle_pub_ptr)
{
    throttle = clamp(throttle, cmd_throttle, max_throttle, max_throttle_rate, dt);
    publish(real_throttle_pub_ptr, throttle);

    double throttle_force = throttle > 0 ? kThrottle * throttle * exp(-velocity * kVelExp) : kBrake * throttle;
    double acc = 1.0 / kMass * (throttle_force - velocity * velocity * kWindFriction - kFriction * velocity);
    return acc;
}

void on_timer(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_ptr,
              rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr real_steer_pub_ptr,
              rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr real_throttle_pub_ptr,
              std::shared_ptr<rclcpp::Node> node)
{
    auto t = node->get_clock()->now();
    auto dt = (t - last_timer_time).seconds();
    last_timer_time = t;
    geometry_msgs::msg::Twist cmd;

    velocity = std::max(0.0, clamp(velocity + acc_from_throttle(dt, real_throttle_pub_ptr) * dt, max_velocity));
    double send_velocity = velocity;
    if (velocity_noise != 0.0 && std::abs(velocity) > 0.01)
        send_velocity = std::max<double>(0.0, send_velocity + noise_distr(noise_generator));

    steering = clamp(steering, cmd_steering, max_steering, max_steering_rate, dt);
    publish(real_steer_pub_ptr, steering);
    cmd.linear.x = send_velocity;
    cmd.angular.z = send_velocity * tan(steering) / car_length;
    twist_pub_ptr->publish(cmd);
}

int main(int argc, char *argv[])
{
    using namespace std::placeholders;
    using namespace std::chrono_literals;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("stage_throttle_controller");

    node->declare_parameter("length", 1.5);
    node->declare_parameter("max_steering", 0.5);
    node->declare_parameter("max_steering_rate", 1.0);
    node->declare_parameter("max_throttle_rate", 200.0);
    node->declare_parameter("max_throttle", 100.0);
    node->declare_parameter("max_velocity", 20.0);
    node->declare_parameter("velocity_noise", 0.0);
    node->declare_parameter("/use_sim_time", false);

    if (velocity_noise != 0.0)
        noise_distr = std::normal_distribution<double>(0.0, velocity_noise);

    node->declare_parameter("mass", 500.0);
    node->declare_parameter("friction", 1.0);
    node->declare_parameter("wind_friction", 0.01);
    node->declare_parameter("brake", 3.0);
    node->declare_parameter("throttle", 2.5);
    node->declare_parameter("exp", 0.8);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_ptr = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steer_sub = node->create_subscription<std_msgs::msg::Float32>("steering", 1, on_steering);
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr throttle_sub = node->create_subscription<std_msgs::msg::Float32>("throttle", 1, on_throttle);

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr real_throttle_pub_ptr = node->create_publisher<std_msgs::msg::Float32>("realized_throttle", 1);
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr real_steer_pub_ptr = node->create_publisher<std_msgs::msg::Float32>("realized_steering", 1);

    if (node->get_parameter("/use_sim_time").as_bool())
        while (rclcpp::ok())
        {
            rclcpp::spin_some(node);

            last_timer_time = node->get_clock()->now();
            if (last_timer_time.seconds())
                break;
        }

    std::function<void()> tim_callback = std::bind(on_timer, twist_pub_ptr, real_steer_pub_ptr, real_throttle_pub_ptr, node);
    auto timer = node->create_wall_timer(50ms, tim_callback);
    last_timer_time = node->get_clock()->now();
    rclcpp::spin(node);
    return 0;
}