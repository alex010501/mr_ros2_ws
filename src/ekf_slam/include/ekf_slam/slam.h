#include <sstream>
#include <math.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <angles/angles.h>

#include <Eigen/Eigen>
#include <Eigen/Core>

// Размер состояния робота
const std::size_t ROBOT_STATE_SIZE = 3;
// Количество маяков
const std::size_t NUMBER_LANDMARKS = 12;
// Очень большая ковариация для маяков
const double HUGE_COVARIANCE = 1e10;

class Slam: public rclcpp::Node
{
private:
    // Подписчик на данные одометрии
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_sub;
    // Подписчик на данные лидара
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    // Публикатор положения робота
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
    // Публикатор положений маяков
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr landmark_pub[NUMBER_LANDMARKS];

    // Обработчики событий
    void on_odo(const nav_msgs::msg::Odometry &odom);
    void on_scan(const sensor_msgs::msg::LaserScan &scan);

    // Публикация результатов
    void publish_results(const std::string &frame, const rclcpp::Time &time);
    // Прогнозирование состояния
    void predict(double dt);
    // Инициализация публикаторов для маяков
    void advertize_landmark_publishers();

    // Детекция маяков по данным лидара
    void detect_landmarks(const sensor_msgs::msg::LaserScan &scan);
    // Добавление информации о маяке
    void add_landmark(const sensor_msgs::msg::LaserScan &scan, std::size_t start, std::size_t finish);
    // Ассоциация измерения с маяком
    int associate_measurement(const Eigen::Vector2d &landmark_measurement);
    // Добавление информации о маяке в состояние
    int add_landmark_to_state(int measurementIndex);
    // Коррекция состояния по измерению маяка
    void correct(int landmarkIndex, int measurementIndex);
    // Публикация трансформации
    void publish_transform(const std_msgs::msg::Header &scan_header);

    // Линейная и угловая скорости
    double v = 0;
    double w = 0;

    // Вектор состояния
    Eigen::VectorXd X;
    // Линеаризованная матрица системы
    Eigen::Matrix3d A;
    // Матрица ковариации ошибок оценок
    Eigen::MatrixXd P;
    // Матрица ковариации ошибок измерения
    Eigen::Matrix2d Q;
    // Матрица ковариации возмущений системы
    Eigen::Matrix3d R;

    // Время последнего измерения
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    rclcpp::Time last_time;

    // Количество обнаруженных маяков
    std::size_t landmarks_found_quantity = 0;

    // Вектор для хранения новых маяков
    std::vector<Eigen::Vector2d> new_landmarks;
    // Вектор для хранения измерений новых маяков
    std::vector<Eigen::Vector2d> new_landmarks_measurement;

    // Публикатор трансформаций
    std::shared_ptr<tf2_ros::TransformBroadcaster> br;

    // Имя фрейма карты (параметр ROS)
    const std::string map_frame;
    // Радиус маяка
    double feature_rad;

public:
    Slam(const std::string &ns = "ekf_slam");
};
