#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <tf/transform_broadcaster.h>

// Размер состояния робота
const std::size_t ROBOT_STATE_SIZE = 3;
// Количество маяков
const std::size_t NUMBER_LANDMARKS = 12;
// Очень большая ковариация для маяков
const double HUGE_COVARIANCE = 1e10;

class Slam {
private:
  // Узел ROS
  ros::NodeHandle nh;
  // Подписчик на данные одометрии
  ros::Subscriber odo_sub;
  // Подписчик на данные лидара
  ros::Subscriber scan_sub;
  // Публикатор положения робота
  ros::Publisher pose_pub;
  // Публикатор положений маяков
  ros::Publisher landmark_pub[NUMBER_LANDMARKS];
  
  // Обработчики событий
  void on_odo(const nav_msgs::Odometry& odom);
  void on_scan(const sensor_msgs::LaserScan& scan);
  
  // Публикация результатов
  void publish_results(const std::string& frame, const ros::Time& time);
  // Прогнозирование состояния
  void predict(double dt);
  // Инициализация публикаторов для маяков
  void advertize_landmark_publishers();
  
  // Детекция маяков по данным лидара
  void detect_landmarks(const sensor_msgs::LaserScan& scan);
  // Добавление информации о маяке
  void add_landmark(const sensor_msgs::LaserScan& scan, std::size_t start, std::size_t finish);
  // Ассоциация измерения с маяком
  int associate_measurement(const Eigen::Vector2d& landmark_measurement);
  // Добавление информации о маяке в состояние
  int add_landmark_to_state(int measurementIndex);
  // Коррекция состояния по измерению маяка
  void correct(int landmarkIndex, int measurementIndex);
  // Публикация трансформации
  void publish_transform(const std_msgs::Header& scan_header);

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
  ros::Time last_time = ros::Time::now();

  // Количество обнаруженных маяков
  std::size_t landmarks_found_quantity = 0;

  // Вектор для хранения новых маяков
  std::vector<Eigen::Vector2d> new_landmarks;
  // Вектор для хранения измерений новых маяков
  std::vector<Eigen::Vector2d> new_landmarks_measurement;

  // Публикатор трансформаций
  tf::TransformBroadcaster br;

  // Имя фрейма карты (параметр ROS)
  const std::string map_frame = nh.param<std::string>("map_frame", "map");
  // Радиус маяка
  double feature_rad = nh.param<double>("feature_radius", 1.0);

public:
  Slam();
};
