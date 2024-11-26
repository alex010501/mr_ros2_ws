#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>


//#include <tf/transform_publisher.h>



class Matcher : public rclcpp::Node
{
public:
  Matcher();

private:
  void on_laser_scan(const sensor_msgs::msg::LaserScan& scan);
  // ищем особенные точки в скане
  void detect_features(const sensor_msgs::msg::LaserScan& scan);
  // предсказываем положение новых особенных точек относительно опорных 
  // по предыдущему инкрементальному трансформуб считая скорость меняется незначительно
  void predict_features_poses();
  // ищем пары ближайших особенных точек
  void find_feature_pairs();
  // функция определения трансформов по парам особенных точек
  void find_transform();
  // публикуем результаты
  void publish_transform(const std_msgs::msg::Header& header);
  // публикуем особенные точки из текущего скана
  void publish_features(const std_msgs::msg::Header& header);
  // обновляем вектор опорных особенных точек
  void update_base_features();
  // добавляет новую feature на которую папали лучи скана от start до finish включительно
  void add_feature(const sensor_msgs::msg::LaserScan& scan, std::size_t start, std::size_t finish);

  double feature_rad;
  std::string map_frame;

  // features from base scan набор опорных особенных точек в СК карты
  std::vector<Eigen::Vector2d> base_features;
  // features from new scan набор особенных точек из нового скана в СК лазерного дальномера
  std::vector<Eigen::Vector2d> new_features;
  // features from new scan transfromed to base scan according to interpolation
  // набор особенных точек нового скана переведенных в СК карты
  // с учетом предсказания - считаем что робот продолжит движение как на предыдущем шаге
  std::vector<Eigen::Vector2d> predicted_features;
  // indexes of corresponding new features for every base feature
  // вектор индексов особенных точек нового скана для каждой опорной особенной точки
  // либо -1 - если нет таковой
  std::vector<int> feature_pair_indices;
  // transform from current scan to map
  // преобразование переводящее текущий скан в карту, обновляется в результате каждого шага
  Eigen::Isometry2d transform = Eigen::Isometry2d::Identity();
  // transform from current scan to previous
  // инкрементальное преобразование за последний шаг в СК карты
  Eigen::Isometry2d incremental_transform = Eigen::Isometry2d::Identity();
  double distancePassed;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // отметка времени предыдущего скана
  rclcpp::Time last_stamp;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr feature_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odo_pub_;
};

