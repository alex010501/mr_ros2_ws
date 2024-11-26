#ifndef SRC_SIMPLE_PLANNER_SRC_PLANNER_H_
#define SRC_SIMPLE_PLANNER_SRC_PLANNER_H_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <shared_mutex>
#include <limits>

namespace simple_planner
{

 // структура, описывающая узел поиска
struct SearchNode{
 enum State {
    CLOSE, OPEN, UNDEFINED
  };
   // состояние узла
   State state = UNDEFINED;
   // значение функции оптимальной стоимости достижения узла
   double g = std::numeric_limits<double>::max();
   // значение функции эвристики
   double h = 0;
};

struct MapIndex {
  int i;
  int j;
};


class Planner : public rclcpp::Node
{
public:
  Planner();

private:
  friend class CompareSearchNodes;
  // обновление положения робота
  void on_pose(const nav_msgs::msg::Odometry& odom);
  // колбек целевой точки
  void on_target(const geometry_msgs::msg::PoseStamped& pose);
  // функция обновления карты (map_)
  bool update_static_map();
  // функция расширения карты препятствий (obstacle_map_)
  void increase_obstacles(std::size_t cells);
  // функция вычисления пути в заданную точку
  void calculate_path();

  double heruistic(int i, int j);

  std::unique_lock<std::mutex> move_to_position(double target_x, double target_y, double linear_speed, double angular_speed);

  void odomCallback(const nav_msgs::msg::Odometry& odom_msg);

  // функции для работы с картами и индексами
  // Проверка индексов на нахождение в карте
  bool indices_in_map(int i, int j);
  // Возвращает ссылку на значение в карте
  template <class T>
  T& map_value(std::vector<T>& data, int i, int j)
  {
    int index = j * map_.info.width + i;
    return data[index];
  }

  MapIndex point_index(double x, double y) {
    return {
     static_cast<int>(floor((x - map_.info.origin.position.x)/ map_.info.resolution)),
     static_cast<int>(floor((y - map_.info.origin.position.y)/ map_.info.resolution))
  };
  }

private:
  nav_msgs::msg::OccupancyGrid map_;
  nav_msgs::msg::OccupancyGrid obstacle_map_;
  nav_msgs::msg::OccupancyGrid cost_map_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacle_map_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_map_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr map_server_client_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  geometry_msgs::msg::Pose start_pose_;
  geometry_msgs::msg::Pose target_pose_;

  sensor_msgs::msg::PointCloud path_msg_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::SubscriptionOptions options_;

  double robot_radius_ = 0.5;

  std::vector<SearchNode> search_map_;

  double current_x;
  double current_y;
  double current_angle;
  bool abort_execution_ = false;
  std::mutex mutex_;
};

} /* namespace simple_planner */

#endif /* SRC_SIMPLE_PLANNER_SRC_PLANNER_H_ */
