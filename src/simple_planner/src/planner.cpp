
#include "planner.h"

#include <cstddef>
#include <queue>
#include <set>
#include <utility>
#include <chrono>
#include <thread>

namespace simple_planner
{

const MapIndex neighbors[8] = { {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};
const int8_t kObstacleValue = 100;


Planner::Planner() : Node("simple_planner")
{
  using namespace std::placeholders;

  this->declare_parameter("robot_radius", robot_radius_);

  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  options_.callback_group = callback_group_;

  obstacle_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("obstacle_map", 1);
  cost_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("cost_map", 1);
  path_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>("path", 1);
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  map_server_client_ =  this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

  // pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("ground_truth", 1, std::bind(&Planner::on_pose, this, _1), options_);
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1, std::bind(&Planner::odomCallback, this, _1), options_);
  target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("target_pose", 1, std::bind(&Planner::on_target, this, _1), options_);
}

void Planner::on_pose(const nav_msgs::msg::Odometry& odom)
{
  start_pose_ = odom.pose.pose;
}

void Planner::on_target(const geometry_msgs::msg::PoseStamped& pose)
{
  start_pose_.position.x = current_x;
  start_pose_.position.y = current_y;
  start_pose_.orientation.z = current_angle;

  std::unique_lock<std::mutex> lock(mutex_);
  abort_execution_ = true;
  lock.unlock();

  RCLCPP_INFO(this->get_logger(), (std::string("Get goal ") + std::to_string(pose.pose.position.x) + std::string(" ") + std::to_string(pose.pose.position.y)).c_str());
  RCLCPP_INFO(this->get_logger(), (std::string("Start is ") + std::to_string(start_pose_.position.x) + std::string(" ") + std::to_string(start_pose_.position.y)).c_str());

  target_pose_ = pose.pose;

  if (!update_static_map() )
  {
    RCLCPP_ERROR(this->get_logger(), "Can not receive map");
    return ;
  }
  cost_map_publisher_->publish(map_);
  increase_obstacles(ceil(robot_radius_/map_.info.resolution));
  obstacle_map_publisher_->publish(obstacle_map_);

  calculate_path();

  if (!path_msg_.points.empty()) {
    path_msg_.header.stamp = this->get_clock()->now();
    path_msg_.header.frame_id = pose.header.frame_id;
    path_publisher_->publish(path_msg_);

    double linear_speed = 6.0;
    double angular_speed = 1.0;
    abort_execution_ = false;

    for (const geometry_msgs::msg::Point32& point : path_msg_.points) {
      double target_x = point.x;      // RCLCPP_INFO_STREAM(this->get_logger(), "x = "<< point32.x <<" y = " << point32.y);
      // RCLCPP_INFO_STREAM(this->get_logger(), "i = "<< point.i <<" j = " << point.j << " g = " << node.g);
      double target_y = point.y;
      RCLCPP_INFO_STREAM(this->get_logger(), "Moving to target position: " << target_x << ", "<< target_y);
      std::unique_lock<std::mutex> lock2 = move_to_position(target_x, target_y, linear_speed, angular_speed);
      if (abort_execution_) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Aborting execution");
        // lock2.unlock();
        break;
      }
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Success!");
  }
  else {
  	RCLCPP_WARN_STREAM(this->get_logger(), "Path not found!");
  }
}

void Planner::odomCallback(const nav_msgs::msg::Odometry& odom_msg) {
    // Извлекаем текущее положение робота из сообщения одометрии
    current_x = odom_msg.pose.pose.position.x;
    current_y = odom_msg.pose.pose.position.y;

    double roll, pitch, yaw;
    tf2::Quaternion tf_orientation(odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y,
                                   odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w);

    tf2::Matrix3x3(tf_orientation).getRPY(roll, pitch, yaw);
    current_angle = yaw;
    // current_angle = odom_msg.pose.pose.orientation.z;
}

std::unique_lock<std::mutex> Planner::move_to_position(double target_x, double target_y, double linear_speed, double angular_speed) {

      while (true) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (abort_execution_) {
          return std::move(lock);
        }
        lock.unlock();

        geometry_msgs::msg::Twist cmd;
        // Рассчитываем разницу между текущим и целевым положением
        double dx = target_x - current_x;
        double dy = target_y - current_y;
        
        // Рассчитываем угол между текущим направлением робота и направлением к цели
        double desired_angle = atan2(dy, dx);
        
        // Рассчитываем разницу между текущим углом и желаемым углом
        double angle_diff = desired_angle - current_angle;
        
        if (std::abs(angle_diff) > 3.1415*2){
          angle_diff = current_angle - desired_angle;
        }
       
        if (std::abs(angle_diff) < 0.2) {
          // Вычисляем линейную и угловую скорости с использованием  П-регулятора
          cmd.linear.x = linear_speed * sqrt(dx * dx + dy * dy);
          cmd.angular.z = 0.0;
          
          cmd_vel_pub_->publish(cmd);
          if (std::abs(dx) < 0.5 && std::abs(dy) < 0.5) {
              RCLCPP_INFO(this->get_logger(), "Target position reached!");
              cmd.linear.x = 0.0;
              cmd.angular.z = 0.0;
              cmd_vel_pub_->publish(cmd);
              std::unique_lock<std::mutex> lock(mutex_);
              return std::move(lock);
          }
        } 
        else {
          cmd.linear.x = 0.0;
          cmd.angular.z = angular_speed * angle_diff;
          
          cmd_vel_pub_->publish(cmd);
        }

        // std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
  }

bool Planner::update_static_map()
{
  using namespace std::chrono_literals;

  while(!map_server_client_->wait_for_service(1s)) {
    RCLCPP_INFO(this->get_logger(), "Wait map server");
  }
  RCLCPP_INFO(this->get_logger(), "Service connected");

  auto srv_future = map_server_client_->async_send_request(std::make_shared<nav_msgs::srv::GetMap::Request>());
  auto status = srv_future.wait_for(5s);

  if (status == std::future_status::ready)
  {
    map_ = srv_future.get()->map;
    RCLCPP_INFO_STREAM(this->get_logger(), "Map received : " << map_.info.width << " " << map_.info.height);
  } 
  else if (status == std::future_status::timeout){
    RCLCPP_ERROR(this->get_logger(), "Map request timeout");
    return false;
  }
  else {
    RCLCPP_ERROR(this->get_logger(), "Map request deferred");
    return false;
  }
  return true;
}

bool Planner::indices_in_map(int i, int j)
{
  return i >= 0 && j >= 0 && i < map_.info.width && j < map_.info.height;
}

void Planner::increase_obstacles(std::size_t cells)
{
  obstacle_map_.info = map_.info;
  obstacle_map_.header = map_.header;
  obstacle_map_.data.resize(map_.data.size());
  obstacle_map_.data = map_.data;

  std::queue<MapIndex> wave;
  for (int i = 0; i < map_.info.width; ++i)
  {
    for (int j = 0; j < map_.info.height; ++j)
    {
      if (map_value(map_.data, i, j) != kObstacleValue)
      {
        continue;
      }
      // else - obstacle
      // check neighbors
      for(const auto& shift : neighbors)
      {
        int neighbor_i = i + shift.i;
        int neighbor_j = j + shift.j;
        if (!indices_in_map(neighbor_i, neighbor_j))
        {
          continue;
        }
        // if neighbor is not obstacle - add i, j to wave
        if (map_value(map_.data, neighbor_i, neighbor_j) != kObstacleValue)
        {
          wave.push({i, j});
          break;
        }
      }
    }
  }
  // RCLCPP_INFO_STREAM(this->get_logger(), "Start wave size = " << wave.size());
  for(std::size_t step = 0; step < cells; ++step)
  {
    std::queue<MapIndex> next_wave;
    while(!wave.empty()) {
      auto indices = wave.front();
      wave.pop();
      for(const auto& shift : neighbors)
      {
        auto neightbor_index = indices;
        neightbor_index.i += shift.i;
        neightbor_index.j += shift.j;
        if (!indices_in_map(neightbor_index.i, neightbor_index.j))
        {
          continue;
        }
        if (map_value(obstacle_map_.data, neightbor_index.i, neightbor_index.j) != kObstacleValue)
        {
          map_value(obstacle_map_.data, neightbor_index.i, neightbor_index.j) = kObstacleValue;
          next_wave.push(neightbor_index);
        }
      }
    } // wave empty
    std::swap(wave, next_wave);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Wave size = " << wave.size());
  }
}

double Planner::heruistic(int i, int j) {
  return 0;
}

class CompareSearchNodes {
public:
  explicit CompareSearchNodes(Planner& planner): planner_(planner) {}
  bool operator () (const MapIndex& left_index, const MapIndex& right_index) const {
  	SearchNode& left = planner_.map_value(planner_.search_map_, left_index.i, left_index.j);
  	SearchNode& right = planner_.map_value(planner_.search_map_, right_index.i, right_index.j);
    if (left.g + left.h == right.g + right.h) {
    	if (left_index.i == right_index.i) {
    		return left_index.j < right_index.j;
    	}
    	return left_index.i < right_index.i;
    }
    return left.g + left.h < right.g + right.h;
  }
private:
  Planner& planner_;
};

void Planner::calculate_path()
{
  // очищаем карту поиска
  search_map_.resize(map_.data.size());
  std::fill(search_map_.begin(), search_map_.end(), SearchNode());
  path_msg_.points.clear();

  // Здесь необходимо продолжить код для поиска пути
  std::set<MapIndex, CompareSearchNodes> queue(CompareSearchNodes(*this));
  MapIndex start_index = point_index(start_pose_.position.x, start_pose_.position.y);
  MapIndex target_index = point_index(target_pose_.position.x, target_pose_.position.y);

  if (!indices_in_map( start_index.i, start_index.j) || !indices_in_map(target_index.i, target_index.j)) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Start or target point is out of map bounds!");
    return;
  }
  
  // Создание карты волн
  std::vector<std::vector<int>> wave_map(map_.info.width, std::vector<int>(map_.info.height, -1));
  // Инициализация стартовой ячейки с нулевой стоимостью
  wave_map[start_index.i][start_index.j] = 0;
  std::queue<MapIndex> wave; 
  wave.push(start_index);
  bool found = false; // Пока ничего не найдено
  
  // Расширение волны от стартовой точки к целевой
  while (!wave.empty() && !found) {
    auto current_index = wave.front();
    wave.pop();

    /*Смысл здесь такой, что сначала перебираем
    все соседнии ячейки для расширения волны*/
    for (const auto& shift : neighbors) {
      auto neighbor_index = current_index;
      neighbor_index.i += shift.i;
      neighbor_index.j += shift.j;

      /* Потом опять проверим, что соседняя ячейка находится внутри карты 
      и не является препятствием */
      if (indices_in_map(neighbor_index.i, neighbor_index.j)) {
        if ((wave_map[neighbor_index.i][neighbor_index.j] == -1) &&
          (map_value(obstacle_map_.data, neighbor_index.i, neighbor_index.j)
          != kObstacleValue)) {
          /* По мере прохождения и добавления соседних ячеек в волну */
          wave_map[neighbor_index.i][neighbor_index.j] =
          wave_map[current_index.i][current_index.j] + 1;
          wave.push(neighbor_index);
          // Делаем проверку на достижение целевой точки
          if ((neighbor_index.i == target_index.i) && 
          (neighbor_index.j == target_index.j)) {
            found = true;
            break;
          }
        }
      }
    }
  }

  if (found) {
    // Надо сформировать путь на основе построенной волновой карте

    // Создаём путь от целевой точки к стартовой
    std::vector<MapIndex> path;
    MapIndex current_index = target_index;
    path.push_back(current_index);

    while ((current_index.i != start_index.i) &&
    (current_index.j != start_index.j)) {
      for (const auto& shift : neighbors) {
        auto neighbor_index = current_index;
        neighbor_index.i += shift.i;
        neighbor_index.j += shift.j;

        // Ищем соседнюю ячейку с предыдущей стоимостью
        if (indices_in_map(neighbor_index.i, neighbor_index.j) &&
            (wave_map[neighbor_index.i][neighbor_index.j] == wave_map[current_index.i][current_index.j] - 1)) {
              path.push_back(neighbor_index);
              current_index = neighbor_index;
              break;
            }
      }
    }
    std::reverse(path.begin(), path.end());
    for (const MapIndex& point : path) {
      geometry_msgs::msg::Point32 point32;
      point32.x = point.i * map_.info.resolution + map_.info.origin.position.x;
      point32.y = point.j * map_.info.resolution + map_.info.origin.position.y;
      auto& node = map_value(search_map_, point.i, point.j);
      path_msg_.points.push_back(point32);
      // RCLCPP_INFO_STREAM(this->get_logger(), "x = "<< point32.x <<" y = " << point32.y);
      // RCLCPP_INFO_STREAM(this->get_logger(), "i = "<< point.i <<" j = " << point.j << " g = " << node.g);
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "The path has been found!");
  }


}

} /* namespace simple_planner */
