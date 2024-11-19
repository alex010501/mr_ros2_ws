#include "planner.h"
//http://docs.ros.org/en/jade/api/tf/html/c++/transform__datatypes_8h.html
#include <tf/transform_datatypes.h>
//Включает заголовок стандартной <библиотеки C stddef.h> и добавляет связанные имена в std пространство имен
#include <cstddef>
#include <queue>
#include <set>

//C++ includes a variety of utility libraries that provide functionality ranging from bit-counting to partial function application.
#include <utility>

namespace simple_planner
{

const MapIndex neighbors[8] = { {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};
const int8_t kObstacleValue = 100;


Planner::Planner(ros::NodeHandle& nh) :
 nh_(nh)
{
  while(!map_server_client_.waitForExistence(ros::Duration(1))) {
    ROS_INFO_STREAM("Wait map server");
  }
  ROS_INFO_STREAM("Service connected");
}
// обновление положения робота
void Planner::on_pose(const nav_msgs::Odometry& odom)
{
  start_pose_ = odom.pose.pose;
}

void Planner::on_target(const geometry_msgs::PoseStamped& pose)
{
  ROS_INFO_STREAM("Get goal " << pose.pose.position.x << " " << pose.pose.position.y);
  ROS_INFO_STREAM("Start is " << start_pose_.position.x << " " << start_pose_.position.y);
  target_pose_ = pose.pose;

  if (!update_static_map() )
  {
    ROS_ERROR_STREAM("Can not receive map");
    return ;
  }

  increase_obstacles(ceil(robot_radius_/map_.info.resolution));
  //std::cout<<"map reslotuion"<<map_.info.resolution<<std::endl;
  obstacle_map_publisher_.publish(obstacle_map_);

  calculate_path();

  if (!path_msg_.points.empty()) {
    path_msg_.header.stamp = ros::Time::now();
    path_msg_.header.frame_id = pose.header.frame_id;
    path_publisher_.publish(path_msg_);

    double linear_speed = 2.0;
    double angular_speed = 2.0;

    for (const geometry_msgs::Point32& point : path_msg_.points) {
      double target_x = point.x;
      double target_y = point.y;
      ROS_INFO_STREAM("Moving to target position: " << target_x << ", "<< target_y);
      move_to_position(target_x, target_y, linear_speed, angular_speed);
    }
    ROS_INFO_STREAM("Success!");
  } else {
  	ROS_WARN_STREAM("Path not found!");
  }
}

void Planner::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    // Извлекаем текущее положение робота из сообщения одометрии
    current_x = odom_msg->pose.pose.position.x;
    current_y = odom_msg->pose.pose.position.y;

    // Извлекаем текущий угол робота (ориентацию) из сообщения одометрии
    current_angle = odom_msg->pose.pose.orientation.z;
}

void Planner::move_to_position(double target_x, double target_y, double linear_speed, double angular_speed) {
    int argc = 0;
    char* argv[] = {nullptr};
    ros::init(argc, argv, "move_to_position");
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Rate rate(10);  // 10 Hz
    

    while (ros::ok()) {
        geometry_msgs::Twist cmd;
        
        // Рассчитываем разницу между текущим и целевым положением
        double dx = target_x - current_x;
        double dy = target_y - current_y;
        
        // Рассчитываем угол между текущим направлением робота и направлением к цели
        double desired_angle = atan2(dy, dx);
        
        // Рассчитываем разницу между текущим углом и желаемым углом
        double angle_diff = desired_angle - current_angle;
        
        if (std::abs(angle_diff) > 3.1415*2-0.2*2){
          angle_diff = current_angle - desired_angle;
        }

        current_x = start_pose_.position.x;
        current_y = start_pose_.position.y;

        geometry_msgs::Quaternion orientation = start_pose_.orientation;
        // Конвертируем кватернион в угол Эйлера (yaw)
        double roll, pitch, yaw;
        tf::Quaternion tf_orientation;
        tf::quaternionMsgToTF(orientation, tf_orientation);
        tf::Matrix3x3(tf_orientation).getRPY(roll, pitch, yaw);
        current_angle = yaw;

        ROS_INFO_STREAM("current_x = " << current_x << " current_y = " << current_y);
        ROS_INFO_STREAM("current_angle = " << current_angle);
        ROS_INFO_STREAM("angle_diff = " << angle_diff);
        if (std::abs(angle_diff) < 0.2){
          // Вычисляем линейную и угловую скорости с использованием  П-регулятора
          cmd.linear.x = linear_speed * sqrt(dx * dx + dy * dy);
          cmd.angular.z = 0.0;
          
          cmd_vel_pub.publish(cmd);
          if (std::abs(dx) < 0.5 && std::abs(dy) < 0.5) {
              ROS_INFO("Target position reached!");
              cmd.linear.x = 0.0;
              cmd.angular.z = 0.0;
          
              cmd_vel_pub.publish(cmd);
              break;
          }
        } else {
          cmd.linear.x = 0.0;
          cmd.angular.z = angular_speed * angle_diff;
          
          cmd_vel_pub.publish(cmd);
        }


        rate.sleep();
        ros::spinOnce(); // Обработка коллбэков
    }

}

bool Planner::update_static_map()
{
  nav_msgs::GetMap service;
  if (!map_server_client_.call(service))
  {
    ROS_ERROR_STREAM("Failed to receive a map");
    return false;
  }
  map_ = service.response.map;
  ROS_INFO_STREAM("Map received : " << map_.info.width << " " << map_.info.height);
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
  ROS_INFO_STREAM("Start wave size = " << wave.size());
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
    ROS_INFO_STREAM("Wave size = " << wave.size());
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
  // Очищаем карту поиска
  search_map_.resize(map_.data.size());
  std::fill(search_map_.begin(), search_map_.end(), SearchNode());
  path_msg_.points.clear();

  // Получение индексов для стартовой и целевой точек на карте
  MapIndex start_index = point_index(start_pose_.position.x, start_pose_.position.y);
  MapIndex target_index = point_index(target_pose_.position.x, target_pose_.position.y);
  
  // Проверка, что стартовая и целевая точки находятся внутри границ карты
  if (!indices_in_map( start_index.i, start_index.j) || !indices_in_map(target_index.i, target_index.j)) {
    ROS_WARN_STREAM("Start or target point is out of map bounds!");
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
      geometry_msgs::Point32 point32;
      point32.x = point.i * map_.info.resolution + map_.info.origin.position.x;
      point32.y = point.j * map_.info.resolution + map_.info.origin.position.y;
      auto& node = map_value(search_map_, point.i, point.j);
      path_msg_.points.push_back(point32);
      ROS_INFO_STREAM("x = "<< point32.x <<" y = " << point32.y);
      ROS_INFO_STREAM("i = "<< point.i <<" j = " << point.j << " g = " << node.g);
    }
    ROS_INFO_STREAM("The path has been found!");
  }
}
}
