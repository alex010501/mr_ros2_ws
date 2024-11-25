#include <simple_planner/planner.h>

#include <queue>
#include <set>
#include <cmath>
#include <utility>

namespace simple_planner {

const MapIndex neighbors[8] = {
    {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}
};
const int8_t kObstacleValue = 100;

Planner::Planner(ros::NodeHandle& nh) : nh_(nh)
{
    while (!map_server_client_.waitForExistence(ros::Duration(1)))
        ROS_INFO_STREAM("Waiting for map server...");
    ROS_INFO_STREAM("Map server connected.");
}

void Planner::on_pose(const nav_msgs::Odometry& odom)
{
    start_pose_ = odom.pose.pose;
}

void Planner::on_target(const geometry_msgs::PoseStamped& pose)
{
    ROS_INFO_STREAM("Goal received: " << pose.pose.position.x << ", " << pose.pose.position.y);
    ROS_INFO_STREAM("Start position: " << start_pose_.position.x << ", " << start_pose_.position.y);
    target_pose_ = pose.pose;

    if (!update_static_map())
    {
        ROS_ERROR_STREAM("Cannot receive map.");
        return;
    }

    increase_obstacles(std::ceil(robot_radius_ / map_.info.resolution));
    obstacle_map_publisher_.publish(obstacle_map_);

    calculate_path();

    if (!path_msg_.points.empty())
    {
        path_msg_.header.stamp = ros::Time::now();
        path_msg_.header.frame_id = pose.header.frame_id;
        path_publisher_.publish(path_msg_);
    }
    else
        ROS_WARN_STREAM("Path not found!");
}

bool Planner::update_static_map()
{
    nav_msgs::GetMap service;
    if (!map_server_client_.call(service))
    {
        ROS_ERROR_STREAM("Failed to receive a map.");
        return false;
    }
    map_ = service.response.map;
    ROS_INFO_STREAM("Map received: " << map_.info.width << " x " << map_.info.height);
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
                continue;

            for (const auto& shift : neighbors)
            {
                int neighbor_i = i + shift.i;
                int neighbor_j = j + shift.j;
                if (!indices_in_map(neighbor_i, neighbor_j))
                    continue;

                if (map_value(map_.data, neighbor_i, neighbor_j) != kObstacleValue)
                {
                    wave.push({i, j});
                    break;
                }
            }
        }
    }

    for (std::size_t step = 0; step < cells; ++step)
    {
        std::queue<MapIndex> next_wave;
        while (!wave.empty())
        {
            auto indices = wave.front();
            wave.pop();
            for (const auto& shift : neighbors)
            {
                auto neighbor_index = indices;
                neighbor_index.i += shift.i;
                neighbor_index.j += shift.j;
                if (!indices_in_map(neighbor_index.i, neighbor_index.j))
                    continue;
                
                if (map_value(obstacle_map_.data, neighbor_index.i, neighbor_index.j) != kObstacleValue)
                {
                    map_value(obstacle_map_.data, neighbor_index.i, neighbor_index.j) = kObstacleValue;
                    next_wave.push(neighbor_index);
                }
            }
        }
        wave.swap(next_wave);
    }
}

double Planner::heruistic(int i, int j)
{
    // Манхэттенское расстояние как эвристика
    double dx = std::abs(target_pose_.position.x - (i * map_.info.resolution + map_.info.origin.position.x));
    double dy = std::abs(target_pose_.position.y - (j * map_.info.resolution + map_.info.origin.position.y));
    return dx + dy;
}

void Planner::calculate_path()
{
    search_map_.resize(map_.data.size());
    std::fill(search_map_.begin(), search_map_.end(), SearchNode());
    path_msg_.points.clear();

    std::set<MapIndex, CompareSearchNodes> queue(CompareSearchNodes(*this));
    MapIndex start_index = point_index(start_pose_.position.x, start_pose_.position.y);
    MapIndex target_index = point_index(target_pose_.position.x, target_pose_.position.y);

    if (map_value(obstacle_map_.data, start_index.i, start_index.j) == kObstacleValue)
    {
        ROS_WARN_STREAM("Start is in an obstacle!");
        return;
    }

    auto& start_node = map_value(search_map_, start_index.i, start_index.j);
    start_node.g = 0;
    start_node.h = heruistic(start_index.i, start_index.j);
    start_node.state = SearchNode::OPEN;

    queue.insert(start_index);

    bool found = false;

    while (!queue.empty())
    {
        auto current_index = *queue.begin();
        queue.erase(queue.begin());

        auto& current_node = map_value(search_map_, current_index.i, current_index.j);
        current_node.state = SearchNode::CLOSE;

        if (current_index.i == target_index.i && current_index.j == target_index.j)
        {
            found = true;
            break;
        }

        for (const auto& neighbor_shift : neighbors)
        {
            MapIndex neighbor_index = {current_index.i + neighbor_shift.i, current_index.j + neighbor_shift.j};

            if (!indices_in_map(neighbor_index.i, neighbor_index.j))
                continue;

            if (map_value(obstacle_map_.data, neighbor_index.i, neighbor_index.j) == kObstacleValue)
                continue;

            auto& neighbor_node = map_value(search_map_, neighbor_index.i, neighbor_index.j);
            if (neighbor_node.state == SearchNode::CLOSE)
                continue;

            double tentative_g = current_node.g + map_.info.resolution;

            if (neighbor_node.state == SearchNode::UNDEFINED || tentative_g < neighbor_node.g)
            {
                neighbor_node.g = tentative_g;
                neighbor_node.h = heruistic(neighbor_index.i, neighbor_index.j);
                neighbor_node.state = SearchNode::OPEN;
                queue.insert(neighbor_index);
            }
        }
    }

    if (found)
    {
        geometry_msgs::Point32 point;
        for (MapIndex index = target_index;
             index.i != start_index.i || index.j != start_index.j;
             index = map_value(search_map_, index.i, index.j).parent_index)
        {
            point.x = index.i * map_.info.resolution + map_.info.origin.position.x;
            point.y = index.j * map_.info.resolution + map_.info.origin.position.y;
            path_msg_.points.push_back(point);
        }
        std::reverse(path_msg_.points.begin(), path_msg_.points.end());
    }
    else
        ROS_WARN_STREAM("Path not found.");
    
}

}
