#include <cmath>
#include <set>
#include <queue>
#include <utility>

#include <simple_planner/planner.h>

namespace simple_planner
{

    const MapIndex neighbors[8] = {
        {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};
    const int8_t kObstacleValue = 100;

    Planner::Planner(const rclcpp::NodeOptions &options)
        : Node("simple_planner", options), robot_radius_(declare_parameter("robot_radius", 0.5))
    {
        obstacle_map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("obstacle_map", 10);
        path_publisher_ = create_publisher<sensor_msgs::msg::PointCloud>("path", 10);
        map_server_client_ = create_client<nav_msgs::srv::GetMap>("/static_map");

        pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "ground_truth", 10, std::bind(&Planner::on_pose, this, std::placeholders::_1));

        target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "target_pose", 10, std::bind(&Planner::on_target, this, std::placeholders::_1));
    }

    void Planner::on_pose(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        start_pose_ = odom->pose.pose;
    }

    void Planner::on_target(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
    {
        RCLCPP_INFO(get_logger(), "Received goal: [%f, %f]", pose->pose.position.x, pose->pose.position.y);
        target_pose_ = pose->pose;

        if (!update_static_map())
        {
            RCLCPP_ERROR(get_logger(), "Failed to update map");
            return;
        }

        increase_obstacles(static_cast<std::size_t>(std::ceil(robot_radius_ / map_.info.resolution)));
        obstacle_map_publisher_->publish(obstacle_map_);
        calculate_path();

        if (!path_msg_.points.empty())
        {
            path_msg_.header.stamp = now();
            path_msg_.header.frame_id = pose->header.frame_id;
            path_publisher_->publish(path_msg_);
        }
        else
        {
            RCLCPP_WARN(get_logger(), "No path found!");
        }
    }

    bool Planner::update_static_map()
    {
        auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
        auto response = map_server_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), response) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "Failed to call map server");
            return false;
        }
        map_ = response.get()->map;
        RCLCPP_INFO(get_logger(), "Map updated: [%d x %d]", map_.info.width, map_.info.height);
        return true;
    }

    bool Planner::indices_in_map(int i, int j)
    {
        return i >= 0 && j >= 0 && i < static_cast<int>(map_.info.width) && j < static_cast<int>(map_.info.height);
    }

    void Planner::increase_obstacles(std::size_t cells)
    {
        obstacle_map_.info = map_.info;
        obstacle_map_.header = map_.header;
        obstacle_map_.data = map_.data;

        std::queue<MapIndex> wave;
        for (int i = 0; i < static_cast<int>(map_.info.width); ++i)
        {
            for (int j = 0; j < static_cast<int>(map_.info.height); ++j)
            {
                if (map_value(map_.data, i, j) != kObstacleValue)
                {
                    continue;
                }

                for (const auto &shift : neighbors)
                {
                    int neighbor_i = i + shift.i;
                    int neighbor_j = j + shift.j;
                    if (!indices_in_map(neighbor_i, neighbor_j))
                    {
                        continue;
                    }
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
                for (const auto &shift : neighbors)
                {
                    auto neighbor_index = indices;
                    neighbor_index.i += shift.i;
                    neighbor_index.j += shift.j;
                    if (!indices_in_map(neighbor_index.i, neighbor_index.j))
                    {
                        continue;
                    }
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

    double Planner::heuristic(int i, int j)
    {
        int target_i = point_index(target_pose_.position.x, target_pose_.position.y).i;
        int target_j = point_index(target_pose_.position.x, target_pose_.position.y).j;
        return std::hypot(target_i - i, target_j - j);
    }

    class CompareSearchNodes
    {
    public:
        explicit CompareSearchNodes(Planner &planner) : planner_(planner) {}
        bool operator()(const MapIndex &left_index, const MapIndex &right_index) const
        {
            SearchNode &left = planner_.map_value(planner_.search_map_, left_index.i, left_index.j);
            SearchNode &right = planner_.map_value(planner_.search_map_, right_index.i, right_index.j);
            if (left.g + left.h == right.g + right.h)
            {
                if (left_index.i == right_index.i)
                {
                    return left_index.j < right_index.j;
                }
                return left_index.i < right_index.i;
            }
            return left.g + left.h < right.g + right.h;
        }

    private:
        Planner &planner_;
    };

    void Planner::calculate_path()
    {
        search_map_.resize(map_.data.size());
        std::fill(search_map_.begin(), search_map_.end(), SearchNode());
        path_msg_.points.clear();

        // Очередь для узлов поиска
        std::set<MapIndex, CompareSearchNodes> queue(CompareSearchNodes(*this));
        MapIndex start_index = point_index(start_pose_.position.x, start_pose_.position.y);
        SearchNode &start = map_value(search_map_, start_index.i, start_index.j);
        start.g = 0;
        start.h = heuristic(start_index.i, start_index.j);
        start.state = SearchNode::OPEN;

        auto &start_obstacle_value = map_value(obstacle_map_.data, start_index.i, start_index.j);
        if (start_obstacle_value == kObstacleValue)
        {
            RCLCPP_WARN(get_logger(), "Start is in obstacle!");
            return;
        }
        queue.insert(start_index);

        MapIndex target_index = point_index(target_pose_.position.x, target_pose_.position.y);
        bool found = false;

        // Основной цикл A*
        while (!queue.empty())
        {
            auto node_index_iter = queue.begin();
            auto node_index = *node_index_iter;
            auto &node = map_value(search_map_, node_index.i, node_index.j);
            node.state = SearchNode::CLOSE;
            queue.erase(node_index_iter);

            // Если мы достигли целевой точки
            if (node_index.i == target_index.i && node_index.j == target_index.j)
            {
                found = true;
                break;
            }

            // Обработка соседей
            for (const auto &shift : neighbors)
            {
                MapIndex neighbor_index{node_index.i + shift.i, node_index.j + shift.j};

                if (!indices_in_map(neighbor_index.i, neighbor_index.j))
                    continue;

                auto &neighbor = map_value(search_map_, neighbor_index.i, neighbor_index.j);
                if (neighbor.state == SearchNode::CLOSE)
                    continue;

                double tentative_g = node.g + std::hypot(shift.i, shift.j) * map_.info.resolution;

                if (map_value(obstacle_map_.data, neighbor_index.i, neighbor_index.j) == kObstacleValue)
                    continue;

                if (neighbor.state != SearchNode::OPEN || tentative_g < neighbor.g)
                {
                    neighbor.g = tentative_g;
                    neighbor.h = heuristic(neighbor_index.i, neighbor_index.j);
                    neighbor.state = SearchNode::OPEN;

                    queue.insert(neighbor_index);
                }
            }
        }

        // Восстановление пути, если найден
        if (found)
        {
            int i = target_index.i;
            int j = target_index.j;
            geometry_msgs::msg::Point32 p;

            while (i != start_index.i || j != start_index.j)
            {
                p.x = i * map_.info.resolution + map_.info.origin.position.x;
                p.y = j * map_.info.resolution + map_.info.origin.position.y;
                path_msg_.points.push_back(p);

                auto &current_node = map_value(search_map_, i, j);
                double min_g = current_node.g;

                for (const auto &shift : neighbors)
                {
                    int neighbor_i = i + shift.i;
                    int neighbor_j = j + shift.j;

                    if (!indices_in_map(neighbor_i, neighbor_j))
                        continue;

                    auto &neighbor_node = map_value(search_map_, neighbor_i, neighbor_j);
                    if (neighbor_node.state == SearchNode::CLOSE && neighbor_node.g < min_g)
                    {
                        min_g = neighbor_node.g;
                        i = neighbor_i;
                        j = neighbor_j;
                    }
                }
            }

            // Добавляем начальную точку в путь
            p.x = start_index.i * map_.info.resolution + map_.info.origin.position.x;
            p.y = start_index.j * map_.info.resolution + map_.info.origin.position.y;
            path_msg_.points.push_back(p);

            std::reverse(path_msg_.points.begin(), path_msg_.points.end());
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Path not found!");
        }
    }

} // namespace simple_planner
