#include <mapping/map.h>

Map::Map(): Node("map_node")
{
    map_frame_ = this->declare_parameter<std::string>("map_frame", "odom");
    map_resolution_ = this->declare_parameter("map_resolution", 0.1);
    map_width_ = this->declare_parameter("map_width", 2000);
    map_height_ = this->declare_parameter("map_height", 2000);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/simple_map", 10);
    prepareMapMessage();

    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&Map::laserCallback, this, std::placeholders::_1));
}

void Map::prepareMapMessage()
{
    map_msg_.header.frame_id = map_frame_;
    map_msg_.info.height = map_height_;
    map_msg_.info.width = map_width_;
    map_msg_.info.resolution = map_resolution_;
    map_msg_.info.origin.position.x = -map_width_ * map_resolution_ / 2.0;
    map_msg_.info.origin.position.y = -10.0;
    map_msg_.data.resize(map_height_ * map_width_, -1);
}

float Map::log2p(float p)
{
    return std::log(p / (1 - p));
}

float Map::calc_p(float l)
{
    return 1 - (1 / (1 + std::exp(l)));
}

int Map::get_map(float l)
{
    float p = calc_p(l);
    if (p < 0.4)
        return 0;
    else if (p > 0.6)
        return 100;
    else
        return 50;
}

bool Map::determineScanTransform(geometry_msgs::msg::TransformStamped& scan_transform,
                                 const rclcpp::Time& stamp,
                                 const std::string& laser_frame)
{
    try
    {
        scan_transform = tf_buffer_->lookupTransform(map_frame_, laser_frame, stamp, rclcpp::Duration::from_seconds(0.1));
    }
    catch (tf2::TransformException& ex)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to get transform: %s", ex.what());
        return false;
    }
    return true;
}

void Map::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    geometry_msgs::msg::TransformStamped scan_transform;
    const auto& laser_frame = scan->header.frame_id;
    const auto& laser_stamp = scan->header.stamp;

    if (!determineScanTransform(scan_transform, laser_stamp, laser_frame))
        return;

    map_msg_.header.stamp = laser_stamp;

    tf2::Vector3 zero_pose(0, 0, 0);
    tf2::Quaternion rotation(
        scan_transform.transform.rotation.x,
        scan_transform.transform.rotation.y,
        scan_transform.transform.rotation.z,
        scan_transform.transform.rotation.w
    );

    tf2::Vector3 translation(
        scan_transform.transform.translation.x,
        scan_transform.transform.translation.y,
        scan_transform.transform.translation.z
    );

    tf2::Transform scan_pose_transform(rotation, translation);

    tf2::Vector3 scan_pose = scan_pose_transform.inverse() * zero_pose;

    int y = (scan_pose.y() - map_msg_.info.origin.position.y) / map_resolution_;
    int x = (scan_pose.x() - map_msg_.info.origin.position.x) / map_resolution_;

    map_msg_.data[y * map_width_ + x] = 0;

    int size_ranges = scan->ranges.size();
    float curr_angle = scan->angle_min;
    float delta_angle = scan->angle_increment;
    int i = 0;

    while (curr_angle < scan->angle_max)
    {
        float range = scan->ranges[i];
        if (range > scan->range_min && range < scan->range_max)
        {
            float h = scan->range_min;
            while (h <= range)
            {
                tf2::Vector3 h_pose(h * cos(curr_angle), h * sin(curr_angle), 0);
                tf2::Vector3 h_transformed = scan_pose_transform * h_pose;

                int h_y = (h_transformed.y() - map_msg_.info.origin.position.y) / map_resolution_;
                int h_x = (h_transformed.x() - map_msg_.info.origin.position.x) / map_resolution_;

                float p = 0.5;
                if (std::abs(range - h) < 0.1)
                {
                    p = 1.0;
                }
                else if (range - h > 0.1)
                {
                    p = 0.0;
                }

                float log_prev = log2p(static_cast<float>(map_msg_.data[h_y * map_width_ + h_x]) / 100);
                float log_free = log2p(0.5);
                float log_inv = log2p(p);
                float log_ti = log_inv + log_prev - log_free;

                map_msg_.data[h_y * map_width_ + h_x] = get_map(log_ti);
                h += 0.01;
            }
        }
        i++;
        curr_angle += delta_angle;
    }

    map_pub_->publish(map_msg_);
}
