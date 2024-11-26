#include "matcher.h"
#include <tf2/transform_datatypes.h>

Matcher::Matcher() : Node("matcher"),
				 	 feature_rad(this->declare_parameter("feature_radius", 0.55)),
				 	 map_frame(this->declare_parameter("map_frame", "map"))
{
	using std::placeholders::_1;
	laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/base_scan", 1, std::bind(&Matcher::on_laser_scan, this, _1));

	rclcpp::QoS oLatching = rclcpp::QoS(rclcpp::KeepLast(1));
	oLatching.transient_local();

	feature_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("features", oLatching);
	odo_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odo", 1);
	last_stamp = this->get_clock()->now();
	tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void Matcher::on_laser_scan(const sensor_msgs::msg::LaserScan& scan)
{
	RCLCPP_INFO_STREAM(this->get_logger(), "on laser scan");
	detect_features(scan);
	publish_features(scan.header);
	predict_features_poses();
	find_feature_pairs();
	find_transform();
	update_base_features();
	publish_transform(scan.header);
}


void Matcher::publish_features(const std_msgs::msg::Header& header)
{
	visualization_msgs::msg::Marker marker;
	marker.header = header;
	marker.color.a = 1.0;
	marker.color.g = 1.0;
	marker.type = visualization_msgs::msg::Marker::POINTS;
	marker.points.resize(new_features.size());

	for (std::size_t i = 0; i < marker.points.size(); ++i) {
		marker.points[i].x = new_features[i].x();
		marker.points[i].y = new_features[i].y();
		marker.points[i].z = 0;
	}

	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.id = 100;
	marker.ns = "features";
	feature_pub_->publish(marker);
}

void Matcher::add_feature(const sensor_msgs::msg::LaserScan& scan, std::size_t start, std::size_t finish)
{
	if (finish - start < 2) {
		return;
	}
	RCLCPP_INFO_STREAM(this->get_logger(), "Add feature between " << start << " " << finish);
	double x = 0.0, y = 0.0;
	auto nearestPoint = [&]() {
		double minDist = 1e100;
		size_t minIdx;
		for (size_t idx = start; idx < finish; ++idx) {
			if (scan.ranges[idx] < minDist) {
				minDist = scan.ranges[idx];
				minIdx = idx;
			}
		}
		double pointDist = minDist + feature_rad;
		double pointAngle = minIdx * scan.angle_increment - M_PI_2;
		// Feature position relative to laser
		x = pointDist * cos(pointAngle);
		y = pointDist * sin(pointAngle);
	};
	nearestPoint();
	new_features.push_back(Eigen::Vector2d(x, y));
}

void Matcher::detect_features(const sensor_msgs::msg::LaserScan& scan) {
	new_features.clear();
	size_t index = 0;
	while (true) {
		if (scan.ranges[index] > scan.range_max - 1.0) {
			++index;
		}
		else if (abs(scan.ranges[index] - scan.ranges[index + 1]) < feature_rad) {
			size_t end_index = index + 1;
			while (abs(scan.ranges[end_index] - scan.ranges[end_index + 1]) < feature_rad && end_index < scan.ranges.size() - 1) {
				++end_index;
			}
			add_feature(scan, index, end_index);
			index = end_index + 1;
		}
		else {++index;}

		if (index >= scan.ranges.size()) {
			break;
		}
	}
}

void Matcher::predict_features_poses() {
	const Eigen::Isometry2d interpolated_transform = incremental_transform * transform;
	predicted_features.resize(new_features.size());
	//interpolated_transform
	for (size_t idx = 0; idx < new_features.size(); ++idx) {
		predicted_features[idx] = interpolated_transform * new_features[idx];
	}
}
void Matcher::find_feature_pairs() {
	feature_pair_indices.clear();
	for (std::size_t base_index = 0; base_index < base_features.size(); ++base_index) {
		std::size_t nearest_index = 0;
		double nearest_distance = 1000000;
		for (std::size_t new_index = 0; new_index < predicted_features.size(); ++new_index) {
			const double distance =(base_features[base_index] - predicted_features[new_index]).norm();
		if (distance < nearest_distance) {
			nearest_index = new_index;
			nearest_distance = distance;
		}
		}
		if (nearest_distance < 3.0) {
			feature_pair_indices.push_back(nearest_index);
		}
		else {
			feature_pair_indices.push_back(-1);
		}
	}
}
void Matcher::find_transform() {
    Eigen::Matrix2d W = Eigen::Matrix2d::Zero();
    Eigen::Vector2d base_center = Eigen::Vector2d::Zero();
    Eigen::Vector2d new_center = Eigen::Vector2d::Zero();
    std::size_t pairs = 0;

    for (std::size_t i = 0; i < feature_pair_indices.size(); ++i) {
        if (feature_pair_indices[i] >= 0) {
            ++pairs;
            base_center += base_features[i];
            new_center += predicted_features[feature_pair_indices[i]];
        }
    }

    RCLCPP_INFO(this->get_logger(), "pairs = %d", (int)pairs);
    if (pairs < 2) {
        RCLCPP_ERROR(this->get_logger(), "Not enough feature pairs!!!");
        return;
    }

    base_center /= pairs;
    new_center /= pairs;

    for (std::size_t i = 0; i < feature_pair_indices.size(); ++i) {
        if (feature_pair_indices[i] >= 0) {
            W += (predicted_features[feature_pair_indices[i]] - new_center) * (base_features[i] - base_center).transpose();
        }
    }

    // Calculate angle from SVD decomposition
    float angle = atan2((W(0, 1) - W(1, 0)), (W(0, 0) + W(1, 1)));
    Eigen::Matrix2d R;
    R(0, 0) = R(1, 1) = cos(angle);
    R(0, 1) = -sin(angle);
    R(1, 0) = sin(angle);
    Eigen::Vector2d t = base_center - R * new_center;
    Eigen::Isometry2d result = Eigen::Translation2d(t) * Eigen::Isometry2d(R);

    // Update transforms
    incremental_transform = result * incremental_transform;
    transform = incremental_transform * transform;
    distancePassed += incremental_transform.translation().norm();

    // Calculate mean error
    double err = 0;
    for (std::size_t i = 0; i < feature_pair_indices.size(); ++i) {
        if (feature_pair_indices[i] >= 0) {
            err += (base_features[i] - transform * new_features[feature_pair_indices[i]]).norm();
        }
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Mean error = " << err/pairs);
    RCLCPP_INFO_STREAM(this->get_logger(), "Incremental transform " << std::endl << incremental_transform.matrix());
    RCLCPP_INFO_STREAM(this->get_logger(), "Transform to initial pose " << std::endl << transform.matrix());
}

void Matcher::publish_transform(const std_msgs::msg::Header& header) {
    nav_msgs::msg::Odometry odo;
    odo.header.stamp = header.stamp;
    odo.header.frame_id = map_frame;
    odo.child_frame_id = header.frame_id;

    const auto& matrix = transform.matrix();
    double yaw = atan2(matrix(1, 0), matrix(0, 0));
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    odo.pose.pose.orientation.x = q.x();
    odo.pose.pose.orientation.y = q.y();
    odo.pose.pose.orientation.z = q.z();
    odo.pose.pose.orientation.w = q.w();
    odo.pose.pose.position.x = transform.translation().x();
    odo.pose.pose.position.y = transform.translation().y();

    // Calculate velocities
    const auto& inc_matrix = incremental_transform.matrix();
    double dt = (rclcpp::Time(header.stamp) - last_stamp).seconds();
    last_stamp = header.stamp;
    double omega = atan2(matrix(1, 0), matrix(0, 0)) / dt;
    odo.twist.twist.angular.z = omega;
    odo.twist.twist.linear.x = incremental_transform.translation().x() / dt;
    odo.twist.twist.linear.y = incremental_transform.translation().y() / dt;
    odo_pub_->publish(odo);

    // Publish transform from scan to map
    // Not the opposite, since tf tree is unidirectional and already has base - odom system
    tf2::Transform tf_transform;
    Eigen::Isometry2d inverted_transform = transform.inverse();
    tf_transform.setOrigin(tf2::Vector3(inverted_transform.translation().x(),
                                     inverted_transform.translation().y(),
                                     0.0));
    tf2::Quaternion inv_q;
    const auto& inv_matrix = inverted_transform.matrix();
    double inv_yaw = atan2(inv_matrix(1, 0), inv_matrix(0, 0));
    inv_q.setRPY(0, 0, inv_yaw);
    tf_transform.setRotation(inv_q);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = header.stamp;
    t.header.frame_id = header.frame_id;
    t.child_frame_id = map_frame;
    t.transform.translation.x = inverted_transform.translation().x();
    t.transform.translation.y = inverted_transform.translation().y();
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = inv_q.x();
    t.transform.rotation.y = inv_q.y();
    t.transform.rotation.z = inv_q.z();
    t.transform.rotation.w = inv_q.w();
    
    tf_broadcaster_->sendTransform(t);
}

void Matcher::update_base_features() {
    if (base_features.empty()) {
        feature_pair_indices.clear();
        base_features = new_features;
        distancePassed = 0.0;
    } else {
        if (distancePassed > 2.0) {
            base_features.resize(new_features.size());
            for (size_t idx = 0; idx < new_features.size(); ++idx) {
                base_features[idx] = transform * new_features[idx];
            }
            distancePassed = 0.0;
        }
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Features");
    for (const auto& feature : base_features) {
        RCLCPP_INFO_STREAM(this->get_logger(), feature.transpose());
    }
}
