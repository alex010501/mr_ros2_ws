/*
 * Controller.h
 *
 *  Created on: 30 апр. 2017 г.
 *      Author: aminin
 */

#ifndef SRC_CONTROLLER_H_
#define SRC_CONTROLLER_H_


#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/point_cloud.h>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <list>
#include <memory>
#include <chrono>
#include <thread>
#include <mutex>

#include "trajectory_segment.h"

namespace simple_controller
{

using TrajPtr = std::shared_ptr<trajectory::TrajectorySegment>;

/*!
 *\brief robot controller
 * controls following along defined trajectory via simple pid regulator
 * angular_velocity = pid(error)
 * error is distance to trajectory
 * trajectory is list of angular and linear segments, saved as pointers to base class Trajectory
 * Trajectory is cycled
 * feedback from robot is received via ground_truth callback (real position of robot)
 * during control future trajectory is published for velocity controller
 */
class Controller : public rclcpp::Node
{
protected:


  double p_factor;
  double d_factor;
  double i_factor;
  double max_antiwindup_error;
  double error_integral;
  double last_error;

  ///\ circle params
  double radius;
  ///\ second circle center
  double  cy;

  double max_curvature;

  //discrete of publish trajectory
  double traj_dl;
  //length of published trajectory
  double traj_length;

  double robot_x = 0.0;
  double robot_y = 0.0;
  double robot_theta = 0.0;

  double lam = 0.1;
	double c = 1;
  
  //time of robot coordinates update
  rclcpp::Time robot_time;

  double current_linear_velocity = 0.0;
  double current_angular_velocity = 0.0;
  
  using Trajectory = std::list<TrajPtr>;
  /// \ container of trajectory segments
  std::list<TrajPtr> trajectory;

  nav_msgs::msg::Path path;
  std::size_t nearest_point_index;

  /// \ current segment
  std::list<TrajPtr>::iterator current_segment;
  /// \ length of the current segment at the current point
  double current_segment_length = 0.0;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr err_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steer_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  /// \ frame_id for coordinates of controller
  std::string world_frame_id;

  std::size_t cal_target_index();

  void on_timer();
  void on_pose(const nav_msgs::msg::Odometry& odom);
  void on_path(const nav_msgs::msg::Path& path);
  /*
   *@brief calculates feedback error for trajectory
   *@return feedback error
   */
  double cross_track_error();

  /// \ update robot pose to current time based on last pose and velocities
  void update_robot_pose(double dt);
  /*
   * \brief publishes trajectory as pointcloud message
   */
  void publish_trajectory();
  void on_odo(const nav_msgs::msg::Odometry& odom);
  void publish_error(double error);
  nav_msgs::msg::Path create_path() const;
  std::size_t get_nearest_path_pose_index(int start_index,
                                          std::size_t search_len);

public:
  double get_p_factor(){ return p_factor; }
  double get_d_factor(){ return d_factor; }
  double get_i_factor(){ return i_factor; }
  void reset();
  void reset(double p, double d, double i );
  Controller();
  virtual ~Controller();
};

} /* namespace simple_controller */

#endif /* SRC_CONTROLLER_H_ */
