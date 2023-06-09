#include <algorithm>
#include <string>
#include <memory>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_line_following_controller/line_following_controller.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "nav2_line_following_controller/geometry.h"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

namespace nav2_line_following_controller
{


/**
 * Find element in iterator with the minimum calculated value
 */
template<typename Iter, typename Getter>
Iter min_by(Iter begin, Iter end, Getter getCompareVal)
{
  if (begin == end) {
    return end;
  }
  auto lowest = getCompareVal(*begin);
  Iter lowest_it = begin;
  for (Iter it = ++begin; it != end; ++it) {
    auto comp = getCompareVal(*it);
    if (comp < lowest) {
      lowest = comp;
      lowest_it = it;
    }
  }
  return lowest_it;
}


Angle yaw_from_pose(const geometry_msgs::msg::PoseStamped & pose ) {
  tf2::Quaternion q;
  tf2::fromMsg(pose.pose.orientation, q);
  tf2::Matrix3x3 m(q);
  tf2Scalar r,p,y;
  m.getRPY(r,p,y);
  return Angle::radians(y);
}




rcl_interfaces::msg::SetParametersResult LineFollowingController::on_parameters_set_callback(
  const std::vector<rclcpp::Parameter> & parameters) {
  
  RCLCPP_INFO(
    logger_,
    "Got a parameters reconfigure callback: %s",
    plugin_name_.c_str());
  
  for(const auto & parameter : parameters) {
    if(parameter.get_name() == plugin_name_+".max_cte") {
      max_cte_ = parameter.as_double();
    }
    if(parameter.get_name() == plugin_name_+".max_velocity") {
      max_velocity_ = parameter.as_double();
    }
    if(parameter.get_name() == plugin_name_+".max_reverse_velocity") {
      max_reverse_velocity_ = parameter.as_double();
    }
    if(parameter.get_name() == plugin_name_+".max_acceleration") {
      max_acceleration_ = parameter.as_double();
    }
    if(parameter.get_name() == plugin_name_+".max_deceleration") {
      max_deceleration_ = parameter.as_double();
    }
    if(parameter.get_name() == plugin_name_+".max_lateral_acceleration") {
      max_lateral_acceleration_ = parameter.as_double();
    }
    if(parameter.get_name() == plugin_name_+".min_turn_radius") {
      min_turn_radius_ = parameter.as_double();
    }
    if(parameter.get_name() == plugin_name_+".steering_k_p") {
      steering_k_p_ = parameter.as_double();
    }
    if(parameter.get_name() == plugin_name_+".steering_k_d") {
      steering_k_d_ = parameter.as_double();
    }
    if(parameter.get_name() == plugin_name_+".transform_tolerance") {
      transform_tolerance_ = rclcpp::Duration::from_seconds(parameter.as_double());
    }
    if(parameter.get_name() == plugin_name_+".lookahead_distance") {
      lookahead_distance_ = fabs(parameter.as_double());
    }

    RCLCPP_INFO(
      logger_,
      "%s: Parameter name: %s value: %s",
      plugin_name_.c_str(),
      parameter.get_name().c_str(),
      parameter.value_to_string().c_str());

  }

  rcl_interfaces::msg::SetParametersResult result; 
  result.successful = true;
  result.reason = "";
  return result;
}



void LineFollowingController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, 
    std::string name, 
    std::shared_ptr<tf2_ros::Buffer> tf, 
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
  )
{
  node_ = parent;

  auto node = node_.lock();

  costmap_ros_ = costmap_ros;
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  declare_parameter_if_not_declared(
    node,
    plugin_name_ + ".max_cte", 
    rclcpp::ParameterValue(0.2));

  declare_parameter_if_not_declared(
    node,
    plugin_name_ + ".max_velocity", 
    rclcpp::ParameterValue(1.0));

  declare_parameter_if_not_declared(
    node, 
    plugin_name_ + ".max_reverse_velocity",
    rclcpp::ParameterValue(0.5));

  declare_parameter_if_not_declared(
    node, 
    plugin_name_ + ".max_acceleration",
    rclcpp::ParameterValue(0.2));

  declare_parameter_if_not_declared(
    node, 
    plugin_name_ + ".max_deceleration",
    rclcpp::ParameterValue(0.2));

  declare_parameter_if_not_declared(
    node, 
    plugin_name_ + ".max_lateral_acceleration",
    rclcpp::ParameterValue(0.1));

  declare_parameter_if_not_declared(
    node, 
    plugin_name_ + ".lookahead_distance",
    rclcpp::ParameterValue(0.4));

  declare_parameter_if_not_declared(
    node, 
    plugin_name_ + ".transform_tolerance", 
    rclcpp::ParameterValue(0.1));

  declare_parameter_if_not_declared(
    node, 
    plugin_name_ + ".min_turn_radius", 
    rclcpp::ParameterValue(0.5));
  

  declare_parameter_if_not_declared(
    node,
    plugin_name_ + ".steering_k_p",
    rclcpp::ParameterValue(3.0));

  declare_parameter_if_not_declared(
    node,
    plugin_name_ + ".steering_k_d",
    rclcpp::ParameterValue(3.0));


  // handle keeps the callback alive
  parameters_callback_handle_ = node->add_on_set_parameters_callback(
    std::bind(&LineFollowingController::on_parameters_set_callback, this, std::placeholders::_1));
  

  node->get_parameter(plugin_name_ + ".max_cte", max_cte_);
  node->get_parameter(plugin_name_ + ".max_velocity", max_velocity_);
  node->get_parameter(plugin_name_ + ".max_reverse_velocity", max_reverse_velocity_);
  node->get_parameter(plugin_name_ + ".max_deceleration", max_deceleration_);
  node->get_parameter(plugin_name_ + ".max_acceleration", max_acceleration_);
  node->get_parameter(plugin_name_ + ".max_lateral_acceleration", max_lateral_acceleration_);
  node->get_parameter(plugin_name_ + ".lookahead_distance", lookahead_distance_);
  node->get_parameter(plugin_name_ + ".min_turn_radius", min_turn_radius_);
  node->get_parameter(plugin_name_ + ".steering_k_p", steering_k_p_);
  node->get_parameter(plugin_name_ + ".steering_k_d", steering_k_d_);
  double transform_tolerance;
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

  global_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  
}

void LineFollowingController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type nav2_line_following_controller::LineFollowingController",
    plugin_name_.c_str());
  global_pub_.reset();
}

void LineFollowingController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type nav2_line_following_controller::LineFollowingController",
    plugin_name_.c_str());
  global_pub_->on_activate();
}

void LineFollowingController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Dectivating controller: %s of type nav2_line_following_controller::LineFollowingController",
    plugin_name_.c_str());
  global_pub_->on_deactivate();
}

geometry_msgs::msg::TwistStamped LineFollowingController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & twist_pv,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  Angle yaw = yaw_from_pose(pose);

  route_position_->set_position({pose.pose.position.x, pose.pose.position.y});


  if(fabs(route_position_->cte) > max_cte_) {
    RCLCPP_WARN(logger_, "%s - %s", plugin_name_.c_str(), "aborting control, cross track error is too high");
    std::stringstream ss;
    ss << "aborting, cross track error is too high cte: " << route_position_->cte << " x: " << pose.pose.position.x << " y: " << pose.pose.position.y;
    throw std::runtime_error(ss.str().c_str());
  }

  if(route_position_->done) {
    // another subroute available?
    if(current_subroute_index_ < routes_.size() - 1) {
      // yes, advance
      ++current_subroute_index_;
      route_ = routes_[current_subroute_index_];
      publish_local_plan();
      route_position_ = std::make_shared<Route::Position>(*route_);
      route_position_->set_position({pose.pose.position.x, pose.pose.position.y});
    } else {
      RCLCPP_INFO(logger_, "%s - %s", plugin_name_.c_str(), "done with route");
    }
  }

  RCLCPP_INFO(
    logger_,
    "%s - Route position: index: %lu progress: %f cte: %f done: %d",
      plugin_name_.c_str(),
      route_position_->index,
      route_position_->progress,
      route_position_->cte,
      route_position_->done
  );



  double velocity = route_->get_velocity(*route_position_);



  auto route_yaw = route_->get_yaw(*route_position_);
  auto yaw_diff = route_yaw - yaw;
  yaw_diff.standardize();
  bool reverse = (fabs(yaw_diff.radians()) > M_PI/2);

  std::cout<<"route_yaw: " << route_yaw.degrees() << " yaw: " << yaw.degrees() << " diff: " << yaw_diff.degrees() << " reverse: " << reverse << std::endl;

  // if reverse, yaw is opposite
  if(reverse) {
    yaw = yaw+Angle::radians(M_PI);
    yaw.standardize();
    velocity = std::min(velocity, max_reverse_velocity_);
  }
  auto yaw_error = route_yaw - yaw;
  yaw_error.standardize();


  auto route_curvature = route_->curvature_ahead(*route_position_, lookahead_distance_).radians();

  // yaw error is considered d_error
  auto d_error = std::sin (yaw_error.radians());
  auto p_error = route_position_->cte;

  auto d_contribution = d_error * steering_k_d_;
  auto p_contribution = p_error * steering_k_p_;
  auto curvature = route_curvature +  d_contribution + p_contribution;


  // auto curvature = 2.0 * goal_pose.position.y /
  //   (goal_pose.position.x * goal_pose.position.x + goal_pose.position.y * goal_pose.position.y);
  
  // enforce turn radius
  if(min_turn_radius_ > 0) {
    auto max_curvature = 1.0 / min_turn_radius_;
    curvature = std::clamp(curvature, -max_curvature, max_curvature);
  }
  
  double angular_velocity = velocity * curvature;

  // Create and publish a TwistStamped message with the desired velocity
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.twist.linear.x = reverse ? -velocity : velocity;
  cmd_vel.twist.angular.z = angular_velocity;
  

  std::cout<< 
    "curvature: " << curvature
    << " route_curvature: " << route_curvature
    << " d_error: " << d_error
    << " p_error: " << p_error
    << " d_contribution " << d_contribution
    << " p_contribution: " << p_contribution
    << " reverse: " << reverse
    << " vel: " << velocity
    << " twist.x: " << twist_pv.linear.x
    << std::endl;


  return cmd_vel;
}


void LineFollowingController::publish_local_plan() {
  nav_msgs::msg::Path local_path;
  local_path.header.frame_id = global_plan_.header.frame_id;
  local_path.header.stamp = clock_->now();

  for(auto & node : route_->nodes) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = local_path.header.frame_id;
    pose.header.stamp = local_path.header.stamp;
    pose.pose.position.x = node.x;
    pose.pose.position.y = node.y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, node.yaw);
    pose.pose.orientation.w = q.getW();
    pose.pose.orientation.x = q.getX();
    pose.pose.orientation.y = q.getY();
    pose.pose.orientation.z = q.getZ();

    local_path.poses.push_back(pose);
  }
  global_pub_->publish(local_path);
}

void LineFollowingController::setPlan(const nav_msgs::msg::Path & path)
{
  Route full_route;
  full_route.set_path(path);

  // calculate and optimize sub_routes at reversals
  routes_ = full_route.split_at_reversals();
  std::cout << "route count: " << routes_.size() << std::endl;
  for(auto route : routes_) {
    route->calc_angles_and_velocities(this->max_velocity_, this->max_acceleration_, this->max_deceleration_, this->max_lateral_acceleration_);
  }

  std::cout << "publishing local path" << std::endl;

  global_plan_ = path;

  
  // set current route and position
  current_subroute_index_ = 0;
  route_ = routes_[current_subroute_index_];
  publish_local_plan();
  route_position_ = std::make_shared<Route::Position>(*route_);
}

void LineFollowingController::setSpeedLimit(const double & /*speed_limit*/, const bool & /*percentage*/) {
}


}  // namespace nav2_line_following_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(nav2_line_following_controller::LineFollowingController, nav2_core::Controller)