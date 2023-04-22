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




rcl_interfaces::msg::SetParametersResult LineFollowingController::on_parameters_set_callback(
  const std::vector<rclcpp::Parameter> & parameters) {
  
  RCLCPP_INFO(
    logger_,
    "Got a parameters reconfigure callback: %s",
    plugin_name_.c_str());
  
  for(const auto & parameter : parameters) {
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

  // handle keeps the callback alive
  parameters_callback_handle_ = node->add_on_set_parameters_callback(
    std::bind(&LineFollowingController::on_parameters_set_callback, this, std::placeholders::_1));
  

  node->get_parameter(plugin_name_ + ".max_velocity", max_velocity_);
  node->get_parameter(plugin_name_ + ".max_reverse_velocity", max_reverse_velocity_);
  node->get_parameter(plugin_name_ + ".max_deceleration", max_deceleration_);
  node->get_parameter(plugin_name_ + ".max_acceleration", max_acceleration_);
  node->get_parameter(plugin_name_ + ".max_lateral_acceleration", max_lateral_acceleration_);
  node->get_parameter(plugin_name_ + ".max_velocity", max_velocity_);
  node->get_parameter(plugin_name_ + ".lookahead_distance", lookahead_distance_);
  node->get_parameter(plugin_name_ + ".min_turn_radius", min_turn_radius_);
  double transform_tolerance;
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

  global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  
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
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{

  route_position_->set_position({pose.pose.position.x, pose.pose.position.y});
  RCLCPP_INFO(
    logger_,
    "%s - Route position: index: %lu progress: %f cte: %f done: %d",
      plugin_name_.c_str(),
      route_position_->index,
      route_position_->progress,
      route_position_->cte,
      route_position_->done
      );


  auto transformed_plan = transformGlobalPlan(pose);

  // Find the first pose which is at a distance greater than the specified lookahead distance
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_distance_;
    });

  // If the last pose is still within lookahead distance, take the last pose
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  }
  auto goal_pose = goal_pose_it->pose;


  double abs_velocity = route_.get_velocity(*route_position_);
  RCLCPP_INFO(
    logger_,
    "%s - Route velocity: %f",
    plugin_name_.c_str(),
    abs_velocity
    );

  double velocity = (goal_pose.position.x > 0) ? abs_velocity : -abs_velocity;
  auto curvature = 2.0 * goal_pose.position.y /
    (goal_pose.position.x * goal_pose.position.x + goal_pose.position.y * goal_pose.position.y);
  
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
  cmd_vel.twist.linear.x = velocity;
  cmd_vel.twist.angular.z = angular_velocity;
  
  return cmd_vel;
}




void LineFollowingController::setPlan(const nav_msgs::msg::Path & path)
{
  global_pub_->publish(path);
  global_plan_ = path;

  route_.set_path(path);
  route_.optimize_velocity(this->max_velocity_, this->max_acceleration_, this->max_deceleration_, this->max_lateral_acceleration_);
  route_position_ = std::make_shared<Route::Position>(route_);

}

nav_msgs::msg::Path
LineFollowingController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  // Original mplementation taken fron nav2_dwb_controller

  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(
      tf_, global_plan_.header.frame_id, pose,
      robot_pose, transform_tolerance_))
  {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }

  // We'll discard points on the plan that are outside the local costmap
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
    costmap->getResolution() / 2.0;

  // First find the closest pose on the path to the robot
  auto transformation_begin =
    min_by(
    global_plan_.poses.begin(), global_plan_.poses.end(),
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // From the closest point, look for the first point that's further then dist_threshold from the
  // robot. These points are definitely outside of the costmap so we won't transform them.
  auto transformation_end = std::find_if(
    transformation_begin, end(global_plan_.poses),
    [&](const auto & global_plan_pose) {
      return euclidean_distance(robot_pose, global_plan_pose) > dist_threshold;
    });

  // Helper function for the transform below. Transforms a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      // We took a copy of the pose, let's lookup the transform at the current time
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      transformPose(
        tf_, costmap_ros_->getBaseFrameID(),
        stamped_pose, transformed_pose, transform_tolerance_);
      return transformed_pose;
    };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
  global_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}


void LineFollowingController::setSpeedLimit(const double & /*speed_limit*/, const bool & /*percentage*/) {
}


bool LineFollowingController::transformPose(
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  const rclcpp::Duration & transform_tolerance
) const
{
  // Implementation taken as is fron nav_2d_utils in nav2_dwb_controller

  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf->transform(in_pose, out_pose, frame);
    return true;
  } catch (tf2::ExtrapolationException & ex) {
    auto transform = tf->lookupTransform(
      frame,
      in_pose.header.frame_id,
      tf2::TimePointZero
    );
    if (
      (rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) >
      transform_tolerance)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("tf_help"),
        "Transform data too old when converting from %s to %s",
        in_pose.header.frame_id.c_str(),
        frame.c_str()
      );
      RCLCPP_ERROR(
        rclcpp::get_logger("tf_help"),
        "Data time: %ds %uns, Transform time: %ds %uns",
        in_pose.header.stamp.sec,
        in_pose.header.stamp.nanosec,
        transform.header.stamp.sec,
        transform.header.stamp.nanosec
      );
      return false;
    } else {
      tf2::doTransform(in_pose, out_pose, transform);
      return true;
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("tf_help"),
      "Exception in transformPose: %s",
      ex.what()
    );
    return false;
  }
  return false;
}

}  // namespace nav2_line_following_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(nav2_line_following_controller::LineFollowingController, nav2_core::Controller)