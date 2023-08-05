#include "nav2_line_following_controller/route.h"
#include "gtest/gtest.h"

/*
Returns three node route:
(0,0) -> (1,0) -> (2,1)
*/

nav_msgs::msg::Path get_simple_test_path() {
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = rclcpp::Time(0);
  path.poses.resize(3);
  path.poses[0].pose.position.x = 0;
  path.poses[0].pose.position.y = 0;
  path.poses[0].pose.position.z = 0;
  path.poses[0].pose.orientation.x = 0;
  path.poses[0].pose.orientation.y = 0;
  path.poses[0].pose.orientation.z = 0;
  path.poses[0].pose.orientation.w = 1;
  path.poses[1].pose.position.x = 1;
  path.poses[1].pose.position.y = 0;
  path.poses[1].pose.position.z = 0;
  path.poses[1].pose.orientation.x = 0;
  path.poses[1].pose.orientation.y = 0;
  path.poses[1].pose.orientation.z = 0;
  path.poses[1].pose.orientation.w = 1;
  path.poses[2].pose.position.x = 2;
  path.poses[2].pose.position.y = 1;
  path.poses[2].pose.position.z = 0;
  path.poses[2].pose.orientation.x = 0;
  path.poses[2].pose.orientation.y = 0;
  path.poses[2].pose.orientation.z = 0;
  path.poses[2].pose.orientation.w = 1;
  return path;
}

TEST(route_tests, initialization) {
  Route route;
  nav_msgs::msg::Path path = get_simple_test_path();
  route.set_path(path);
  ASSERT_EQ(route.nodes.size(), 3);
}

TEST(route_tests, traverse_route) {
  const auto tolerance = 1e-6;

  Route route;
  nav_msgs::msg::Path path = get_simple_test_path();
  route.set_path(path);

  double max_velocity = 2.0;
  double max_deceleration = 1.0;
  route.calc_angles_and_velocities(max_velocity, max_deceleration);


  auto route_position = std::make_shared<Route::Position>(route);
  route_position->set_position({0, 0});

  // start route
  ASSERT_EQ(route_position->index, 0);
  ASSERT_EQ(route_position->cte, 0.0);
  ASSERT_EQ(route_position->progress, 0.0);
  ASSERT_NEAR(route.get_velocity(*route_position),  velocity_at_position(2.0, 1.0, 0.0), tolerance);

  // move along first segment with a little error above
  route_position->set_position({0.5, 0.1});
  ASSERT_EQ(route_position->index, 0);
  ASSERT_EQ(route_position->cte, -0.1);
  ASSERT_EQ(route_position->progress, 0.5);

  // move further along first segment with a little error below
  route_position->set_position({0.6, -0.1});
  ASSERT_EQ(route_position->index, 0);
  ASSERT_EQ(route_position->cte, 0.1);
  ASSERT_EQ(route_position->progress, 0.6);

  // move past first segment, below second
  route_position->set_position({2.0, 0.0});
  ASSERT_EQ(route_position->index, 1.0);
  ASSERT_NEAR (route_position->cte, sqrt(2)/2.0, tolerance);
  ASSERT_EQ(route_position->progress, 0.5);

  // move further along second segment, no error
  route_position->set_position({1.9, 0.9});
  ASSERT_EQ(route_position->index, 1);
  ASSERT_NEAR (route_position->cte, 0.0, tolerance);
  ASSERT_NEAR(route_position->progress, 0.9, tolerance);

  // move past end of route
  route_position->set_position({2.1, 1.1});
  ASSERT_EQ(route_position->index, 1);
  ASSERT_EQ(route_position->done, true);
  ASSERT_NEAR (route_position->cte, 0.0, tolerance);
  ASSERT_NEAR(route_position->progress, 1.1, tolerance);
}