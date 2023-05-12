#include <cmath>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include "geometry.h"



class Route
{
public:
  struct Node {
    double x = NAN;
    double y = NAN;
    tf2::Quaternion q;
    double yaw = NAN;
    double velocity = NAN;
    double curvature = NAN;
  };

  struct Position {
    const Route & route;
    bool done = false;
    size_t index = 0;
    double progress = 0.0; // portion progress along current route segment
    double cte = 0.0;      // cross track error, signed distance from robot to centerline

    Position(const Route & route) :
      route(route) {
    }

    void set_position(Point position)
    {
      while(index + 1 < route.nodes.size()) {
        auto & p1 = route.nodes[index];
        auto & p2 = route.nodes[index+1];

        // d from p1 to p2
        auto dx = p2.x-p1.x;
        auto dy = p2.y-p1.y;

        // d from p1 to robot
        auto drx = position.x - p1.x;
        auto dry = position.y - p1.y;

        double l = length(dx,dy);
        if(l < 0.000001) {
          progress = 1.1;
        } else {
          progress = (drx * dx + dry * dy)/(dx * dx + dy * dy);
          cte = (drx * dy - dry * dx ) / l;
        }

        if (progress < 1.0)
          break;
        if (done)
          break;

        advance_to_next_segment();
      }        
    }

    void advance_to_next_segment() {
      if (index >= route.nodes.size()-2) {
        done = true;
      }
      ++index;
    }    


  };

  vector<Node> nodes;


  Angle get_yaw(const Position & position) {
    auto p0 = nodes[position.index];
    auto p1 = nodes[position.index+1];

    if(position.progress < 0.0) {
      return Angle::radians(p0.yaw);
    }
    if(position.progress > 1.0) {
      return Angle::radians(p1.yaw);
    }
    return Angle::radians(
      p0.yaw + (p1.yaw - p0.yaw) * position.progress);
  }

  // returned is theta / m in radians
  Angle curvature_ahead(const Position & position, double d) {
    auto ahead = this->get_position_ahead(position, d);
    Angle theta_start = this->get_yaw(position);
    Angle theta_end = this->get_yaw(ahead.position);

    auto curvature_ahead =  ahead.actual_distance <= 0.0 ? 
      Angle::radians(0.0)
      :  Angle::radians((theta_end-theta_start).radians() / ahead.actual_distance);

    std::cout << "theta_start: " << theta_start.degrees() << " theta_end: " << theta_end.degrees() << " actual_d: " << ahead.actual_distance << " curvature: " << curvature_ahead << std::endl ;
    return curvature_ahead;
  }


  void set_path(const nav_msgs::msg::Path & path) {
    nodes.resize(path.poses.size());
    for(size_t i = 0; i < nodes.size(); ++i) {
      auto & node = nodes[i];
      const auto & pose = path.poses[i].pose;
      node.x = pose.position.x;
      node.y = pose.position.y;
    }

    // calculate route directions
    for(size_t i = 0; i < nodes.size(); ++ i) {
      auto & node = nodes[i];
      if(i < nodes.size()-1) {
        const auto & next_node = nodes[i+1];
        node.yaw = atan2(next_node.y - node.y, next_node.x - node.x);
      } else {
        node.yaw = nodes[i-1].yaw;
      }    
    }
  }

  // returns desired velocity for current position
  double get_velocity(const Route::Position & position)
  {
    if(position.done)
      return 0;
    auto p0 = nodes[position.index];
    auto p1 = nodes[position.index+1];
    double v = NAN;

    // if we just turned around, take the velocity from the node in the new direction
    if (position.progress < 0) {
      v =  p0.velocity;
    } else if (position.progress > 1){
      v = p1.velocity;
    } else {
      v = p0.velocity + (p1.velocity - p0.velocity) * position.progress;
    }

    return v ;
  }

  struct PositionAheadResult {
    Position position;
    double actual_distance;
  };

  PositionAheadResult get_position_ahead(const Position & position, double ahead_d) const {
    Position ahead_position(position.route);
    ahead_position.index = position.index;
    ahead_position.progress = position.progress;

    double total_ahead_distance = 0.0;

    while(true) {
      const auto & p1 = nodes[ahead_position.index];
      const auto & p2 = nodes[ahead_position.index+1];
      double dx = p2.x-p1.x;
      double dy = p2.y-p1.y;
      auto segment_length = ::length(dx,dy);
      double segment_progress_d = segment_length * ahead_position.progress;
      double segment_remaining_d = segment_length - segment_progress_d;

      // is the end in this segment, we can return progress in this segment
      if ( ahead_d < segment_remaining_d) {
        ahead_position.progress += ahead_d / segment_length;
        total_ahead_distance += ahead_d;
        break;
      }

      // did we fall off the end of the route?
      if( ahead_position.index >= nodes.size() -2 ) {
        ahead_position.progress += ahead_d / segment_length;
        total_ahead_distance += segment_remaining_d;
        break;
      }

      ++ahead_position.index;
      ahead_position.progress = 0.0;
      total_ahead_distance += segment_remaining_d;
      ahead_d -= segment_remaining_d;
    }

    return {ahead_position, total_ahead_distance};
  }

  // splits a route that may include reversals at the reversing points into a 
  // a vector of one-way subroutes
  std::vector<std::shared_ptr<Route>> split_at_reversals() {
    std::vector<std::shared_ptr<Route>> subroutes; 
    std::shared_ptr<Route> subroute = std::make_shared<Route>();
    subroutes.push_back(subroute);

    for(size_t i=0; i < nodes.size(); ++i) {
      if(subroute->nodes.size() < 2) {
        subroute->nodes.push_back(nodes[i]);
      } else {
        // check to see if there is a reversal with dot product
        auto & p1 = subroute->nodes[subroute->nodes.size() - 2];
        auto & p2 = subroute->nodes[subroute->nodes.size() - 1];
        auto & p3 = nodes[i];
        bool is_reversal = ((p2.x-p1.x) * (p3.x-p2.x) + (p2.y-p1.y) * (p3.y-p2.y)) < 0;

        if(is_reversal) {
          // start a new subroute
          subroute = std::make_shared<Route>();
          subroutes.push_back(subroute);
          subroute->nodes.push_back(p2);
        } 
        subroute->nodes.push_back(p3);
      }
    }
    return subroutes;
  }
  
  void calc_angles_and_velocities(double max_velocity, double /*max_acceleration*/, double max_deceleration, double /*max_lateral_acceleration*/) {

    for(auto & node : nodes) {
      node.velocity = max_velocity;
    }

    // final velocity should be zero
    nodes.back().velocity =0.0;

    // apply slow down / stopping acceleration
    // from end to start
    for(int i = nodes.size()-2; i >= 0; --i) {
      auto & p0 = nodes[i];
      auto & p1 = nodes[i+1];
      double ds = ::distance(p0.x,p0.y,p1.x,p1.y);
      p0.velocity = min(p0.velocity, velocity_at_position(ds,max_deceleration,p1.velocity));
      std::cout
        << "i: " << i 
        << " max_velocity: " << max_velocity
        << " max_deceleration: " << max_deceleration
        << " ds: " << ds
        << " p0.velocity" << p0.velocity
        << std::endl;
      p0.yaw = p1.yaw = atan2(p1.y-p0.y,p1.x-p0.x);
    }

  }
};