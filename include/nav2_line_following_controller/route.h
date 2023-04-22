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
    Route & route;
    bool done = false;
    size_t index = 0;
    double progress = 0.0; // portion progress along current route segment
    double cte = 0.0;      // cross track error, signed distance from robot to centerline

    Position(Route & route) :
      route(route) {
    }

    void set_position(Point position)
    {
      while(index + 1 < route.nodes.size()) {
        Route::Node & p1 = route.nodes[index];
        Route::Node & p2 = route.nodes[index+1];

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


  void set_path(const nav_msgs::msg::Path & path) {
    nodes.resize(path.poses.size());
    for(size_t i = 0; i < nodes.size(); ++i) {
      auto & node = nodes[i];
      const auto & pose = path.poses[i].pose;
      node.x = pose.position.x;
      node.y = pose.position.y;

      tf2::Quaternion q;
      tf2::fromMsg(pose.orientation, q);
      tf2::Matrix3x3 m(q);
      tf2Scalar r,p,y;
      m.getRPY(r,p,y);
      node.yaw = y;
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


  void optimize_velocity(double max_velocity, double /*max_acceleration*/, double max_deceleration, double /*max_lateral_acceleration*/) {

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
    }


    // // starting velocity must be zero
    // // apply speed up / acceleration
    // // from start to end
    // velocity_plan_[0] = 0;
    // for(int i=0; i < (int)poses.size()-2; i++) {
    //   auto & p0 = poses[i].pose.position;
    //   auto & p1 = poses[i+1].pose.position;
    //   double ds = ::distance(p0.x,p0.y,p1.x,p1.y);
    //   velocity_plan_[i+1] = min(velocity_plan_[i+1], velocity_at_position(ds,max_acceleration,velocity_plan_[i]));
    // }
  }
};