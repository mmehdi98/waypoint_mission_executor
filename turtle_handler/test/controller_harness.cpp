#include "turtle_handler/controller.hpp"

#include <vector>
#include <string>
#include <format>
#include <cmath>
#include <cstdio>

int g_failed = 0;

struct Point
{
  double x;
  double y;
};


void check(bool ok, const std::string& label)
{
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", label.c_str());
  if (!ok) {
    g_failed++;
  }
}

turtle_handler::Pose sim_next_turtle_pose(turtle_handler::Pose p, const turtle_handler::Command& c){
  const double dt = 1.0/62.0;
  p.x += c.v_lin * std::cos(p.theta) * dt;
  p.y += c.v_lin * std::sin(p.theta) * dt;
  p.theta += c.v_ang * dt;
  p.v_lin = c.v_lin;
  p.v_ang = c.v_ang;
  return p;
}

void velocity_limit_check(){
  std::vector<double> ds = {0.0, 0.05, 0.66, 1.0, 5.0, 50.0};
  std::vector<double> es = {0.0, 0.1, M_PI/4, M_PI/2, 3.0, M_PI};
  std::vector<turtle_handler::Pose> pose_range = {{.x = 10.0, .y=5.0}, {.x = 1.0, .y= 9.0}, {.x = 4.5, .y = 2.0}};

  for (double d : ds) {
    for (double e : es) {
      for (turtle_handler::Pose p : pose_range){
        turtle_handler::MoveInstruction i;
        i.x = p.x + d * std::cos(p.theta + e);
        i.y = p.y + d * std::sin(p.theta + e);
        i.velocity = 1.0;
        i.zone_data = 0.1;

        turtle_handler::Command c = turtle_handler::computeVelCmd(p, i);

        std::string label = std::format("Velocity within limits: [d={:.2f}, e={:.2f}]  --- Initial pose: [x={:.2f}, y={:.2f}]", d, e, p.x, p.y);
        check(c.v_lin <= i.velocity && c.v_lin>=0, label);
      }
    }
  }
}

struct MissionResult
{
  bool converged;
  double elapsed;
  double final_distance;
  double max_distance;
};

MissionResult run_to_waypoint(turtle_handler::Pose p, const turtle_handler::MoveInstruction& i, double arrival_tol, double timeout){
  const double dt = 1.0 / 62.0;
  double t = 0.0;
  double d = std::hypot(i.x - p.x, i.y - p.y);
  double d_max = d;

  while (d > arrival_tol && t < timeout) {
    p = sim_next_turtle_pose(p, turtle_handler::computeVelCmd(p, i));
    t += dt;
    d = std::hypot(i.x - p.x, i.y - p.y);
    d_max = std::fmax(d_max, d);
  }

  return MissionResult{d <= arrival_tol, t, d, d_max};
}

void convergence_check(){
  std::vector<turtle_handler::Pose> pose_range = {
    {.x = 10.0, .y=5.0, .theta=0.5, .v_lin = 1.0, .v_ang = 0.1}, 
    {.x = 1.0, .y= 9.0, .theta=0.1, .v_lin = 1.0, .v_ang = 0.1}, 
    {.x = 4.5, .y =2.0, .theta=0.4, .v_lin = 1.0, .v_ang = 0.1}
  };

  std::vector<turtle_handler::MoveInstruction> instruction_range = {
    {.x = 0.0, .y = 0.0, .theta = 0.0, .velocity = 1.0, .zone_data = 0.0},
    {.x = 8.0, .y = 7.0, .theta = 0.0, .velocity = 1.0, .zone_data = 0.0},
    {.x = 3.0, .y = 1.0, .theta = 0.0, .velocity = 1.0, .zone_data = 0.0}
  };

  double timeout = 50.0;
  for (turtle_handler::Pose p : pose_range){
    for(turtle_handler::MoveInstruction i : instruction_range){
      const double d0 = std::hypot(p.x-i.x,p.y-i.y);
      const double tol = std::fmax(i.zone_data, 0.05);
      const double timeout = d0/i.velocity + 10.0;

      MissionResult r = run_to_waypoint(p,i,tol,timeout);

      const std::string label = std::format(
        "converges [start=({:.1f},{:.1f},th={:+.2f}) goal=({:.1f},{:.1f})] "
        "d0={:.2f} t={:.2f}/{:.2f}s final_d={:.3f} max_d={:.2f}",
        p.x, p.y, p.theta, i.x, i.y, d0,
        r.elapsed, timeout, r.final_distance, r.max_distance
      );

      check(r.converged, label);
      
    }
  }
}

int main()
{

  velocity_limit_check();

  convergence_check();

  std::printf("\n%d failed\n", g_failed);
  return g_failed;
}