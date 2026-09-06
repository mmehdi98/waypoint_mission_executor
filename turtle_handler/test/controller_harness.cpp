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


void check(bool ok, std::string& label)
{
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", label.c_str());
  if (!ok) {
    g_failed++;
  }
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

int main()
{

  velocity_limit_check();

  std::printf("\n%d failed\n", g_failed);
  return g_failed;
}