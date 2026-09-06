// Unit tests for turtle_handler::computeVelCmd in src/controller.cpp
//
// g++ -std=c++20 -Wall -Wextra -Iinclude test/controller_harness.cpp src/controller.cpp -o harness ./harness

#include "turtle_handler/controller.hpp"

#include <cmath>
#include <cstdio>
#include <format>
#include <string>
#include <vector>

namespace
{
constexpr double kDt = 1.0 / 62.0;
constexpr double kOmegaLimit = 3.0;
constexpr double kAlphaLimit = 5.0;
constexpr double kMinArrivalTol = 0.05;

constexpr double kMaxPathRatio = 1.30;
constexpr double kMaxTimeRatio = 3.00;

int g_failed = 0;

void check(bool ok, const std::string & label)
{
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", label.c_str());
  if (!ok) {
    g_failed++;
  }
}

turtle_handler::Pose sim_next_turtle_pose(turtle_handler::Pose p, const turtle_handler::Command & c)
{
  p.x += c.v_lin * std::cos(p.theta) * kDt;
  p.y += c.v_lin * std::sin(p.theta) * kDt;
  p.theta += c.v_ang * kDt;
  p.v_lin = c.v_lin;
  p.v_ang = c.v_ang;
  return p;
}

struct MissionResult
{
  bool converged = false;
  int steps = 0;
  double elapsed = 0.0;
  double d0 = 0.0;
  double final_distance = 0.0;
  double path_length = 0.0;
  double effort = 0.0;
  double accel_cost = 0.0;
  double max_omega = 0.0;
  double max_alpha = 0.0;
  double saturated_fraction = 0.0;
  int backward_steps = 0;

  double path_ratio() const { return d0 > 0.0 ? path_length / d0 : 1.0; }
  double time_ratio(double v) const
  {
    const double ideal = d0 / v;
    return ideal > 0.0 ? elapsed / ideal : 0.0;
  }
};

MissionResult run_to_waypoint(
  turtle_handler::Pose p, const turtle_handler::MoveInstruction & goal, double arrival_tol,
  double timeout)
{
  MissionResult r;
  r.d0 = std::hypot(goal.x - p.x, goal.y - p.y);

  double d = r.d0;
  double prev_omega = 0.0;
  bool have_prev_omega = false;
  int saturated = 0;

  while (d > arrival_tol && r.elapsed < timeout) {
    const turtle_handler::Command c = turtle_handler::computeVelCmd(p, goal);

    // --- control input metrics ---
    r.effort += c.v_ang * c.v_ang * kDt;
    r.max_omega = std::fmax(r.max_omega, std::fabs(c.v_ang));
    if (c.v_lin >= goal.velocity * 0.999) {
      saturated++;
    }

    // --- smoothness metrics ---
    if (have_prev_omega) {
      const double alpha = (c.v_ang - prev_omega) / kDt;
      r.accel_cost += alpha * alpha * kDt;
      r.max_alpha = std::fmax(r.max_alpha, std::fabs(alpha));
    }
    prev_omega = c.v_ang;
    have_prev_omega = true;

    const turtle_handler::Pose next = sim_next_turtle_pose(p, c);
    r.path_length += std::hypot(next.x - p.x, next.y - p.y);

    const double d_next = std::hypot(goal.x - next.x, goal.y - next.y);
    if (d_next > d) {
      r.backward_steps++;
    }

    p = next;
    d = d_next;
    r.elapsed += kDt;
    r.steps++;
  }

  r.converged = d <= arrival_tol;
  r.final_distance = d;
  r.saturated_fraction = r.steps > 0 ? static_cast<double>(saturated) / r.steps : 0.0;
  return r;
}

// convergence and control quality

struct Case
{
  const char * name;
  turtle_handler::Pose start;
  turtle_handler::MoveInstruction goal;
};

const std::vector<Case> & cases()
{
  static const std::vector<Case> c = {
    {"ahead-ish",
     {.x = 10.0, .y = 5.0, .theta = 0.5, .v_lin = 0.0, .v_ang = 0.0},
     {.x = 2.0, .y = 2.0, .theta = 0.0, .velocity = 1.0, .zone_data = 0.1}},
    {"diagonal",
     {.x = 1.0, .y = 9.0, .theta = 0.1, .v_lin = 0.0, .v_ang = 0.0},
     {.x = 8.0, .y = 3.0, .theta = 0.0, .velocity = 1.0, .zone_data = 0.1}},
    {"facing away",
     {.x = 4.5, .y = 2.0, .theta = M_PI, .v_lin = 0.0, .v_ang = 0.0},
     {.x = 9.0, .y = 6.0, .theta = 0.0, .velocity = 1.0, .zone_data = 0.1}},
    {"short hop",
     {.x = 5.0, .y = 5.0, .theta = 0.0, .v_lin = 0.0, .v_ang = 0.0},
     {.x = 5.0, .y = 5.3, .theta = 0.0, .velocity = 1.0, .zone_data = 0.05}},
    {"fine stop",
     {.x = 2.0, .y = 2.0, .theta = 0.0, .v_lin = 0.0, .v_ang = 0.0},
     {.x = 7.0, .y = 7.0, .theta = 0.0, .velocity = 1.0, .zone_data = 0.0}},
  };
  return c;
}

void metrics_table()
{
  std::printf("\n--- performance ---\n");
  std::printf(
    "%-12s %6s %7s %7s %8s %9s %7s %8s %6s %5s\n", "case", "d0", "pathrat", "timerat", "effort",
    "accel", "max_w", "max_a", "sat%", "back");

  for (const auto & tc : cases()) {
    const double d0 = std::hypot(tc.goal.x - tc.start.x, tc.goal.y - tc.start.y);
    const double tol = std::fmax(tc.goal.zone_data, kMinArrivalTol);
    const double timeout = 4.0 * d0 / tc.goal.velocity + 15.0;

    const MissionResult r = run_to_waypoint(tc.start, tc.goal, tol, timeout);

    std::printf(
      "%-12s %6.2f %7.3f %7.3f %8.2f %9.2f %7.2f %8.2f %6.1f %5d\n", tc.name, r.d0, r.path_ratio(),
      r.time_ratio(tc.goal.velocity), r.effort, r.accel_cost, r.max_omega, r.max_alpha,
      100.0 * r.saturated_fraction, r.backward_steps);
  }
}

void convergence_check()
{
  std::printf("\n--- convergence and quality ---\n");

  for (const auto & tc : cases()) {
    const double d0 = std::hypot(tc.goal.x - tc.start.x, tc.goal.y - tc.start.y);
    const double tol = std::fmax(tc.goal.zone_data, kMinArrivalTol);
    const double timeout = 4.0 * d0 / tc.goal.velocity + 15.0;

    const MissionResult r = run_to_waypoint(tc.start, tc.goal, tol, timeout);

    const std::string where = std::format(
      " [{} d0={:.2f} t={:.2f}/{:.2f}s final_d={:.3f}]", tc.name, r.d0, r.elapsed, timeout,
      r.final_distance);

    check(r.converged, "reaches goal" + where);

    if (r.converged) {
      check(r.path_ratio() < kMaxPathRatio, std::format(
        "path ratio {:.3f} < {:.2f}", r.path_ratio(), kMaxPathRatio) + where);
      check(r.max_omega <= kOmegaLimit, std::format(
        "max omega {:.2f} <= {:.2f} rad/s", r.max_omega, kOmegaLimit) + where);
      check(r.max_alpha <= kAlphaLimit, std::format(
        "max alpha {:.2f} <= {:.2f} rad/s^2", r.max_alpha, kAlphaLimit) + where);

      if (r.d0 > 1.0) {
        check(r.time_ratio(tc.goal.velocity) < kMaxTimeRatio, std::format(
          "time ratio {:.2f} < {:.2f}", r.time_ratio(tc.goal.velocity), kMaxTimeRatio) + where);
      }
    }
  }
}

} 

int main()
{
  metrics_table();
  convergence_check();

  std::printf("\n%d failed\n", g_failed);
  return g_failed;
}