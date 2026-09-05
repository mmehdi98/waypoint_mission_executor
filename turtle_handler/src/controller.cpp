#include "turtle_handler/controller.hpp"
#include <cmath>

turtle_handler::Command turtle_handler::computeVelCmd(const turtle_handler::Pose& current_pose, const turtle_handler::MoveInstruction& instruction) {
    turtle_handler::Command result;
    double Kv = 2;
    double Kh = 10;
    double dx = current_pose.x-instruction.x;
    double dy = current_pose.y-instruction.y;
    double error_dist = sqrt(dx*dx+dy*dy);
    
    double theta_d = std::atan2(-dy,-dx);
    double error_theta = std::remainder(theta_d - current_pose.theta, 2*M_PI);
    double velocity_d = Kv*error_dist*(cos(error_theta/2)*cos(error_theta/2));
    
    if (velocity_d > instruction.velocity)
        velocity_d = instruction.velocity;

    double omega_d = current_pose.v_lin*(dx*std::sin(current_pose.theta)-dy*std::cos(current_pose.theta))/pow(error_dist,2);

    result.v_lin = velocity_d;
    result.v_ang = omega_d + Kh*error_theta;

    return result;
}