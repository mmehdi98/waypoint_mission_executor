#ifndef TURTLE_HANDLER_CONVERSIONS_HPP
#define TURTLE_HANDLER_CONVERSIONS_HPP

#include "turtle_handler/controller.hpp"
#include "turtlesim_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_robot_interfaces/msg/move_instruction.hpp"

namespace turtle_handler{

    inline Pose toPose(const turtlesim_msgs::msg::Pose& pose){
        Pose result;
        result.x = pose.x;
        result.y = pose.y;
        result.theta = pose.theta;
        result.v_lin = pose.linear_velocity;
        result.v_ang = pose.angular_velocity;
        return result;
    }

    inline geometry_msgs::msg::Twist toTwist(const Command& cmd){
        geometry_msgs::msg::Twist result;
        result.linear.x = cmd.v_lin;
        result.angular.z = cmd.v_ang;
        return result;
    }

    inline MoveInstruction toMoveInstruction(const my_robot_interfaces::msg::MoveInstruction& instruction){
        MoveInstruction result;
        result.x = instruction.target.x;
        result.y = instruction.target.y;
        result.theta = instruction.target.theta;
        result.velocity = instruction.velocity;
        result.zone_data = instruction.zone_data;
        return result;
    }
    
}


#endif