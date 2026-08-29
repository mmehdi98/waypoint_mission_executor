#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim_msgs/msg/pose.hpp"
#include "my_robot_interfaces/action/move_along_path.hpp"
#include "my_robot_interfaces/msg/rob_target.hpp"
#include "my_robot_interfaces/msg/move_instruction.hpp"

using Twist = geometry_msgs::msg::Twist;
using Pose = turtlesim_msgs::msg::Pose;
using MoveAlongPath = my_robot_interfaces::action::MoveAlongPath;
using MoveAlongPathGoalHandle = rclcpp_action::ServerGoalHandle<MoveAlongPath>;
using RobTarget = my_robot_interfaces::msg::RobTarget;
using MoveInstruction = my_robot_interfaces::msg::MoveInstruction;
using namespace std::placeholders;

class TurtleController : public rclcpp::Node
{
public:
    TurtleController() : Node("turtle_controller"){
        pose_subscriber_ = this -> create_subscription<Pose>("turtle1/pose", 10, std::bind(&TurtleController::publishCmdCallback, this, _1));
        vel_cmd_publisher_ = this -> create_publisher<Twist>("turtle1/cmd_vel",10);
        motion_server_ = rclcpp_action::create_server<MoveAlongPath>(this, "move_along_path", 
                        std::bind(&TurtleController::goalCallback, this, _1, _2),
                        std::bind(&TurtleController::cancelCallback, this, _1),
                        std::bind(&TurtleController::acceptedCallback, this, _1));
    }

    void publishCmdCallback(const Pose& current_pose){
        current_pose_ = current_pose;

        motionPlanner_();
        
        if (active_goal_){
            vel_cmd_ = computeVelCmd_(current_pose, current_instruction_);
        }
        else{
            vel_cmd_.linear.x = 0;
            vel_cmd_.angular.z = 0;
        }
        vel_cmd_publisher_->publish(vel_cmd_);
    }

    rclcpp_action::GoalResponse goalCallback(const rclcpp_action::GoalUUID uuid, std::shared_ptr<const MoveAlongPath::Goal> goal){
        auto path = goal -> path;
        for (MoveInstruction instruction : path){
            if (false)
                return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancelCallback(const std::shared_ptr<MoveAlongPathGoalHandle>& goal_handle){

    }

    void acceptedCallback(const std::shared_ptr<MoveAlongPathGoalHandle>& goal_handle){
        request_queue_.push_back(goal_handle);
    }


private:
    rclcpp::Publisher<Twist>::SharedPtr vel_cmd_publisher_;
    rclcpp::Subscription<Pose>::SharedPtr pose_subscriber_;
    rclcpp_action::Server<MoveAlongPath>::SharedPtr motion_server_;

    Pose current_pose_;
    Twist vel_cmd_;
    MoveInstruction current_instruction_;
    std::shared_ptr<MoveAlongPathGoalHandle> active_goal_;
    std::shared_ptr<MoveAlongPathGoalHandle> previous_goal_;
    std::deque<std::shared_ptr<MoveAlongPathGoalHandle>> request_queue_;
    int path_index_;

    void motionPlanner_(){
        std::vector<MoveInstruction> path;

        if (request_queue_.size() != 0){
            previous_goal_ = active_goal_;
            active_goal_ = request_queue_[0];
            path = active_goal_ ->get_goal()->path;
        }
        else{
            active_goal_ = nullptr;
        }

        if ((previous_goal_ != active_goal_))
            path_index_ = 0;

        if (computeRemainingDistance_(current_pose_, path[path_index_].target) <= path[path_index_].zone_data)
            ++path_index_;

        if (path_index_ >= path.size()){
            path_index_ = 0;
            
            auto result = std::make_shared<MoveAlongPath::Result>();
            result -> message = "Finished path";
            request_queue_[0]->succeed(result);
            request_queue_.pop_front();

        }
        
        current_instruction_ = path[path_index_];
    }

    Twist computeVelCmd_(const Pose& current_pose, const MoveInstruction& current_instruction) const{

    }

    double computeRemainingDistance_(const Pose& current_pose, const RobTarget& target){

    }

};


int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

