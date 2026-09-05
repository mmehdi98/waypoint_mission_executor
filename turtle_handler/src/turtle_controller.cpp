#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim_msgs/msg/pose.hpp"
#include "waypoint_interfaces/action/move_along_path.hpp"
#include "waypoint_interfaces/msg/rob_target.hpp"
#include "waypoint_interfaces/msg/move_instruction.hpp"
#include "turtle_handler/controller.hpp"
#include "turtle_handler/conversions.hpp"

using Twist = geometry_msgs::msg::Twist;
using Pose = turtlesim_msgs::msg::Pose;
using MoveAlongPath = waypoint_interfaces::action::MoveAlongPath;
using MoveAlongPathGoalHandle = rclcpp_action::ServerGoalHandle<MoveAlongPath>;
using RobTarget = waypoint_interfaces::msg::RobTarget;
using MoveInstruction = waypoint_interfaces::msg::MoveInstruction;
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
            vel_cmd_ = turtle_handler::toTwist(turtle_handler::computeVelCmd(turtle_handler::toPose(current_pose), turtle_handler::toMoveInstruction(current_instruction_)));
        }
        else{
            vel_cmd_.linear.x = 0;
            vel_cmd_.angular.z = 0;
        }
        vel_cmd_publisher_->publish(vel_cmd_);
    }

    rclcpp_action::GoalResponse goalCallback([[maybe_unused]] const rclcpp_action::GoalUUID uuid, std::shared_ptr<const MoveAlongPath::Goal> goal){
        auto path = goal -> path;
        for (std::size_t i=0; i < path.size(); i++){
            const MoveInstruction& instruction = path[i];
            std::string reason = validateInstruction_(instruction);
            if (!reason.empty()){
                RCLCPP_ERROR(this->get_logger(), "Instruction %zu rejected: %s", i, reason.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancelCallback([[maybe_unused]] const std::shared_ptr<MoveAlongPathGoalHandle>& goal_handle){
        RCLCPP_INFO(this->get_logger(), "Received a cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
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
    std::deque<std::shared_ptr<MoveAlongPathGoalHandle>> request_queue_;
    std::vector<MoveInstruction> active_path_;
    std::size_t path_index_ = 0;
    double dx_;
    double dy_;
    double remaining_dist_;

    void motionPlanner_(){
        
        while(true){
            if(!active_goal_ && !request_queue_.empty()){
                path_index_ = 0;
                active_goal_ = request_queue_.front();
            }

            if (!active_goal_)
                return;
            
            active_path_ = active_goal_ ->get_goal()->path;

            dx_ = current_pose_.x - active_path_[path_index_].target.x;
            dy_ = current_pose_.y - active_path_[path_index_].target.y;
            remaining_dist_ = sqrt(dx_*dx_+dy_*dy_);


            if (remaining_dist_<= active_path_[path_index_].zone_data)
                ++path_index_;

            if (path_index_ >= active_path_.size()){
                auto result = std::make_shared<MoveAlongPath::Result>();
                result -> message = "Finished path";
                request_queue_[0]->succeed(result);
                request_queue_.pop_front();
                active_goal_ = nullptr;
                
                continue;
            }

            current_instruction_ = active_goal_->get_goal()->path[path_index_];
            return;
        }
    }

    std::string validateInstruction_(const MoveInstruction& instruction) const{
        if (instruction.target.x > 11 || instruction.target.x < 0){
            return "Target out of range";
        }
        if (instruction.target.y > 11 || instruction.target.y < 0){
            return "Target out of range";
        }
        if (instruction.velocity <= 0.01){
            return "Velocity too low";
        }
        if (instruction.zone_data < 0 || instruction.zone_data > 5){
            return "Zone data too large";
        }
        return "";
    }

};


int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

