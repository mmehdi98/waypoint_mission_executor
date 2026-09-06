#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "waypoint_interfaces/action/move_along_path.hpp"

using MoveAlongPath = waypoint_interfaces::action::MoveAlongPath;
using MoveAlongPathGoalHandle = rclcpp_action::ClientGoalHandle<MoveAlongPath>;
using MoveInstruction = waypoint_interfaces::msg::MoveInstruction;
using namespace std::placeholders;

class MissionClient : public rclcpp::Node
{
public:
    MissionClient() : Node("mission_client"){
        mission_client_ = rclcpp_action::create_client<MoveAlongPath>(this, "move_along_path");
    }

    void sendGoal(const std::vector<MoveInstruction>& path){
        mission_client_->wait_for_action_server();
        auto goal = MoveAlongPath::Goal();
        goal.path = path;
        auto options = rclcpp_action::Client<MoveAlongPath>::SendGoalOptions();
        options.goal_response_callback = std::bind(&MissionClient::goalResponseCallback_, this, _1);
        options.result_callback = std::bind(&MissionClient::goalResultCallback_, this, _1);
        options.feedback_callback = std::bind(&MissionClient::goalFeedbackCallback_, this, _1, _2);
        mission_client_->async_send_goal(goal, options);
    }

    void cancelGoal(){
        RCLCPP_INFO(this->get_logger(), "Sending a cancel goal request");
        mission_client_ ->async_cancel_all_goals();
    }

private:
    rclcpp_action::Client<MoveAlongPath>::SharedPtr mission_client_;

    void goalResponseCallback_(const MoveAlongPathGoalHandle::SharedPtr goal_handle){
        if (!goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Goal got rejected");
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Goal got accepted");
        }
    }

    void goalResultCallback_(const MoveAlongPathGoalHandle::WrappedResult result){
        auto status = result.code;
        if (status == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Succeeded");
        }
        else if (status == rclcpp_action::ResultCode::ABORTED) {
            RCLCPP_ERROR(this->get_logger(), "Aborted");
        }
        else if (status == rclcpp_action::ResultCode::CANCELED) {
            RCLCPP_WARN(this->get_logger(), "Canceled");
        }
        std::string message = result.result -> message;
        RCLCPP_INFO(this->get_logger(), "Result: %s", message.c_str());
        rclcpp::shutdown();
    }

    void goalFeedbackCallback_([[maybe_unused]]const MoveAlongPathGoalHandle::SharedPtr& goal_handle, const std::shared_ptr<const MoveAlongPath::Feedback> feedback){
        RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(), 1000, "Current Position: (x=%f, y=%f, theta=%f)", feedback->current_position.x, feedback->current_position.y, feedback->current_position.theta);
        RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(), 1000, "Active Target: (x=%f, y=%f, theta=%f)", feedback->current_target.x, feedback->current_target.y, feedback->current_target.theta);
        RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(), 1000, "Remaining Distance: %f", feedback->distance_to_next);
    }
};

static waypoint_interfaces::msg::MoveInstruction make_wp(double x, double y, double theta, double velocity, double zone_data)
{
  waypoint_interfaces::msg::MoveInstruction m;
  m.target.x = x;
  m.target.y = y;
  m.target.theta = theta;
  m.velocity = velocity;
  m.zone_data = zone_data;
  return m;
}

int main(int argc, char** argv){
    std::vector<MoveInstruction> path  = {
        make_wp(4.0, 4.0, 0.2, 1.0, 0.1),
        make_wp(8.0, 4.0, 0.4, 1.0, 0.5),
        make_wp(8.0, 9.0, 0.6, 0.5, 0.0)
    };

    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionClient>();
    node->sendGoal(path);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}