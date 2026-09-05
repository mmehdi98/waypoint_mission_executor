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
        mission_client_->async_send_goal(goal, options);
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
};

int main(int argc, char** argv){
    MoveInstruction instruction;
    instruction.target.x = 3;
    instruction.target.y = 2;
    instruction.target.theta = 0.5;
    instruction.velocity = 1.0;
    instruction.zone_data = 0.2;
    std::vector<MoveInstruction> path;
    path.push_back(instruction);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionClient>();
    node->sendGoal(path);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}