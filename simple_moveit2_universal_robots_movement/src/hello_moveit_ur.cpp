#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"

class PoseSubscriberNode : public rclcpp::Node {
public:
    PoseSubscriberNode() : Node("pose_subscriber_node") {
        aruco_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "aruco", 10, std::bind(&PoseSubscriberNode::poseCallback, this, std::placeholders::_1));

        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>("ur_manipulator");
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received pose: [%f, %f, %f]",
                    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

        // Set the target pose
        move_group_->setPoseTarget(msg->pose);

        // Plan the trajectory
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(this->get_logger(), "Planning successful. Executing...");
            move_group_->execute(my_plan);
            RCLCPP_INFO(this->get_logger(), "Execution complete.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_subscriber_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
