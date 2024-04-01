#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur_move_group");

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv)
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true)
    auto move_group_node = rclcpp::Node::make_shared("move_it_aruco",node_options)

    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);   
    std::thread([&executor]() { executor.spin(); }).detach();
    static const std::string PLANNING_GROUP = "ur_manipulator";

    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);


    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
                std::ostream_iterator<std::string>(std::cout, ", "));


    rclcpp::shutdown();
    return 0;

}