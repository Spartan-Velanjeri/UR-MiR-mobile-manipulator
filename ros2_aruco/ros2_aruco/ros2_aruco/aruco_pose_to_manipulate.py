import time
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseArray

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose

# from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters

class ArucoPoseToManipulate(Node):
    def __init__(self):
        super().__init__('aruco_pose_to_manipulate')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/aruco_poses',
            self.aruco_pose_callback,
            10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)
        self.universal_robot = MoveItPy(node_name="moveit_py")
        self.ur_manipulator = universal_robot.get_planning_component("ur_manipulator")
        self.get_logger().info("MoveItPy instance created")

    def aruco_pose_callback(self, msg):
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.poses[0]
        
        if self.goal_sent:
            return  # If a goal has already been sent, do nothing
        try:
            # Wait for the transformation to be available
            self.get_logger().info('Waiting for transform...')
            transform = self.tf_buffer.lookup_transform('map',
                                                        msg.header.frame_id,  # From the frame in the message
                                                        rclpy.time.Time())    # Get the latest available
            #self.get_logger().info('PoseStamped: %s', pose_stamped)
            transformed_pose = do_transform_pose(pose_stamped.pose, transform)

            # Create a new PoseStamped object with the transformed pose
            new_pose_stamped = PoseStamped()
            new_pose_stamped.header = pose_stamped.header  # Reuse the original header
            new_pose_stamped.header.frame_id = "map"  # Set to the target frame
            new_pose_stamped.pose = transformed_pose
            # Send the goal which is 0.5m behind the marker in both x and y directions irresespective of the marker orientation
            # Take absolute value of x,y and subtract 0.5, then multiply by the sign of x,y
            self.send_manipulation_goal(new_pose_stamped)
            self.goal_sent = True  # Set the flag to True after sending the goal
        except tf2_ros.LookupException:
            self.get_logger().error('LookupException caught')
        except tf2_ros.ConnectivityException:
            self.get_logger().error('ConnectivityException caught')
        except tf2_ros.ExtrapolationException:
            self.get_logger().error('ExtrapolationException caught')
        except TransformException as e:
            self.get_logger().error(f'Could not transform from {msg.header.frame_id} to map: {e}')

    def plan_and_execute(
        self,
        robot,
        planning_component,
        single_plan_parameters=None,
        multi_plan_parameters=None,
        sleep_time=0.0,
    ):
        """Helper function to plan and execute a motion."""
        # plan to goal
        self.get_logger().info("Planning trajectory")
        if multi_plan_parameters is not None:
            plan_result = planning_component.plan(
                multi_plan_parameters=multi_plan_parameters
            )
        elif single_plan_parameters is not None:
            plan_result = planning_component.plan(
                single_plan_parameters=single_plan_parameters
            )
        else:
            plan_result = planning_component.plan()

        # execute the plan
        if plan_result:
            self.get_logger().info("Executing plan")
            robot_trajectory = plan_result.trajectory
            robot.execute(robot_trajectory, controllers=[])
        else:
            self.get_logger().info.error("Planning failed")

        time.sleep(sleep_time)
    
    def initial_feedback_pose_goal(self):
        self.ur_manipulator.set_start_state_to_current_state()
        self.ur_manipulator.set_goal_state(configuration_name = "feedback")
        plan_and_execute(self.universal_robot,self.ur_manipulator)
    
    def send_manipulation_goal(self,goal):
        self.ur_manipulator.set_start_state_to_current_state()
        goal.pose.position.z = 0.5
        self.ur_manipulator.set_goal_state(pose_stamped_msg=goal, pose_link=goal.header.frame_id)
        plan_and_execute(self.universal_robot,self.ur_manipulator)


def main(args=None):
    rclpy.init(args=args)
    aruco_pose_to_manipulate = ArucoPoseToManipulate()
    aruco_pose_to_manipulate.initial_feedback_pose_goal()
    rclpy.spin(aruco_pose_to_manipulate)
    aruco_pose_to_manipulate.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
