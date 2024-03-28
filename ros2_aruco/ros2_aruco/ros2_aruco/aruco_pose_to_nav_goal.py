import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray 
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose
from nav2_simple_commander.robot_navigator import BasicNavigator
import tf2_ros

class ArucoPoseToNavGoal(Node):
    def __init__(self):
        super().__init__('aruco_pose_to_nav_goal')
        self.subscription = self.create_subscription(
            PoseArray,
            '/aruco_poses',
            self.aruco_pose_callback,
            10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigator = BasicNavigator()
        self.goal_sent = False  # Flag to check if the goal has been sent

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
            result = lambda x : x-1.0 if x>0 else x+1.0
            #new_pose_stamped.pose.position.x = result(new_pose_stamped.pose.position.x)
            new_pose_stamped.pose.position.y = result(new_pose_stamped.pose.position.y) 
            self.send_goal(new_pose_stamped)
            self.goal_sent = True  # Set the flag to True after sending the goal
        except tf2_ros.LookupException:
            self.get_logger().error('LookupException caught')
        except tf2_ros.ConnectivityException:
            self.get_logger().error('ConnectivityException caught')
        except tf2_ros.ExtrapolationException:
            self.get_logger().error('ExtrapolationException caught')
        except TransformException as e:
            self.get_logger().error(f'Could not transform from {msg.header.frame_id} to map: {e}')

    def send_goal(self, pose):
        self.navigator.waitUntilNav2Active(localizer="bt_navigator")
        self.get_logger().info(f'Sending goal to Navigator: {pose}')
        # Here we're assuming the pose is already in the correct frame and orientation
        self.navigator.goToPose(pose)

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)  # Allow ROS2 to process incoming messages
            feedback = self.navigator.getFeedback()
            # Optional: Print feedback
        result = self.navigator.getResult()
        self.get_logger().info(f'Navigation result: {result}')

        # After task completes, shut down
        self.shutdown_node()
    
    def shutdown_node(self):
        self.get_logger().info('Shutting down node...')
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    aruco_pose_to_nav_goal = ArucoPoseToNavGoal()
    rclpy.spin(aruco_pose_to_nav_goal)
    # No longer need to explicitly call shutdown here since it's handled after navigation completes
    #aruco_pose_to_nav_goal.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()