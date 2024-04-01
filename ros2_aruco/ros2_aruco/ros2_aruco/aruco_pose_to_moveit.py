import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray 
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose
import tf2_ros

class ArucoPoseToMoveit(Node):
    def __init__(self):
        super().__init__('aruco_pose_to_moveit')
        self.subscription = self.create_subscription(
            PoseArray,
            '/aruco_poses',
            self.aruco_pose_callback,
            10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        #self.goal_sent = False  # Flag to check if the goal has been sent
        self.publisher = self.create_publisher(PoseStamped, 'aruco_moveit_poses', 10)

    def aruco_pose_callback(self, msg):
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.poses[0]
        
        """ if self.goal_sent:
            return  # If a goal has already been sent, do nothing """
        try:
            # Wait for the transformation to be available
            #self.get_logger().info('Waiting for transform...')
            transform = self.tf_buffer.lookup_transform('ur_base_link',
                                                        msg.header.frame_id,  # From the frame in the message
                                                        rclpy.time.Time())    # Get the latest available
            #self.get_logger().info('PoseStamped: %s', pose_stamped)
            transformed_pose = do_transform_pose(pose_stamped.pose, transform)

            # Create a new PoseStamped object with the transformed pose
            new_pose_stamped = PoseStamped()
            new_pose_stamped.header = pose_stamped.header  # Reuse the original header
            new_pose_stamped.header.frame_id = "ur_base_link"  # Set to the target frame
            new_pose_stamped.pose = transformed_pose
            self.publisher.publish(new_pose_stamped)
            #self.goal_sent = True  # Set the flag to True after sending the goal
        except tf2_ros.LookupException:
            self.get_logger().error('LookupException caught')
        except tf2_ros.ConnectivityException:
            self.get_logger().error('ConnectivityException caught')
        except tf2_ros.ExtrapolationException:
            self.get_logger().error('ExtrapolationException caught')
        except TransformException as e:
            self.get_logger().error(f'Could not transform from {msg.header.frame_id} to ur_base_link: {e}')
        

def main(args=None):
    rclpy.init(args=args)
    aruco_pose_to_moveit = ArucoPoseToMoveit()
    rclpy.spin(aruco_pose_to_moveit)
    # No longer need to explicitly call shutdown here since it's handled after navigation completes
    aruco_pose_to_moveit.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()