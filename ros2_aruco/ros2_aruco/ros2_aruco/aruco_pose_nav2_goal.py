'''
This is the /aruco_poses topic echo message - 
header:
  stamp:
    sec: 1612
    nanosec: 314000000
  frame_id: realsense_color_optical_frame
poses:
- position:
    x: 0.5086111256416823
    y: 0.9669190080342706
    z: 3.7320618000897716
  orientation:
    x: 0.7878575916124552
    y: -0.10085447731477504
    z: 0.07626659757951117
    w: 0.6027372527377335

Create a python node which takes the pose from /aruco_poses topic and gives the 2d goal pose to the mobile robot in the map frame. The map to realsense_color_optical_frame transformation is provided by /tf topic.
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from tf2_ros import Buffer, TransformListener
from tf2_ros.transform_listener import TransformException
from tf2_geometry_msgs import do_transform_pose
from nav2_simple_commander.robot_navigator import BasicNavigator

class ArucoPoseToNavGoal(Node):
    def __init__(self):
        super().__init__('aruco_pose_to_nav_goal')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/aruco_poses',
            self.aruco_pose_callback,
            10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigator = BasicNavigator()

    def aruco_pose_callback(self, msg):
        try:
            # Wait for the transformation to be available
            self.get_logger().info('Waiting for transform...')
            transform = self.tf_buffer.lookup_transform('map',
                                                        msg.header.frame_id,  # From the frame in the message
                                                        rclpy.time.Time())    # Get the latest available
            transformed_pose = do_transform_pose(msg, transform)
            self.send_goal(transformed_pose)
        except TransformException as e:
            self.get_logger().error(f'Could not transform from {msg.header.frame_id} to map: {e}')

    def send_goal(self, pose):
        self.get_logger().info(f'Sending goal to Navigator: {pose}')
        # Here we're assuming the pose is already in the correct frame and orientation
        self.navigator.goToPose(pose)
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            # Optional: Print feedback
        result = self.navigator.getResult()
        self.get_logger().info(f'Navigation result: {result}')

def main(args=None):
    rclpy.init(args=args)
    aruco_pose_to_nav_goal = ArucoPoseToNavGoal()
    rclpy.spin(aruco_pose_to_nav_goal)
    aruco_pose_to_nav_goal.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

