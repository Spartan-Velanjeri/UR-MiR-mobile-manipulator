import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudSubscriber(Node):

    def __init__(self):
        super().__init__('point_cloud_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/realsense/depth/color/points',
            self.callback,
            10
        )
        self.publisher = self.create_publisher(
            PointCloud2,
            '/processed_point_cloud',
            10
        )

    def callback(self,msg):
        self.get_logger().info('Received a point cloud')
        # processed_msg = self.process_point_cloud(msg)
        center = self.compute_centroid(msg)
        processed_msg = self.create_point_cloud(center)
        if processed_msg is not None:
            self.publisher.publish(processed_msg)
    
    def process_point_cloud(self,msg):
        points = list(pc2.read_points(msg,skip_nans=True,field_names=('x','y','z')))
        print(len(points))
        processed_points = [(x,y,z+0.1) for x,y,z in points]

        processed_msg = pc2.create_cloud_xyz32(msg.header,processed_points)
        return processed_msg
    
    def compute_centroid(self,msg):
        # points = list(pc2.read_points(msg,skip_nans=True,field_names=('x','y','z')))
        points = np.array([p[0],p[1],p[2]] for p in pc2.read_points(msg,skip_nans=True,field_names=('x','y','z')))
        print(points)
        if points.shape != ():
            # print(points.shape)
            center = np.mean(points,axis=0)
            print(center)
        else:
            center = None
        return center

    def create_point_cloud(self,center):
        if center != None:
            fields = [
                pc2.PointField('x',0,pc2.PointField.FLOAT32,1),
                pc2.PointField('y',4,pc2.PointField.FLOAT32,1),
                pc2.PointField('z',8,pc2.PointField.FLOAT32,1),
            ]
            points = [[center[0],center[1],center[2]]]
            center_msg = pc2.create_cloud_xyz32(msg.header,fields,points)
        else:
            center_msg = None
        return center_msg


def main(args=None):
    rclpy.init(args=args)
    point_cloud_subscriber = PointCloudSubscriber()
    rclpy.spin(point_cloud_subscriber)
    point_cloud_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
