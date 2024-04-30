import sys

from depth_optimization_interfaces.srv import DepthOptimize
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

import rclpy
from rclpy.node import Node
import time
import cv2
from cv_bridge import CvBridge
import numpy as np


class SimpleDepthOptClient(Node):

    # camera parameters
    image_width = 640
    image_height = 480
    depth_scale = 0.001

    # variables to store the pose and depth
    pose_read = PoseStamped()
    depth_read = Image()
    depth_array = np.zeros(image_width * image_height, dtype=np.float64)
    



    def __init__(self):
        super().__init__('simple_depth_client')
        self.cli = self.create_client(DepthOptimize, 'depth_optimize')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = DepthOptimize.Request()

    def send_request(self):
        self.req.estimated_pose.header = self.pose_read.header
        self.req.estimated_pose.pose = self.pose_read.pose

         # compute the depth array
        self.compute_depth()
        self.req.depth_matrix = list(self.depth_array.flatten().astype(np.float64))
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        
        return self.future.result()
    
    def pose_callback(self, msg):
        # Process the received pose message
        self.pose_read = msg
        print("Pose received")
        return
    
    def depth_callback(self, msg_d):
        # Process the received depth message
        self.depth_read = msg_d
        print("Depth received")
        return
    
    def compute_depth(self):
        # this method converts the depth image to a depth array in meters
        bridge = CvBridge()

        current_depth = np.zeros((self.image_height, self.image_width), dtype=np.float64)
        current_depth = bridge.imgmsg_to_cv2(self.depth_read, self.depth_read.encoding)
        
        #clear the depth array
        self.depth_array = np.zeros(self.image_width * self.image_height, dtype=np.float64)

        for i in range(self.image_height):
            for j in range(self.image_width):
                self.depth_array[i * self.image_width + j] += current_depth[i, j] * np.float64(self.depth_scale)

        return



def main(args=None):
    rclpy.init(args=args)

    minimal_client = SimpleDepthOptClient()

    # topics where the estimated pose and depth are published
    pose_topic = '/dope/pose_fork'
    depth_topic = '/camera/aligned_depth_to_color/image_raw'


    # read the estimated pose
    pose_subscriber = minimal_client.create_subscription(
        PoseStamped,
        pose_topic,
        minimal_client.pose_callback,
        1
    )

    depth_subscriber = minimal_client.create_subscription(
        Image,
        depth_topic,
        minimal_client.depth_callback,
        1
    )
    
    # spin to read some messages
    N = 10
    for _ in range(N):
        rclpy.spin_once(minimal_client)
        time.sleep(0.1)
   


    # publisher to publish the refined pose
    refined_pose_pub = minimal_client.create_publisher(PoseStamped, 'refined_pose', 1)

    # send the requests
    time.sleep(1)

    while True:
        response = minimal_client.send_request()

        print("RESPONSE RECEIVED")
        print("refined position:")
        print("x = " + str(response.refined_pose.pose.position.x))
        print("y = " + str(response.refined_pose.pose.position.y))
        print("z = " + str(response.refined_pose.pose.position.z))

        print("scale factor = " + str(response.scale_obj))

        # publish the refined pose
        refined_pose_pub.publish(response.refined_pose)

        # rclpy.spin_once(minimal_client, timeout_sec=10.0)
        for _ in range(N):
            rclpy.spin_once(minimal_client)
            time.sleep(0.1)


        user_input = input("Do you want to continue? (y/n): ")
        if user_input.lower() != 'y':
            N = 30
            for _ in range(N):
                rclpy.spin_once(minimal_client)
                time.sleep(0.1)
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()