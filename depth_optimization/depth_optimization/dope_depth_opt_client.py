
from depth_optimization_interfaces.srv import DopeDepthOptimize
from geometry_msgs.msg import PoseStamped, PoseArray, Pose

import rclpy
from rclpy.node import Node
import time
import sys




class SimpleDepthOptClient(Node):

    def __init__(self):
        super().__init__('dope_optimization_client')
        self.cli = self.create_client(DopeDepthOptimize, 'dope_depth_optimize')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = DopeDepthOptimize.Request()

    def send_request(self, class_id, n_max_poses, optimize):
        self.req.class_id = class_id
        self.req.n_max_poses = n_max_poses
        self.req.optimize = optimize

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        
        return self.future.result()
    


def main(args=None):
    rclpy.init(args=args)

    minimal_client = SimpleDepthOptClient()

    # get the class_id and n_max_poses from the command line
    class_id = int(sys.argv[1])
    n_max_poses = int(sys.argv[2])
    optimize = bool(sys.argv[3])
    response = minimal_client.send_request(class_id, n_max_poses, optimize)
    
    for pose in response.refined_poses:
        print("RESPONSE RECEIVED")
        print("refined position:")
        print("frame_id = " + str(pose.header.frame_id))
        print("x = " + str(pose.pose.position.x))
        print("y = " + str(pose.pose.position.y))
        print("z = " + str(pose.pose.position.z))
        
    for scale_obj in response.scale_obj:
        print("scale factor = " + str(scale_obj))


    if len(response.refined_poses) == 0:
        print("No poses found for class_id = " + str(class_id))
        return
    
    # publish the refined poses
    refined_pose_pub = minimal_client.create_publisher(PoseArray, 'refined_poses', 1) 
    pose_array = PoseArray()
    pose_array.poses = [Pose() for i in range(len(response.refined_poses))]
    pose_array.header = response.refined_poses[0].header
    
    i = 0
    for pose in response.refined_poses:
        pose_array.poses[i] =  pose.pose
        i = i + 1

    # publish the refined poses
    while rclpy.ok():
        refined_pose_pub.publish(pose_array)
        time.sleep(1)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()