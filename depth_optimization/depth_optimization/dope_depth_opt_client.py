
from depth_optimization_interfaces.srv import DopeDepthOptimize
from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.node import Node



class SimpleDepthOptClient(Node):

    def __init__(self):
        super().__init__('simple_depth_client')
        self.cli = self.create_client(DopeDepthOptimize, 'dope_depth_optimize')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = DopeDepthOptimize.Request()

    def send_request(self):
        self.req.class_id = 5
        self.req.n_max_poses = 1
        self.req.optimize = True

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        
        return self.future.result()
    


def main(args=None):
    rclpy.init(args=args)

    minimal_client = SimpleDepthOptClient()

    # publisher to publish the refined pose
    #refined_pose_pub = minimal_client.create_publisher(PoseStamped, 'refined_pose', 1)


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