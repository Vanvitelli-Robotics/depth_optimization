
from __future__ import print_function
# ros2 pkgs
from depth_optimization_interfaces.srv import DopeDepthOptimize
from geometry_msgs.msg import PoseStamped
import rclpy.parameter_service
import rclpy.utilities
from vision_msgs.msg import Detection3D, Detection3DArray
from sensor_msgs.msg import Image

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

# python modules
import nvisii
import numpy as np
from scipy.optimize import minimize_scalar
import matplotlib.pyplot as plt
from sklearn import linear_model
from numpy import linalg as LA
import time
import os
from cv_bridge import CvBridge
import cv2

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import resource_retriever



"""
This node refines the estimated pose from a 6D pose algorithm estimator through depth measurements solving an optimization problem. 
The implementation is coupled with the pkg DOPE (Deep Object Pose Estimation) for 6D pose estimation.  
The service server reads from /dope_node/detected_objects topic the estimated pose and the depth values from the /aligned_rgb_to_depth_image topic, and uses the class DepthOptimizer 
to refine the pose. The optimization problem is solved by minimizing the cost function, which is the sum of the squared differences between the real depth values and the virtual depth values
Input: class_id (int), n_max_optimization (int >=1), optimize (bool)
Output: refined_poses (PoseStamped[]), scale_obj (float[]), success (bool[])
If the optimization is required (optimize = True), the node will return the refined poses and the scale of the object in the scene.
"""

class DepthOptimizer():

    # camera width, height and intrinsics
    width_ = 640
    height_ = 480
    fx = 610.59326171875 
    fy = 610.605712890625
    cx = 317.7075500488281
    cy = 238.1421356201172
    focal_length = 0  

    # Object parameters
    object_name = "object_entity"
    mesh_path = []    
    mesh_scale = []

    # [m] max depth value captured by the virtual camera in meters
    max_virtual_depth = 5 

    estimated_pose = PoseStamped()

    # Optimization parameters
    pixel_cad_h = [] # Pixel coordinates of the object in the virtual scene
    pixel_cad_w = [] # Pixel coordinates of the object in the virtual scene
    real_useful_depth = [] # Real depth values of the pixels corresponding to the object in the virtual scene
    tol_optimization = 0.1 # Tolerance for optimization
    remove_outliers = True # Remove outliers with RANSAC regression
    bound_scale = 0.8   # Bound scale for sigma_min and sigma_max in optimization

    # Service
    initialized_scene = False
    interactive = True


    # methods
    def __init__(self, camera_parameters, object_parameters, optimization_parameters):
        print("Depth Optimizer Object Created")
        self.width_ = camera_parameters['width_']
        self.height_ = camera_parameters['height_']
        self.fx = camera_parameters['fx']
        self.fy = camera_parameters['fy']
        self.cx = camera_parameters['cx']
        self.cy = camera_parameters['cy']
        self.focal_length = camera_parameters['focal_length']
        self.mesh_path = object_parameters['mesh_path']
        self.mesh_scale = object_parameters['mesh_scale']
        self.max_virtual_depth = optimization_parameters['max_virtual_depth']
        self.tol_optimization = optimization_parameters['tol_optimization']
        self.remove_outliers = optimization_parameters['remove_outliers']
        self.bound_scale = optimization_parameters['bound_scale']
        

    def change_mesh(self, object_parameters):
        print("Changing mesh...")
        self.mesh_path = object_parameters['mesh_path']
        self.mesh_scale = object_parameters['mesh_scale']
        self.reset_scene(True)


        return SetParametersResult(successful=True)

    def scene_initialization_nvisii(self):
        nvisii.initialize(headless=not self.interactive, verbose=True)
        nvisii.disable_updates()
        self.populate_scene()

    def get_useful_pixels(self,real_depth_array):
        # This function selects the pixels corresponding to cad object in the virtual scene and corresponding real depth values
        self.virtual_depth_map(0)
        for i in range(self.height_):
            for j in range(self.width_):
                if self.virtual_depth_array[i, j, 0] < self.max_virtual_depth and self.virtual_depth_array[i, j, 0] > 0:
                    self.pixel_cad_h.append(i)
                    self.pixel_cad_w.append(j)


        for i in range(len(self.pixel_cad_h)):
            self.real_useful_depth.append(
                real_depth_array[self.pixel_cad_h[i]*self.width_ + self.pixel_cad_w[i]])
        
    def virtual_depth_map(self,sigma):
        print("generating virtual depth map...", sigma)

        obj_mesh = nvisii.entity.get(self.object_name)

        x = self.estimated_pose.pose.position.x
        y = -self.estimated_pose.pose.position.y
        z = -self.estimated_pose.pose.position.z

        p = np.array([x, y, z])
        f = np.array([0, 0, self.focal_length])

        p_new = p + np.multiply(np.divide((f-p), LA.norm(p-f)), sigma)
        scale_obj = LA.norm(p_new-f)/LA.norm(p-f)

        obj_mesh.get_transform().set_position(p_new)

        rotation_flip = nvisii.angleAxis(-nvisii.pi(),nvisii.vec3(1,0,0)) * nvisii.quat(self.estimated_pose.pose.orientation.w,
                                                        self.estimated_pose.pose.orientation.x, self.estimated_pose.pose.orientation.y, self.estimated_pose.pose.orientation.z)
        obj_mesh.get_transform().set_rotation(rotation_flip)

        obj_mesh.get_transform().set_scale(nvisii.vec3(scale_obj*self.mesh_scale))
        
        
        self.virtual_depth_array = nvisii.render_data(
            width=int(self.width_),
            height=int(self.height_),
            start_frame=0,
            frame_count=1,
            bounce=int(0),
            options="depth"
        )

        self.virtual_depth_array = np.array(
            self.virtual_depth_array).reshape(self.height_, self.width_, 4)
        self.virtual_depth_array = np.flipud(self.virtual_depth_array)

    
    def delete_outliers(self):
        line_ransac = linear_model.RANSACRegressor()
        line_ransac.fit(np.linspace(0, 1, len(self.real_useful_depth)
                                    ).reshape(-1, 1), self.real_useful_depth)
        inlier_mask = line_ransac.inlier_mask_
        outlier_mask = np.logical_not(inlier_mask)
        for i in range(len(outlier_mask)):
            if (outlier_mask[i] == True):
                self.real_useful_depth[i] = 0.0
    
    def convert_from_uvd(self,v, u, d, fx, fy, cx, cy):
        x_over_z = (cx - u) / fx
        y_over_z = (cy - v) / fy
        z = d / np.sqrt(1. + x_over_z**2 + y_over_z**2)
        return z
    

    def cost_function(self,sigma):
        tic1 = time.perf_counter()

        # Create virtual depth map based on estimated pose and actual sigma
        self.virtual_depth_map(sigma)
        toc1 = time.perf_counter()
        print(f"virtual depth map realized in {toc1 - tic1:0.4f} seconds")

        sum = 0
        num_pixel_obj = 0
        tic1 = time.perf_counter()
        for k in range(len(self.pixel_cad_h)):
            if (self.real_useful_depth[k] != 0):  # if depth value isn't an outlier
                i = self.pixel_cad_h[k]
                j = self.pixel_cad_w[k]

        
                d_hat = self.convert_from_uvd(i, j, self.virtual_depth_array[i, j, 0], self.fx, self.fy, self.cx, self.cy)
                d = self.real_useful_depth[k]

                sum = sum + pow((d-d_hat), 2)
                num_pixel_obj = num_pixel_obj+1
        toc1 = time.perf_counter()
        print(f"evaluation cost function realized in {toc1 - tic1:0.4f} seconds")
        try:
            return sum/num_pixel_obj
        except ZeroDivisionError:
            print("Error: Division by zero")


    def depth_optimize(self, estimated_pose, depth_matrix):

        print('Depth optimization started...')
        tic = time.perf_counter()

        optimized_pose = PoseStamped()

        # Getting estimated object pose
        self.estimated_pose = estimated_pose

        # Initialize the virtual scene
        if not self.initialized_scene:
            tic1 = time.perf_counter()
            self.scene_initialization_nvisii()
            toc1 = time.perf_counter()
            print(f"NVISII initialization realized in {toc1 - tic1:0.4f} seconds")
            self.initialized_scene = True

        # Getting useful pixels
        tic1 = time.perf_counter()
        self.get_useful_pixels(depth_matrix)
        toc1 = time.perf_counter()
        print(f"get useful pixels realized in {toc1 - tic1:0.4f} seconds")

        
        # If desired, remove outliers with RANSAC regression
        if self.remove_outliers:
            tic1 = time.perf_counter()
            self.delete_outliers()
            toc1 = time.perf_counter()
            print(f"remove outliers realized in {toc1 - tic1:0.4f} seconds")

        # Minimize objective function
        sigma_min = -self.estimated_pose.pose.position.z * self.bound_scale
        sigma_max = +self.estimated_pose.pose.position.z * self.bound_scale
        tic1 = time.perf_counter()
        res = minimize_scalar(self.cost_function, bounds=[
                            sigma_min, sigma_max], tol=self.tol_optimization)
        toc1 = time.perf_counter()
        print(f"minimization realized in {toc1 - tic1:0.4f} seconds")

        toc = time.perf_counter()
        print(f"Optimization realized in {toc - tic:0.4f} seconds")


        if res.fun > self.tol_optimization:
            success = False
        else:
            success = True

      
        x = self.estimated_pose.pose.position.x
        y = self.estimated_pose.pose.position.y
        z = self.estimated_pose.pose.position.z
        p = np.array([x, y, z])
        f = np.array([0, 0, self.focal_length])
        p_new = p + np.multiply(np.divide((f-p), LA.norm(p-f)), res.x)

        optimized_pose.pose.position.x = p_new[0]
        optimized_pose.pose.position.y = p_new[1]
        optimized_pose.pose.position.z = p_new[2]
        optimized_pose.header.frame_id = self.estimated_pose.header.frame_id
        optimized_pose.pose.orientation = self.estimated_pose.pose.orientation

        scale_obj = LA.norm(p_new-f)/LA.norm(p-f)



        print("OPTIMIZATION TERMINATED:\n")    
        print("Function_min: ", res.fun)
        print("Sigma_min [m]: ", res.x)
        print("Optimized Pose: ", optimized_pose)
        print("Scale Object: ", scale_obj)
        print("Success: ", success)

        # Clear variables for next service call
        self.reset_scene()
        

        return optimized_pose, scale_obj, success
    
    def reset_scene(self, update_mesh=False):
        if self.initialized_scene and update_mesh:
            nvisii.clear_all()
            self.populate_scene()
        self.pixel_cad_h = []
        self.pixel_cad_w = []
        self.real_useful_depth = []
        self.virtual_depth_array = []
        self.estimated_pose = PoseStamped()

    def close_scene(self):
        nvisii.deinitialize()
        self.initialized_scene = False

    def populate_scene(self):
        camera = nvisii.entity.create(
            name="camera",
            transform=nvisii.transform.create("camera"),
            camera=nvisii.camera.create_from_intrinsics(
                name="camera",
                fx=self.fx,
                fy=self.fy,
                cx=self.cx,
                cy=self.cy,
                width=self.width_,
                height=self.height_
            )
        )
        camera.get_transform().look_at(
            at=(0, 0, 0),
            up=(0, -1, -1),
            eye=(1, 1, 0)
        )
        nvisii.set_camera_entity(camera)
        
        obj_mesh = nvisii.entity.create(
            name=self.object_name,
            mesh=nvisii.mesh.create_from_file(self.object_name, self.mesh_path),
            transform=nvisii.transform.create(self.object_name),
            material=nvisii.material.create(self.object_name)
        )

        obj_mesh.get_transform().set_parent(camera.get_transform())

        nvisii.sample_pixel_area(
            x_sample_interval=(.5, .5),
            y_sample_interval=(.5, .5)
        )


class DopeDepthOptimizerServer(Node):
    camera_parameters = {
        'width_': 640,
        'height_': 480,
        'fx': 610.59326171875,
        'fy': 610.605712890625,
        'cx': 317.7075500488281,
        'cy': 238.1421356201172,
        'focal_length': 0,
        'depth_scale' : 0.001
    } 
    object_parameters = {
        'mesh_path': [],
        'mesh_scale': []
    }  
    optimization_parameters = {
        'max_virtual_depth': 5,
        'tol_optimization': 0.1,
        'remove_outliers': True,
        'bound_scale': 0.8
    }
    
    dope_sub = None
    depth_sub = None
    dope_msg_received = False
    depth_msg_received = False
    
    detection_msg = Detection3DArray()
    depth_msg = Image()

    def __init__(self):
        super().__init__('dope_depth_optimizer_server', automatically_declare_parameters_from_overrides=True)
        # read configuration parameters from yaml file
        self.read_parameters()

        # create subscribers
        subscribers_callback_grp = MutuallyExclusiveCallbackGroup()
        self.dope_sub = self.create_subscription(Detection3DArray, self.dope_detection_topic, self.dope_callback, 1, callback_group=subscribers_callback_grp)
        self.depth_sub = self.create_subscription(Image, self.camera_topic_aligned_depth_to_color, self.depth_callback, 1, callback_group=subscribers_callback_grp)
       

        # create a DepthOptimizer object with the camera parameters, object parameters and optimization parameters
        self.depth_optimizer = DepthOptimizer(self.camera_parameters, self.object_parameters, self.optimization_parameters)

        # create the service
        self.srv = self.create_service(DopeDepthOptimize, 'dope_depth_optimize', self.dope_depth_optimize)

    def read_parameters(self):
        self.get_logger().info("Reading parameters")

        # read topics
        self.dope_detection_topic = self.get_parameter('dope_detection_topic').get_parameter_value().string_value
        self.camera_topic_aligned_depth_to_color = self.get_parameter('camera_topic_aligned_depth_to_color').get_parameter_value().string_value

        
        # read parameters recognized objects
        self.class_ids = self.get_parameters_by_prefix('class_ids').items()

        for name, class_id in self.class_ids:
            print(name)
            print(class_id.get_parameter_value().integer_value)

        self.meshes = self.get_parameters_by_prefix('meshes').items()
        for name, mesh in self.meshes:
            print(name)
            print(mesh.get_parameter_value().string_value)

        self.mesh_scales = self.get_parameters_by_prefix('mesh_scales').items()
        for name, mesh_scale in self.mesh_scales:
            print(name)
            print(mesh_scale.get_parameter_value().double_value)

        # read camera parameters
        self.camera_parameters['width_'] = self.get_parameter('width_').get_parameter_value().integer_value
        self.camera_parameters['height_'] = self.get_parameter('height_').get_parameter_value().integer_value
        self.camera_parameters['fx'] = self.get_parameter('fx').get_parameter_value().double_value
        self.camera_parameters['fy'] = self.get_parameter('fy').get_parameter_value().double_value
        self.camera_parameters['cx'] = self.get_parameter('cx').get_parameter_value().double_value
        self.camera_parameters['cy'] = self.get_parameter('cy').get_parameter_value().double_value
        self.camera_parameters['focal_length'] = self.get_parameter('focal_length').get_parameter_value().double_value
        self.camera_parameters['depth_scale'] = self.get_parameter('depth_scale').get_parameter_value().double_value

        # read optimization parameters
        self.optimization_parameters['max_virtual_depth'] = self.get_parameter('max_virtual_depth').get_parameter_value().double_value
        self.optimization_parameters['tol_optimization'] = self.get_parameter('tol_optimization').get_parameter_value().double_value    
        self.optimization_parameters['remove_outliers'] = self.get_parameter('remove_outliers').get_parameter_value().bool_value
        self.optimization_parameters['bound_scale'] = self.get_parameter('bound_scale').get_parameter_value().double_value

    def get_mesh_parameters(self, class_id):
        for name, class_id_param in self.class_ids:
            if class_id_param.get_parameter_value().integer_value == class_id:
               object_name = name
               print("Selected object: ", object_name)
               break
        else:
            self.get_logger().error("Class id not found in parameters")
            raise Exception("Class id not found in parameters") 
            
        
        for name, mesh in self.meshes:
            if name == object_name:
                mesh_path = mesh.get_parameter_value().string_value
                print("Mesh path: ", mesh_path)
                break
        else:
            self.get_logger().error("Mesh path not found in parameters")
            raise Exception("Mesh path not found in parameters") 
        
        for name, mesh_scale in self.mesh_scales:
            if name == object_name:
                mesh_scale = mesh_scale.get_parameter_value().double_value
                print("Mesh scale: ", mesh_scale)
                break
        else:
            self.get_logger().error("Mesh scale not found in parameters")           
            raise Exception("Mesh scale not found in parameters") 

        return object_name, mesh_path, mesh_scale

    def dope_callback(self, msg):
        self.detection_msg = msg
        #print("Dope msg read")
        self.dope_msg_received = True
        

    def depth_callback(self, msg):
        self.depth_msg = msg
        #print("Depth msg read")
        self.depth_msg_received = True

    def get_detection(self, msg, class_id, n_max_poses):
       # return the poses of the detected objects with the class id, for a max of n_max_poses, ordered on the score value
        class_id = str(class_id)
        poses = []
        scores = []
        pose_stamped = PoseStamped()

        for detection in msg.detections:
            for result in detection.results:
                if result.hypothesis.class_id == class_id:
                    print("Class id found")           
                    pose_stamped = PoseStamped()   
                    pose_stamped.header = msg.header 
                    pose_stamped.pose = result.pose.pose
                    poses.append(pose_stamped)
                    scores.append(result.hypothesis.score)

        # sort poses based on scores
        poses = [x for _, x in sorted(zip(scores, poses), key=lambda pair: pair[0], reverse=True)]
        poses = poses[:n_max_poses]

        return poses
    
    def get_depth_image(self, depth_image, height, width, depth_scale):
        # this method converts the depth image to a depth array in meters
        print("Decode depth map from depth msg...")
        
        bridge = CvBridge()

        current_depth = np.zeros((height, width), dtype=np.float64)
        current_depth = bridge.imgmsg_to_cv2(depth_image, depth_image.encoding)

        depth_array = np.zeros( width * height, dtype=np.float64)

        for i in range(height):
            for j in range(width):
                depth_array[i * width + j] += current_depth[i, j] * np.float64(depth_scale)

        return depth_array
    
    def print_opt_result(self, pose, scale, success):
        
        if success == True:
            print("optimization success")
        else:
            print("optimization failure")
            
        print("refined position:")
        print("x = " + str(pose.pose.position.x))
        print("y = " + str(pose.pose.position.y))
        print("z = " + str(pose.pose.position.z))
        print("scale factor = " + str(scale))    
    
    
    
    

        
    def dope_depth_optimize(self, request, response):
        # request
        # int class_id
        # int n_max_poses
        # bool optimize
        # ---
        # response
        # geometry_msgs/PoseStamped[] refined_poses
        # float64[] scale_obj
        # bool[] success

        # compare the class id in the request with the class ids in the parameters
        # if the class id is not in the parameters, return an error message
        # if the class id is in the parameters, get the mesh path and mesh scale from the parameters
        # create a DepthOptimizer object with the camera parameters, object parameters and optimization parameters
        # call the depth_optimize method of the DepthOptimizer object with the estimated pose and depth values
        # return the refined poses, scales of the objects and success of the optimizations

        print("Dope Depth Optimize Service Called")

        # assure to read new messages
        self.dope_msg_received = False
        self.depth_msg_received = False    

        # request parameters
        class_id = request.class_id
        n_max_poses = request.n_max_poses
        optimize = request.optimize

        # object parameters
        object_name = ""
        mesh_path = ""
        mesh_scale = 0.0

        # response parameters
        refined_poses = []
        scale_obj = []
        opt_success = []

        # get the mesh path and mesh scale from the parameters
        try:
            object_name, mesh_path, mesh_scale = self.get_mesh_parameters(class_id)
        except:
            print("Exception raised when loading mesh parameters")
            response.success = []
            response.refined_poses = []
            response.scale_obj = []
            return response
        
        
        while rclpy.ok() and not (self.dope_msg_received and self.depth_msg_received):
             print("Waiting for messages...")
             print("Dope msg status: ", self.dope_msg_received)
             print("Depth msg status: ", self.depth_msg_received)
             time.sleep(1)

        # # reset variables for next service call
        # self.destroy_subscription(self.dope_sub)        
        # self.destroy_subscription(self.depth_sub)


        # get the poses of the detected objects with the class id, for a max of n_max_poses, ordered on the score value
        poses = self.get_detection(self.detection_msg, class_id, n_max_poses)
        print("Poses extracted")
        print(poses)

        # update depth_optimizer object with the object parameters
        mesh_path = resource_retriever.get_filename(
                        mesh_path, use_protocol=False)
        self.object_parameters['mesh_path'] = mesh_path
        self.object_parameters['mesh_scale'] = mesh_scale
        self.depth_optimizer.change_mesh(self.object_parameters)
        
        
        if optimize:
            # extract the depth values from the depth message
            real_depth_array = self.get_depth_image(self.depth_msg, self.camera_parameters['height_'], self.camera_parameters['width_'], self.camera_parameters['depth_scale'])
            print("Depth image extracted")
            print(real_depth_array)
            
            # optimize each pose
            for pose in poses:
                pose, scale, success = self.depth_optimizer.depth_optimize(pose,real_depth_array)
                self.print_opt_result(pose, scale, success)
                refined_poses.append(pose)
                scale_obj.append(scale)
                opt_success.append(success)

                            
            print("Optimizations completed")
                
            response.refined_poses = refined_poses
            response.scale_obj = scale_obj
            response.success = opt_success
        else:
            # return only the estimated poses from DOPE
            response.refined_poses = poses 

            
        return response




def main():
    rclpy.init()

    # If you have multiple gpus select one of them
    #os.environ["CUDA_VISIBLE_DEVICES"] = "0"

    dope_depht_optimizer_server = DopeDepthOptimizerServer()
    executor = MultiThreadedExecutor()
    executor.add_node(dope_depht_optimizer_server)
    


    try:
        executor.spin()
        #rclpy.spin(dope_depht_optimizer_server)
    except KeyboardInterrupt:
        pass

    dope_depht_optimizer_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()