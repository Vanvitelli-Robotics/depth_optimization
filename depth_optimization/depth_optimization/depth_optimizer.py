
from __future__ import print_function
# ros2 pkgs
from depth_optimization_interfaces.srv import DepthOptimize
from geometry_msgs.msg import PoseStamped

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


"""
This node refines the estimated pose from a 6D pose algorithm estimator through depth measurements solving an optimization problem
Input: estimated_pose, depth_values from aligned_rgb_to_depth_image
Output: refined_pose
"""

class DepthOptimizer(Node):

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
    pixel_cad_h = []
    pixel_cad_w = []
    real_useful_depth = []
    tol_optimization = 0.1
    remove_outliers = True

    # Plot parameters
    get_plot = False
    sigma_min_plt = -0.001
    sigma_max_plt = 0.001

    # Service
    initialized_scene = False
    interactive = True


    # methods
    def __init__(self):
        super().__init__('depth_optimizer', automatically_declare_parameters_from_overrides=True)
        
        # read parameters from yaml file
        self.read_parameters()
        self.add_on_set_parameters_callback(self.parameter_callback)

        # service creation
        self.srv = self.create_service(DepthOptimize, 'depth_optimize', self.depth_optimize_callback)

    def parameter_callback(self, parameters):
        for parameter in parameters:
            if parameter.name == 'mesh_path':
                self.mesh_path = parameter.value
                self.get_logger().info(f"mesh_path changed to: {self.mesh_path}")
                self.reset_scene(True)

            if parameter.name == 'mesh_scale':
                self.mesh_scale = parameter.value
                self.get_logger().info(f"mesh_scale changed to: {self.mesh_scale}")


        return SetParametersResult(successful=True)

    def read_parameters(self):
        self.get_logger().info("Reading parameters")
        
        self.width_ = self.get_parameter('width_').get_parameter_value().integer_value
        self.height_ = self.get_parameter('height_').get_parameter_value().integer_value
        self.fx = self.get_parameter('fx').get_parameter_value().double_value
        self.fy = self.get_parameter('fy').get_parameter_value().double_value
        self.cx = self.get_parameter('cx').get_parameter_value().double_value
        self.cy = self.get_parameter('cy').get_parameter_value().double_value
        self.focal_length = self.get_parameter('focal_length').get_parameter_value().double_value
        self.max_virtual_depth = self.get_parameter('max_virtual_depth').get_parameter_value().double_value
        self.tol_optimization = self.get_parameter('tol_optimization').get_parameter_value().double_value
        self.remove_outliers = self.get_parameter('remove_outliers').get_parameter_value().bool_value
        self.mesh_scale = self.get_parameter('mesh_scale').get_parameter_value().double_value
        self.mesh_path = self.get_parameter('mesh_path').get_parameter_value().string_value


        self.get_logger().info(f"Width: {self.width_}, Height: {self.height_}")
        self.get_logger().info(f"fx: {self.fx}, fy: {self.fy}")
        self.get_logger().info(f"cx: {self.cx}, cy: {self.cy}")
        self.get_logger().info(f"Focal Length: {self.focal_length}")
        self.get_logger().info(f"Max Virtual Depth: {self.max_virtual_depth}")
        self.get_logger().info(f"Tolerance for Optimization: {self.tol_optimization}")
        self.get_logger().info(f"Remove Outliers: {self.remove_outliers}")
        self.get_logger().info(f"Mesh Scale: {self.mesh_scale}")
        self.get_logger().info(f"Mesh Path: {self.mesh_path}")
    
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
        print("SIGMA #########################################")
        print(sigma)
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

    
    def plot_depth(self,sigma):
        # Create virtual depth map based on estimated pose and actual sigma
        self.virtual_depth_map(sigma)

        virtual_depth_array_used = []
        real_depth_array_used = []
        x_axis = []
        y_axis = []

        for k in range(len(self.pixel_cad_h)):
            if (self.real_useful_depth[k] != 0):
                i = self.pixel_cad_h[k]
                j = self.pixel_cad_w[k]

                d_hat = self.convert_from_uvd(
                    i, j, self.virtual_depth_array[i, j, 0], self.fx, self.fy, self.cx, self.cy)
                d = self.real_useful_depth[k]

                virtual_depth_array_used.append(d_hat)
                real_depth_array_used.append(d)
                x_axis.append(i)
                y_axis.append(j)

        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.scatter(x_axis, y_axis, real_depth_array_used, marker='o')
        ax.scatter(x_axis, y_axis, virtual_depth_array_used, marker='o')
        ax.set_xlabel('i')
        ax.set_ylabel('j')
        plt.show()


    def plot_cost_function(self,sigma_min, sigma_max):
        # sigma values in meters
        sigma_min = round(sigma_min) * 1000
        sigma_max = round(sigma_max) * 1000
        sigma_ = range(sigma_min, sigma_max)

        values = np.zeros(len(sigma_))
        sigma = np.zeros(len(sigma_))

        for i in range(0, len(sigma_)):
            sigma[i] = sigma[i]*0.001
            values[i] = self.cost_function(sigma[i])

        plt.plot(sigma, values,
                color='blue', label='Cost Function Plot')
        plt.xlabel('sigma')
        plt.ylabel('cost_function')
        plt.legend()
        plt.show()

    def depth_optimize_callback  (self, req, response):

        self.get_logger().info('Request received')
        tic = time.perf_counter()

        optimized_pose = PoseStamped()

        # Getting estimated object pose
        self.estimated_pose = req.estimated_pose

        # Initialize the virtual scene
        if not self.initialized_scene:
            tic1 = time.perf_counter()
            self.scene_initialization_nvisii()
            toc1 = time.perf_counter()
            print(f"NVISII initialization realized in {toc1 - tic1:0.4f} seconds")
            self.initialized_scene = True

        # Getting useful pixels
        tic1 = time.perf_counter()
        self.get_useful_pixels(req.depth_matrix)
        toc1 = time.perf_counter()
        print(f"get useful pixels realized in {toc1 - tic1:0.4f} seconds")

        
        # If desired, remove outliers with RANSAC regression
        if self.remove_outliers:
            tic1 = time.perf_counter()
            self.delete_outliers()
            toc1 = time.perf_counter()
            print(f"remove outliers realized in {toc1 - tic1:0.4f} seconds")

        # Minimize objective function
        bound_scale = 0.8
        sigma_min = -self.estimated_pose.pose.position.z * bound_scale
        sigma_max = +self.estimated_pose.pose.position.z * bound_scale
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

        if (success == False):
            res.x = 0
        
      
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

        if self.get_plot:
            self.plot_cost_function(sigma_min=self.sigma_min_plt, sigma_max=self.sigma_max_plt)
            self.plot_depth(res.x)

        if success:
            print("OPTIMIZATION COMPLETED:\n")    
            print("Function_min: ", res.fun)
            print("Sigma_min [m]: ", res.x)
            print("Optimized Pose: ", optimized_pose)

        # Clear variables for next service call
        self.reset_scene()

        response.refined_pose = optimized_pose
        response.scale_obj = scale_obj
        response.success = success

        return response
    
    def reset_scene(self, update_mesh=False):
        if self.initialized_scene and update_mesh:
            nvisii.clear_all()
            self.populate_scene()
        self.pixel_cad_h = []
        self.pixel_cad_w = []
        self.real_useful_depth = []
        self.virtual_depth_array = []

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

def main():
    rclpy.init()

    os.environ["CUDA_VISIBLE_DEVICES"] = "0"

    depth_optimize = DepthOptimizer()

    try:
        rclpy.spin(depth_optimize)
    except KeyboardInterrupt:
        pass

    depth_optimize.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()