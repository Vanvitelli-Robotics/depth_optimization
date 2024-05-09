
from __future__ import print_function
# ros2 pkgs
from depth_optimization_interfaces.srv import DepthOptimize
from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

# python modules
import numpy as np
from scipy.optimize import minimize_scalar
import matplotlib.pyplot as plt
from sklearn import linear_model
from numpy import linalg as LA
import time
import os
from plyfile import PlyData
from scipy.spatial.transform import Rotation
import cv2
import math


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
    intrinsic_matrix = 0

    # Object parameters
    object_name = "object_entity"
    mesh_path = []  
    mesh_scale = []
    mesh_vertices = []

    # [m] max depth value captured by the virtual camera in meters
    max_virtual_depth = 5

    estimated_pose = PoseStamped()

    # Optimization parameters
    pixel_cad_h = []
    pixel_cad_w = []
    real_useful_depth = []
    tol_optimization = 0.1
    remove_outliers = True

 
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
        



    def generate_depth_map(self,vertices, intrinsic_matrix, extrinsic_matrix, image_dimensions):
        # Convert vertices to homogeneous coordinates (add 1)
        homogeneous_vertices = np.hstack((vertices, np.ones((vertices.shape[0], 1))))

        # Apply extrinsic transformation
        transformed_vertices = np.dot(homogeneous_vertices, extrinsic_matrix.T)

        # Project vertices into the camera
        projected_vertices = np.dot(transformed_vertices, intrinsic_matrix.T)

        # Normalize homogeneous coordinates
        projected_vertices[:, 0] /= projected_vertices[:, 2]
        projected_vertices[:, 1] /= projected_vertices[:, 2]

        # Create the depth map
        depth_map = np.ones(image_dimensions)* np.nan
        object_pixels = []

        # Draw projected points on the depth map
        for point in projected_vertices:
            x, y, z = point.astype(float)
            if 0 <= x < image_dimensions[1] and 0 <= y < image_dimensions[0]:
                xr = math.floor(x)
                yr = math.floor(y)
                if math.isnan(depth_map[yr,xr]):          
                    depth_map[yr, xr] = z  # Depth Z
                    object_pixels.append((xr, yr))  # Add the pixel to the list of object pixels
                else:        
                    # the pixel is already occupied => take the nearest one
                    depth_map[yr,xr] = min(z,depth_map[yr,xr])


        return depth_map, object_pixels
                
        
    def remove_occluded_points(self,depth_map,object_pixels):
        # Define neighborhood size
        neighborhood_size = 10


        # Create an empty array to store the result
        result_map = np.zeros((self.height_, self.width_))
        

        # Iterate over each pixel in the depth map
        for x, y in object_pixels:
                # Get the depth value of the current pixel
                depth_value = depth_map[y, x]
                

                # Check if the current pixel is at the border where the neighborhood is not fully available
                if y - neighborhood_size >= 0 and y + neighborhood_size < self.height_ and \
                        x - neighborhood_size >= 0 and x + neighborhood_size < self.width_:
                    # Extract the neighborhood around the current pixel
                    neighborhood = depth_map[y - neighborhood_size:y + neighborhood_size + 1,
                                            x - neighborhood_size:x + neighborhood_size + 1]
                                        

                    # Compute the mean depth value of the neighborhood
                    mean_depth = np.nanmean(neighborhood)

                    # Check if the depth value of the current pixel is less than the mean depth of the neighborhood
                    if depth_value <= mean_depth:
                        result_map[y, x] = depth_value

        return result_map

    def read_ply_file(self,file_name):
        try:
            # Read the PLY file
            with open(file_name, 'rb') as f:
                ply_data = PlyData.read(f)

            # Extract necessary information from the PLY file
            vertices = np.vstack([ply_data['vertex']['x'],
                                ply_data['vertex']['y'],
                                ply_data['vertex']['z']]).T

            
            return vertices/self.mesh_scale

        except FileNotFoundError:
            print("The specified file was not found.")
            return None, None, None, None
        except Exception as e:
            print("An error occurred while reading the file:", e)
            return None, None, None, None
            
            
    

    def get_useful_pixels(self,real_depth_array):
        # This function selects the pixels corresponding to cad object in the virtual scene and corresponding real depth values
        self.virtual_depth_map(0)
  
        if len(real_depth_array)==0:
            print("empty real depth array")
        
        for i in range(len(self.pixel_cad_h)):
            self.real_useful_depth.append(
                real_depth_array[self.pixel_cad_h[i]*self.width_ + self.pixel_cad_w[i]])
        
        
    def virtual_depth_map(self,sigma):
        try:
            
            print(sigma)
        
            x = self.estimated_pose.pose.position.x
            y = self.estimated_pose.pose.position.y
            z = self.estimated_pose.pose.position.z

            p = np.array([x, y, z])
            f = np.array([0, 0, self.focal_length])

            p_new = p + np.multiply(np.divide((f-p), LA.norm(p-f)), sigma)
            scale_obj = LA.norm(p_new-f)/LA.norm(p-f)
            
            R = Rotation.from_quat([self.estimated_pose.pose.orientation.x,self.estimated_pose.pose.orientation.y,self.estimated_pose.pose.orientation.z,self.estimated_pose.pose.orientation.w])
            R = R.as_matrix()
            p_new = p_new.reshape(3,1)
            extrinsic_matrix = np.hstack((R, p_new))        
            
            scaled_pcl = self.mesh_vertices * scale_obj
            
            
            depth_map, object_pixels = self.generate_depth_map(scaled_pcl, self.intrinsic_matrix, extrinsic_matrix, (self.height_, self.width_))
            self.virtual_depth_array = self.remove_occluded_points(depth_map,object_pixels)
            #self.virtual_depth_array = depth_map
            self.pixel_cad_w, self.pixel_cad_h = zip(*object_pixels)
           
            cv2.imshow("depth map",self.virtual_depth_array)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            
        except Exception as e:
            print("An error occurred while generating virtual depth map:", e)
            print("check if the object is outside the field of view of the camera")
           
        
        
        #self.virtual_depth_array = np.flipud(self.virtual_depth_array)

    
    def delete_outliers(self):
        line_ransac = linear_model.RANSACRegressor()
        line_ransac.fit(np.linspace(0, 1, len(self.real_useful_depth)
                                    ).reshape(-1, 1), self.real_useful_depth)
        inlier_mask = line_ransac.inlier_mask_
        outlier_mask = np.logical_not(inlier_mask)
        for i in range(len(outlier_mask)):
            if (outlier_mask[i] == True):
                self.real_useful_depth[i] = 0.0
    

    def cost_function(self,sigma):
        
        try:
            tic1 = time.perf_counter()

            # Create virtual depth map based on estimated pose and actual sigma
            self.virtual_depth_map(sigma)
            toc1 = time.perf_counter()
            print(f"virtual depth map realized in {toc1 - tic1:0.4f} seconds")

            sum = 0
            num_pixel_obj = 0
            tic1 = time.perf_counter()
            
            for k in range(len(self.pixel_cad_h)-1):
                if (self.real_useful_depth[k] != 0):  # if depth value isn't an outlier
                    i = self.pixel_cad_h[k]
                    j = self.pixel_cad_w[k]
            
                    d_hat = self.virtual_depth_array[i, j]
                    d = self.real_useful_depth[k]

                    sum = sum + pow((d-d_hat), 2)
                    num_pixel_obj = num_pixel_obj+1
                    
            toc1 = time.perf_counter()
            print(f"evaluation cost function realized in {toc1 - tic1:0.4f} seconds")
            return sum/num_pixel_obj
        
        except ZeroDivisionError:
            print("Error: Division by zero")
        except Exception as e:
            print("An error occurred while evaluating cost function", e)

    
    def depth_optimize_callback(self, req, response):

        self.get_logger().info('Request received')
        tic = time.perf_counter()

        optimized_pose = PoseStamped()

        # Getting estimated object pose
        self.estimated_pose = req.estimated_pose

        # initialize scene with object
        self.intrinsic_matrix = np.array([[self.fx, 0, self.cx],
                                          [0, self.fy, self.cy],
                                          [0, 0, 1]])
        
        self.mesh_vertices = self.read_ply_file(self.mesh_path)

        # Getting useful pixels
        tic1 = time.perf_counter()
        # generate fake real depth
        self.virtual_depth_map(-5)
        fake_depth = self.virtual_depth_array
        fake_depth = fake_depth.reshape(self.height_*self.width_)
        print(fake_depth)
        print("shape fake",fake_depth.shape)
        for i in range(640*480):
            if fake_depth[i]!=0:
                print(fake_depth[i])

        self.get_useful_pixels(fake_depth)
        #self.get_useful_pixels(req.depth_matrix)
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
            print("OPTIMIZATION FAILURE:\n")
            print("function value greater then the tol. optimization", res.fun, self.tol_optimization)  
            #res.x  = 0 # return the estimated position without changes
        else:
            success = True
            print("OPTIMIZATION COMPLETED:\n")  
            
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

          
        print("Function_min: ", res.fun)
        print("Sigma_min [m]: ", res.x)
        print("Scale obj: ", scale_obj)
        print("Optimized Pose: ", optimized_pose)


        # Clear variables for next service call
        self.reset_scene()

        response.refined_pose = optimized_pose
        response.scale_obj = scale_obj
        response.success = success

        return response
    
    def reset_scene(self, update_mesh=False):
        self.pixel_cad_h = []
        self.pixel_cad_w = []
        self.real_useful_depth = []
        self.virtual_depth_array = []


def main():
    rclpy.init()

    depth_optimize = DepthOptimizer()

    try:
        rclpy.spin(depth_optimize)
    except KeyboardInterrupt:
        pass

    depth_optimize.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
