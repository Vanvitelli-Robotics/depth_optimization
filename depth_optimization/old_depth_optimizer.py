#!/usr/bin/env python3

"""
This node refines the estimated pose from dope through depth measurements by an optimization algorithm
Input: estimated_pose, depth_values from aligned_rgb_to_depth_image
Output: refined_pose
"""

from __future__ import print_function
import nvisii
import numpy as np
from grasp_dope.srv import depth_optimizer, depth_optimizerResponse
import rospy
from geometry_msgs.msg import PoseStamped
import rospy
from scipy.optimize import minimize_scalar
import matplotlib.pyplot as plt
import matplotlib.markers
from mpl_toolkits import mplot3d
from sklearn import linear_model
from numpy import linalg as LA
import time
import os

os.environ["CUDA_VISIBLE_DEVICES"] = "0"

index_render = 0

### USER PARAMETERS ###
# camera width, height and intrinsics
width_ = 640
height_ = 480
fx = 610.59326171875
fy = 610.605712890625
cx = 317.7075500488281
cy = 238.1421356201172
focal_length = 0  # 1.93 mm

# recognized object parameters

# obj_to_load = "/home/workstation/dope_ros_ws/src/grasp_dope/scripts/models/Apple/Apple_4K/food_apple_01_4k.obj" # absolute path to cad model
# obj_name = "apple"  # obj name
# cad_dimension = [0.09756118059158325, 0.08538994193077087,0.09590171277523041]  # x, y, z, obj cuboid dimensions [m]
# mesh_scale = 1
object_name = rospy.get_param("/dope/object_of_interest")
obj_to_load = rospy.get_param("/dope/meshes")[object_name]   
cad_dimension = rospy.get_param("/dope/dimensions")[object_name] # cm
mesh_scale = rospy.get_param("/dope/mesh_scales")[object_name] 

#CAD dimension in meters
cad_dimension[0] = cad_dimension[0] * 0.01
cad_dimension[1] = cad_dimension[1] * 0.01
cad_dimension[2] = cad_dimension[2] * 0.01



interactive = True
# [m] max depth value captured by virtual camera in meters
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
sigma_min = -0.001
sigma_max = 0.001

# Service
initialized_scene = False


def scene_initialization_nvisii():
    global obj_to_load, object_name
    nvisii.initialize(headless=not interactive, verbose=True)
    nvisii.disable_updates()
    # nvisii.disable_denoiser()

    camera = nvisii.entity.create(
        name="camera",
        transform=nvisii.transform.create("camera"),
        camera=nvisii.camera.create_from_intrinsics(
            name="camera",
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            width=width_,
            height=height_
        )
    )
    camera.get_transform().look_at(
        at=(0, 0, 0),
        up=(0, -1, -1),
        eye=(1, 1, 0)
    )
    nvisii.set_camera_entity(camera)

    obj_mesh = nvisii.entity.create(
        name=object_name,
        mesh=nvisii.mesh.create_from_file(object_name, obj_to_load),
        transform=nvisii.transform.create(object_name),
        material=nvisii.material.create(object_name)
    )

    obj_mesh.get_transform().set_parent(camera.get_transform())

    nvisii.sample_pixel_area(
        x_sample_interval=(.5, .5),
        y_sample_interval=(.5, .5)
    )


def delete_outliers():
    global real_useful_depth
    line_ransac = linear_model.RANSACRegressor()
    line_ransac.fit(np.linspace(0, 1, len(real_useful_depth)
                                ).reshape(-1, 1), real_useful_depth)
    inlier_mask = line_ransac.inlier_mask_
    outlier_mask = np.logical_not(inlier_mask)
    for i in range(len(outlier_mask)):
        if (outlier_mask[i] == True):
            real_useful_depth[i] = 0.0


def get_useful_pixels(real_depth_array):
    global real_useful_depth
    # This function selects the pixels corresponding to cad object in the virtual scene and corresponding real depth values
    virtual_depth_map(0)
    for i in range(height_):
        for j in range(width_):
            if virtual_depth_array[i, j, 0] < max_virtual_depth and virtual_depth_array[i, j, 0] > 0:
                pixel_cad_h.append(i)
                pixel_cad_w.append(j)

    for i in range(len(pixel_cad_h)):
        real_useful_depth.append(
            real_depth_array[pixel_cad_h[i]*width_ + pixel_cad_w[i]])


def convert_from_uvd(v, u, d, fx, fy, cx, cy):
    x_over_z = (cx - u) / fx
    y_over_z = (cy - v) / fy
    z = d / np.sqrt(1. + x_over_z**2 + y_over_z**2)
    return z


def virtual_depth_map(sigma):
    global virtual_depth_array, index_render
    index_render = index_render +1
    obj_mesh = nvisii.entity.get(object_name)

    x = estimated_pose.pose.position.x
    y = -estimated_pose.pose.position.y
    z = -estimated_pose.pose.position.z

    p = np.array([x, y, z])
    f = np.array([0, 0, focal_length])

    p_new = p + np.multiply(np.divide((f-p), LA.norm(p-f)), sigma)
    scale_obj = LA.norm(p_new-f)/LA.norm(p-f)

    obj_mesh.get_transform().set_position(p_new)

    rotation_flip = nvisii.angleAxis(-nvisii.pi(),nvisii.vec3(1,0,0)) * nvisii.quat(estimated_pose.pose.orientation.w,
                                                      estimated_pose.pose.orientation.x, estimated_pose.pose.orientation.y, estimated_pose.pose.orientation.z)
    obj_mesh.get_transform().set_rotation(rotation_flip)

    obj_mesh.get_transform().set_scale(nvisii.vec3(scale_obj*mesh_scale))

    virtual_depth_array = nvisii.render_data(
        width=int(width_),
        height=int(height_),
        start_frame=0,
        frame_count=1,
        bounce=int(0),
        options="depth"
    )

    virtual_depth_array = np.array(
        virtual_depth_array).reshape(height_, width_, 4)
    virtual_depth_array = np.flipud(virtual_depth_array)


def cost_function(sigma):
    global virtual_depth_array
    tic1 = time.perf_counter()

    # Create virtual depth map based on estimated pose and actual sigma
    virtual_depth_map(sigma)
    toc1 = time.perf_counter()
    print(f"virtual depth map realized in {toc1 - tic1:0.4f} seconds")

    sum = 0
    num_pixel_obj = 0
    tic1 = time.perf_counter()
    for k in range(len(pixel_cad_h)):
        if (real_useful_depth[k] != 0):  # if depth value isn't an outlier
            i = pixel_cad_h[k]
            j = pixel_cad_w[k]

            d_hat = convert_from_uvd(
                i, j, virtual_depth_array[i, j, 0], fx, fy, cx, cy)
            d = real_useful_depth[k]

            sum = sum + pow((d-d_hat), 2)
            num_pixel_obj = num_pixel_obj+1
    toc1 = time.perf_counter()
    print(f"evaluation cost function realized in {toc1 - tic1:0.4f} seconds")
    try:
        return sum/num_pixel_obj
    except ZeroDivisionError:
        print("Error: Division by zero")


def plot_depth(sigma):
    # Create virtual depth map based on estimated pose and actual sigma
    virtual_depth_map(sigma)

    virtual_depth_array_used = []
    real_depth_array_used = []
    x_axis = []
    y_axis = []

    for k in range(len(pixel_cad_h)):
        if (real_useful_depth[k] != 0):
            i = pixel_cad_h[k]
            j = pixel_cad_w[k]

            d_hat = convert_from_uvd(
                i, j, virtual_depth_array[i, j, 0], fx, fy, cx, cy)
            d = real_useful_depth[k]

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


def plot_cost_function(sigma_min, sigma_max):
    # sigma values in meters
    sigma_min = round(sigma_min) * 1000
    sigma_max = round(sigma_max) * 1000
    sigma_ = range(sigma_min, sigma_max)

    values = np.zeros(len(sigma_))
    sigma = np.zeros(len(sigma_))

    for i in range(0, len(sigma_)):
        sigma[i] = sigma[i]*0.001
        values[i] = cost_function(sigma[i])

    plt.plot(sigma, values,
             color='blue', label='Cost Function Plot')
    plt.xlabel('sigma')
    plt.ylabel('cost_function')
    plt.legend()
    plt.show()


def handle_depth_optimizer(req):
    global estimated_pose, initialized_scene, pixel_cad_h, pixel_cad_w, real_useful_depth, virtual_depth_array
    tic = time.perf_counter()

    optimized_pose = PoseStamped()

    # Getting estimated object pose
    estimated_pose = req.estimated_pose

    # Initialize the virtual scene
    if not initialized_scene:
        tic1 = time.perf_counter()
        scene_initialization_nvisii()
        toc1 = time.perf_counter()
        print(f"NVISII initialization realized in {toc1 - tic1:0.4f} seconds")
        initialized_scene = True

    # Getting useful pixels
    tic1 = time.perf_counter()
    get_useful_pixels(req.depth_matrix)
    toc1 = time.perf_counter()
    print(f"get useful pixels realized in {toc1 - tic1:0.4f} seconds")

    # If desired, remove outliers with RANSAC regression
    if remove_outliers:
        tic1 = time.perf_counter()
        delete_outliers()
        toc1 = time.perf_counter()
        print(f"remove outliers realized in {toc1 - tic1:0.4f} seconds")

    # Minimize objective function
    bound_scale = 0.8
    sigma_min = -estimated_pose.pose.position.z * bound_scale
    sigma_max = +estimated_pose.pose.position.z * bound_scale
    tic1 = time.perf_counter()
    res = minimize_scalar(cost_function, bounds=[
                          sigma_min, sigma_max], tol=tol_optimization)
    toc1 = time.perf_counter()
    print(f"minimization realized in {toc1 - tic1:0.4f} seconds")

    toc = time.perf_counter()
    print(f"Optimization realized in {toc - tic:0.4f} seconds")

    if res.fun > tol_optimization:
        success = False
    else:
        success = True

    if (success == False):
        res.x = 0
    x = estimated_pose.pose.position.x
    y = estimated_pose.pose.position.y
    z = estimated_pose.pose.position.z
    p = np.array([x, y, z])
    f = np.array([0, 0, focal_length])
    p_new = p + np.multiply(np.divide((f-p), LA.norm(p-f)), res.x)

    optimized_pose.pose.position.x = p_new[0]
    optimized_pose.pose.position.y = p_new[1]
    optimized_pose.pose.position.z = p_new[2]
    optimized_pose.header.frame_id = "camera_color_optical_frame"

    optimized_pose.pose.orientation = estimated_pose.pose.orientation

    scale_obj = LA.norm(p_new-f)/LA.norm(p-f)
    scaled_dimension = []
    for i in range(len(cad_dimension)):
        scaled_dimension.append(cad_dimension[i]*scale_obj)

    print("Function_min: ", res.fun)
    print("Sigma_min [m]: ", res.x)
    print("Scaled cad dimensions [m]: ", scaled_dimension)
    print("Optimized Pose: ", optimized_pose)

    if get_plot:
        plot_cost_function(sigma_min=sigma_min, sigma_max=sigma_max)
        plot_depth(res.x)

    # let's clean up the GPU
    # nvisii.deinitialize()

    # Clear variables for next service call
    pixel_cad_h = []
    pixel_cad_w = []
    real_useful_depth = []
    virtual_depth_array = []

    return depth_optimizerResponse(optimized_pose, scaled_dimension, scale_obj, success)


def depth_optimizer_service():
    rospy.init_node('depth_optimizer_server')
    s = rospy.Service('depth_optimizer', depth_optimizer,
                      handle_depth_optimizer)

    rospy.spin()


if __name__ == "__main__":
    depth_optimizer_service()