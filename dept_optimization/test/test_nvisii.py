import nvisii
import pdb

interactive = True
width_ = 640
height_ = 480
fx = 610.59326171875
fy = 610.605712890625
cx = 317.7075500488281
cy = 238.1421356201172
focal_length = 0  
obj_to_load = "/home/sfederico/Documents/cad_models/Apple/Apple_4K/food_apple_01_4k.obj" # absolute path to cad model
object_name = "apple"  # obj name
cad_dimension = [0.09756118059158325, 0.08538994193077087,0.09590171277523041]  # x, y, z, obj cuboid dimensions [m]
mesh_scale = 1

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

breakpoint()
obj_mesh.get_transform().set_parent(camera.get_transform())

nvisii.sample_pixel_area(
    x_sample_interval=(.5, .5),
    y_sample_interval=(.5, .5)
)
