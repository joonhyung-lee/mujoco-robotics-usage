import multiprocessing as mp
from multiprocessing import Process, Queue
import os

import cv2
import mujoco
import mujoco.viewer as viewer
import numpy as np

import sys
sys.path.append('../../')
from utils.util import meters2xyz, rpy2r, pr2t

def get_pcd_from_depth_img(cam, depth_img,fovy=45):
    """
        Get point cloud data from depth image
    """
    # Get camera pose
    cam_lookat    = cam.lookat
    cam_elevation = cam.elevation
    cam_azimuth   = cam.azimuth
    cam_distance  = cam.distance

    p_lookat = cam_lookat
    R_lookat = rpy2r(np.deg2rad([0,-cam_elevation,cam_azimuth]))
    T_lookat = pr2t(p_lookat,R_lookat)
    T_viewer = T_lookat @ pr2t(np.array([-cam_distance,0,0]),np.eye(3))

    # Camera intrinsic
    img_height = depth_img.shape[0]
    img_width = depth_img.shape[1]
    focal_scaling = 0.5*img_height/np.tan(fovy*np.pi/360)
    cam_matrix = np.array(((focal_scaling,0,img_width/2),
                        (0,focal_scaling,img_height/2),
                        (0,0,1)))

    # Estimate 3D point from depth image
    xyz_img = meters2xyz(depth_img,cam_matrix) # [H x W x 3]
    xyz_transpose = np.transpose(xyz_img,(2,0,1)).reshape(3,-1) # [3 x N]
    xyzone_transpose = np.vstack((xyz_transpose,np.ones((1,xyz_transpose.shape[1])))) # [4 x N]

    # To world coordinate
    xyzone_world_transpose = T_viewer @ xyzone_transpose
    xyz_world_transpose = xyzone_world_transpose[:3,:] # [3 x N]
    xyz_world = np.transpose(xyz_world_transpose,(1,0)) # [N x 3]
    return xyz_world,xyz_img

# The "vision process" renders images in a totally seperate process from the MuJoCo viewer/physics thread
# It is needed on MacOS because currently MuJoCo does not support OpenGL rendering off of the main
# thread on MacOS.
#
# On MacOS all live visualization (e.g. cv2.imshow() calls) need to occur in this process, because the
# physics thread (and the on_control callback) cannot have a UI
def create_vision_process(m, cam_name, cam_res):
    mp.set_start_method('spawn')
    req_q = Queue()
    ret_q = Queue()
    p = Process(target=vision_process,
                args=(req_q, ret_q, m, cam_name, cam_res),
                daemon=True)
    p.start()
    return req_q, ret_q, p

# req_q is a multiprocessing Queue which will contain MuJoCo "data" objects for rendering
# ret_q is a multiprocessing Queue which will return the image
def vision_process(req_q, ret_q, m, cam_name, cam_res):
    # Make all the things needed to render a simulated camera
    gl_ctx = mujoco.GLContext(*cam_res)
    gl_ctx.make_current()

    scn = mujoco.MjvScene(m, maxgeom=100)

    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
    cam.fixedcamid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, cam_name)

    vopt = mujoco.MjvOption()
    vopt.geomgroup[1] = 0 # Group 1 is the mocap markers for visualization
    pert = mujoco.MjvPerturb()

    ctx = mujoco.MjrContext(m, mujoco.mjtFontScale.mjFONTSCALE_150)
    mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, ctx)

    viewport = mujoco.MjrRect(0, 0, *cam_res)

    image = np.empty((cam_res[1], cam_res[0], 3), dtype=np.uint8)
    depth_img = np.empty((cam_res[1], cam_res[0], 1), dtype=np.float32)
    while True:
        d = req_q.get()

        mujoco.mjv_updateScene(m, d, vopt, pert, cam, mujoco.mjtCatBit.mjCAT_ALL, scn)
        mujoco.mjr_render(viewport, scn, ctx)
        mujoco.mjr_readPixels(image, depth_img, viewport, ctx)
        image = cv2.flip(image, 0) # OpenGL renders with inverted y axis
        depth_img = cv2.flip(depth_img, 0) # OpenGL renders with inverted y axis
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        cv2.imshow('image', image)
        cv2.waitKey(1)

        cv2.imshow('depth image', depth_img)
        cv2.waitKey(1)

        # The computation slows down the simulation. Resize the image would be better. 
        # xyz_world,xyz_img = get_pcd_from_depth_img(cam, depth_img,fovy=45)
        # cv2.imshow('xyz image', xyz_img)
        # cv2.waitKey(1)
        ret_q.put(None)

# on_control callback for physics simulation
last_render_t = 0.0
def control(m, d, req_q, res_q):
  try:

    # Render the camera on the mouse using MacOS workaround
    global last_render_t
    if d.time - last_render_t > 1/60.0:
      last_render_t = d.time

      req_q.put(d)
      result = res_q.get() # For now result is None, it should be the result of processing the image

  except Exception as e:
    print(e)

def load_callback(m=None, d=None, xml_path=None, req_q=None, res_q=None):
  mujoco.set_mjcb_control(None)

  m = mujoco.MjModel.from_xml_path(xml_path)
  d = mujoco.MjData(m)

  if m is not None:
    mujoco.set_mjcb_control(lambda m, d: control(m, d, req_q, res_q))

  return m, d

if __name__ == '__main__':
  xml_path = '../../asset/ur5e/scene_ur5e_rg2_d435i_obj.xml'
  # xml_path = os.path.abspath('mouse.xml')
  m = mujoco.MjModel.from_xml_path(xml_path)
  req_q, res_q, p = create_vision_process(m, 'egocentric', (480, 320))

  viewer.launch(loader=lambda m=None, d=None: load_callback(m, d, xml_path, req_q, res_q))