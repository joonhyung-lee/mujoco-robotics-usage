#!/usr/bin/env python3
"""Render 3D MuJoCo navigation GIF using offscreen renderer."""
import numpy as np
from PIL import Image
import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__), '../../'))
import mujoco
from utils.mujoco_parser import MuJoCoParserClass
from utils.util import r2rpy
from utils.rrt import RapidlyExploringRandomTreesStarClass

CELL_SIZE = 2.6; WALL_HEIGHT = 1.0
START_CELL = (1, 1); GOAL_CELL = (8, 8)
MAZE_MAP = np.array([
    [1,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,0,1,0,0,0,1],
    [1,0,1,1,0,1,0,1,0,1],
    [1,0,1,0,0,0,0,1,0,1],
    [1,0,0,0,1,1,0,0,0,1],
    [1,1,1,0,0,0,0,1,1,1],
    [1,0,0,0,1,0,0,0,0,1],
    [1,0,1,0,1,0,1,1,0,1],
    [1,0,0,0,0,0,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1],
])
n_rows, n_cols = MAZE_MAP.shape
def cell_to_world(r, c):
    return c*CELL_SIZE+CELL_SIZE/2, -(r*CELL_SIZE+CELL_SIZE/2)
start_x, start_y = cell_to_world(*START_CELL)
goal_x, goal_y = cell_to_world(*GOAL_CELL)

# Generate XML
def gen_xml():
    walls = []
    for r in range(n_rows):
        for c in range(n_cols):
            if MAZE_MAP[r,c]==1:
                x,y=cell_to_world(r,c)
                walls.append(f'    <geom name="wall_{len(walls)}" type="box" '
                    f'pos="{x:.3f} {y:.3f} {WALL_HEIGHT/2:.3f}" '
                    f'size="{CELL_SIZE/2:.3f} {CELL_SIZE/2:.3f} {WALL_HEIGHT/2:.3f}" '
                    f'rgba="0.6 0.65 0.7 1" contype="1" conaffinity="1"/>')
    # Add goal marker (red sphere)
    walls.append(f'    <geom name="goal_marker" type="sphere" pos="{goal_x:.3f} {goal_y:.3f} 0.5" '
                 f'size="0.3" rgba="1 0.1 0.1 0.7" contype="0" conaffinity="0"/>')
    xml = ('<mujoco model="husky maze">\n  <include file="./husky_w_lidar.xml"/>\n'
           '  <include file="../object/floor_sky.xml"/>\n  <worldbody>\n'
           + '\n'.join(walls) + '\n  </worldbody>\n</mujoco>\n')
    with open('../../asset/husky/scene_husky_maze_slam.xml','w') as f: f.write(xml)
gen_xml()

# Load
env = MuJoCoParserClass(name='Husky',
    rel_xml_path='../../asset/husky/scene_husky_maze_slam.xml', VERBOSE=False)
wheel_base = 0.2854*2

# ── OccupancyGrid (for RRT*) ──
class OccupancyGrid:
    def __init__(self, x_range, y_range, resolution=0.15):
        self.resolution=resolution
        self.x_min,self.x_max=x_range; self.y_min,self.y_max=y_range
        self.width=int(np.ceil((self.x_max-self.x_min)/resolution))
        self.height=int(np.ceil((self.y_max-self.y_min)/resolution))
        self.log_odds=np.zeros((self.height,self.width),dtype=np.float32)
        self.l_max=5.0; self.l_min=-5.0
    def world_to_grid(self,x,y):
        return (np.clip(int((x-self.x_min)/self.resolution),0,self.width-1),
                np.clip(int((y-self.y_min)/self.resolution),0,self.height-1))
    def bresenham(self,x0,y0,x1,y1):
        cells=[]; dx,dy=abs(x1-x0),abs(y1-y0)
        sx=1 if x0<x1 else -1; sy=1 if y0<y1 else -1; err=dx-dy
        while True:
            cells.append((x0,y0))
            if x0==x1 and y0==y1: break
            e2=2*err
            if e2>-dy: err-=dy; x0+=sx
            if e2<dx: err+=dx; y0+=sy
        return cells
    def is_free(self,x,y,margin=0.3):
        gx,gy=self.world_to_grid(x,y); r=int(margin/self.resolution)
        for dx in range(-r,r+1):
            for dy in range(-r,r+1):
                cx,cy=gx+dx,gy+dy
                if not(0<=cx<self.width and 0<=cy<self.height): return False
                if self.log_odds[cy,cx]>=0.0: return False
        return True
    def is_line_free(self,x1,y1,x2,y2,margin=0.3):
        r=int(margin/self.resolution)
        for cx,cy in self.bresenham(*self.world_to_grid(x1,y1),*self.world_to_grid(x2,y2)):
            for ddx in range(-r,r+1):
                for ddy in range(-r,r+1):
                    nx_,ny_=cx+ddx,cy+ddy
                    if not(0<=nx_<self.width and 0<=ny_<self.height): return False
                    if self.log_odds[ny_,nx_]>=0.0: return False
        return True

# Build known map for RRT*
margin = 2.0
og = OccupancyGrid(x_range=(-margin, n_cols*CELL_SIZE+margin),
                    y_range=(-(n_rows*CELL_SIZE+margin), margin))
for r in range(n_rows):
    for c in range(n_cols):
        wx,wy=cell_to_world(r,c); gx,gy=og.world_to_grid(wx,wy)
        half=int(CELL_SIZE/2/og.resolution)+1  # +1 to avoid gaps between cells
        for dx in range(-half,half+1):
            for dy in range(-half,half+1):
                cx,cy=gx+dx,gy+dy
                if 0<=cx<og.width and 0<=cy<og.height:
                    og.log_odds[cy,cx]=og.l_max if MAZE_MAP[r,c]==1 else og.l_min

# ── RRT* ──
print("=== RRT* Planning ===")
goal_xy = np.array([goal_x, goal_y])
rrt = RapidlyExploringRandomTreesStarClass(
    name='RRT', point_min=np.array([og.x_min+0.5,og.y_min+0.5]),
    point_max=np.array([og.x_max-0.5,og.y_max-0.5]),
    goal_select_rate=0.15, steer_len_max=1.0, search_radius=2.0,
    n_node_max=5000, SPEED_UP=True)
rrt.init_rrt_star(point_root=np.array([start_x,start_y]), point_goal=goal_xy, seed=42)
for it in range(5000):
    ps=goal_xy.copy() if np.random.rand()<0.15 else rrt.sample_point()
    if not og.is_free(ps[0],ps[1]): continue
    nn=rrt.get_node_nearest(ps); pt,cs=rrt.steer(nn,ps)
    if pt is None or not og.is_free(pt[0],pt[1]): continue
    pn=rrt.get_node_point(nn)
    if not og.is_line_free(pn[0],pn[1],pt[0],pt[1]): continue
    near=rrt.get_nodes_near(pt); nb_,cb_=nn,cs
    for nr in near:
        p_nr=rrt.get_node_point(nr); c_nr=rrt.get_node_cost(nr)+rrt.get_dist(p_nr,pt)
        if c_nr<cb_ and og.is_line_free(p_nr[0],p_nr[1],pt[0],pt[1]): nb_,cb_=nr,c_nr
    n_new=rrt.add_node(point=pt,cost=cb_,node_parent=nb_)
    for nr in near:
        if nr==nb_: continue
        p_nr=rrt.get_node_point(nr); c_new=cb_+rrt.get_dist(pt,p_nr)
        if c_new<rrt.get_node_cost(nr) and og.is_line_free(pt[0],pt[1],p_nr[0],p_nr[1]):
            rrt.replace_node_parent(nr,n_new); rrt.update_node_info(nr,cost=c_new)
    if rrt.get_dist(pt,goal_xy)<1.0 and og.is_line_free(pt[0],pt[1],goal_xy[0],goal_xy[1]):
        cg=cb_+rrt.get_dist(pt,goal_xy); eg=rrt.get_node_goal(eps=0.05)
        if eg is None or cg<rrt.get_node_cost(eg):
            rrt.add_node(point=goal_xy.copy(),cost=cg,node_parent=n_new)

path_to_goal, _ = rrt.get_path_to_goal()
assert path_to_goal is not None, "No path found!"
print(f"Path: {len(path_to_goal)} waypoints")

# ── Offscreen renderer setup ──
W, H = 480, 360
# Set offscreen framebuffer size
env.model.vis.global_.offwidth = W
env.model.vis.global_.offheight = H
renderer = mujoco.Renderer(env.model, height=H, width=W)
scene_opt = mujoco.MjvOption()
scene_opt.sitegroup[:] = 0
scene_opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = 0  # hide yellow lidar rays

# Camera: top-down oblique view of the maze
maze_cx = n_cols * CELL_SIZE / 2
maze_cy = -n_rows * CELL_SIZE / 2
cam = mujoco.MjvCamera()
cam.lookat[:] = [maze_cx, maze_cy, 0.0]
cam.distance = 30.0
cam.azimuth = 90.0
cam.elevation = -60.0

# ── Add waypoint markers to the scene XML ──
# Regenerate XML with path waypoint spheres baked in
def gen_xml_with_waypoints(path):
    walls=[]
    for r in range(n_rows):
        for c in range(n_cols):
            if MAZE_MAP[r,c]==1:
                x,y=cell_to_world(r,c)
                walls.append(f'    <geom name="wall_{len(walls)}" type="box" '
                    f'pos="{x:.3f} {y:.3f} {WALL_HEIGHT/2:.3f}" '
                    f'size="{CELL_SIZE/2:.3f} {CELL_SIZE/2:.3f} {WALL_HEIGHT/2:.3f}" '
                    f'rgba="0.6 0.65 0.7 1" contype="1" conaffinity="1"/>')
    # Goal marker
    walls.append(f'    <geom name="goal_marker" type="sphere" pos="{goal_x:.3f} {goal_y:.3f} 0.5" '
                 f'size="0.3" rgba="1 0.1 0.1 0.7" contype="0" conaffinity="0"/>')
    # Waypoint spheres: start/goal=black, others=magenta (changed to green at runtime)
    for i, wp in enumerate(path):
        if i == 0:
            rgba = "0.1 0.1 0.1 0.9"  # start = black
        elif i == len(path)-1:
            rgba = "0.1 0.1 0.1 0.9"  # goal = black
        else:
            rgba = "1 0 1 0.6"         # unvisited = magenta
        walls.append(f'    <geom name="wp_{i}" type="sphere" pos="{wp[0]:.3f} {wp[1]:.3f} 0.4" '
                     f'size="0.15" rgba="{rgba}" contype="0" conaffinity="0"/>')
    # Path cylinders between waypoints
    for i in range(len(path)-1):
        mx = (path[i][0]+path[i+1][0])/2
        my = (path[i][1]+path[i+1][1])/2
        dx = path[i+1][0]-path[i][0]; dy = path[i+1][1]-path[i][1]
        length = np.sqrt(dx**2+dy**2)
        angle = np.arctan2(dy, dx)
        # Capsule along path segment
        walls.append(f'    <geom name="pathseg_{i}" type="capsule" '
                     f'pos="{mx:.3f} {my:.3f} 0.4" '
                     f'size="0.05 {length/2:.3f}" '
                     f'euler="0 1.5708 {angle:.4f}" '
                     f'rgba="1 0 1 0.4" contype="0" conaffinity="0"/>')
    xml=('<mujoco model="husky maze">\n  <include file="./husky_w_lidar.xml"/>\n'
         '  <include file="../object/floor_sky.xml"/>\n  <worldbody>\n'
         +'\n'.join(walls)+'\n  </worldbody>\n</mujoco>\n')
    with open('../../asset/husky/scene_husky_maze_slam.xml','w') as f: f.write(xml)

gen_xml_with_waypoints(path_to_goal)
# Reload model with waypoints
env = MuJoCoParserClass(name='Husky',
    rel_xml_path='../../asset/husky/scene_husky_maze_slam.xml', VERBOSE=False)
# Recreate renderer with new model
renderer.close()
env.model.vis.global_.offwidth = W
env.model.vis.global_.offheight = H
renderer = mujoco.Renderer(env.model, height=H, width=W)

# ── Navigation + Render ──
print("\n=== Navigation (offscreen 3D rendering) ===")
env.data.qpos[0]=start_x; env.data.qpos[1]=start_y; env.data.qpos[2]=0.1
env.data.qpos[3:7]=[1,0,0,0]; env.data.qvel[:]=0
mujoco.mj_forward(env.model, env.data)

frames = []
prev_ci = 1

# Precompute geom IDs for waypoints and path segments
wp_geom_ids = [mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, f'wp_{i}')
               for i in range(len(path_to_goal))]
seg_geom_ids = [mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, f'pathseg_{i}')
                for i in range(len(path_to_goal)-1)]

for step in range(50000):
    p = env.get_p_body('base_husky')
    yaw = r2rpy(env.get_R_body('base_husky'))[2]

    # Goal check
    if np.linalg.norm(p[:2]-goal_xy) < 1.0:
        print(f"  Goal reached at t={env.get_sim_time():.1f}s!")
        # Turn goal waypoint green
        gid = wp_geom_ids[-1]
        if gid >= 0: env.model.geom_rgba[gid] = [0.1, 0.9, 0.2, 0.9]
        for _ in range(10):
            renderer.update_scene(env.data, cam, scene_option=scene_opt)
            frames.append(Image.fromarray(renderer.render()))
        break

    # Advance ci
    ci = prev_ci
    while ci < len(path_to_goal)-1 and np.linalg.norm(path_to_goal[ci]-p[:2]) < 0.8:
        ci += 1
    while ci < len(path_to_goal)-1:
        if np.linalg.norm(path_to_goal[ci+1]-p[:2]) < np.linalg.norm(path_to_goal[ci]-p[:2]):
            ci += 1
        else: break

    # Color update: turn passed waypoints/segments green
    if ci > prev_ci:
        for j in range(prev_ci, ci):
            # Skip start (index 0) — keep black
            if j > 0 and j < len(path_to_goal)-1:
                gid = wp_geom_ids[j]
                if gid >= 0: env.model.geom_rgba[gid] = [0.1, 0.9, 0.2, 0.8]
            if j < len(path_to_goal)-1:
                sid = seg_geom_ids[j]
                if sid >= 0: env.model.geom_rgba[sid] = [0.1, 0.9, 0.2, 0.5]
            # Also color the segment before this waypoint
            if j > 0:
                sid = seg_geom_ids[j-1]
                if sid >= 0: env.model.geom_rgba[sid] = [0.1, 0.9, 0.2, 0.5]
    prev_ci = ci

    # Target tracking (stronger gains for corners)
    target = path_to_goal[min(ci, len(path_to_goal)-1)]
    d_pos = target - p[:2]
    dist = np.linalg.norm(d_pos)
    heading = np.arctan2(d_pos[1], d_pos[0]) - yaw
    heading = (heading+np.pi)%(2*np.pi)-np.pi
    v_cmd = 8.0 * np.tanh(dist) * max(np.cos(heading), 0.1)
    w_cmd = 80.0 * np.tanh(heading)
    vl = v_cmd - (w_cmd * wheel_base / 2)
    vr = v_cmd + (w_cmd * wheel_base / 2)
    env.step(ctrl=np.array([vl,vr,vl,vr], dtype=np.float32), ctrl_idxs=[0,1,2,3])

    # Capture frame every 1s
    if env.loop_every(HZ=1):
        renderer.update_scene(env.data, cam, scene_option=scene_opt)
        img = renderer.render()
        frames.append(Image.fromarray(img))

    if env.loop_every(HZ=0.5):
        print(f"  t={env.get_sim_time():.0f}s, ci={ci}/{len(path_to_goal)}")

    # Timeout
    if env.get_sim_time() > 100:
        print(f"  Timeout at wp {ci}")
        break

renderer.close()

# Save GIF
gif_path = os.path.abspath(os.path.join(os.path.dirname(__file__),
    '../../asset/husky_3d_rrt_navigation.gif'))
if frames:
    frames[0].save(gif_path, save_all=True, append_images=frames[1:],
                   duration=100, loop=0, optimize=True)
    print(f"\nGIF: {gif_path}")
    print(f"Frames: {len(frames)}, Size: {os.path.getsize(gif_path)/1024:.0f} KB")
