#!/usr/bin/env python3
"""
Render SLAM exploration → goal discovery → RRT* → Navigation as GIF.
Realistic pipeline: robot doesn't know the goal until it observes it.
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from PIL import Image
import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__), '../../'))

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

class OccupancyGrid:
    def __init__(self, x_range, y_range, resolution=0.15):
        self.resolution = resolution
        self.x_min, self.x_max = x_range; self.y_min, self.y_max = y_range
        self.width = int(np.ceil((self.x_max-self.x_min)/resolution))
        self.height = int(np.ceil((self.y_max-self.y_min)/resolution))
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        self.l_occ=0.7; self.l_free=-0.4; self.l_max=5.0; self.l_min=-5.0
    def world_to_grid(self, x, y):
        return (np.clip(int((x-self.x_min)/self.resolution),0,self.width-1),
                np.clip(int((y-self.y_min)/self.resolution),0,self.height-1))
    def bresenham(self, x0, y0, x1, y1):
        cells=[]; dx,dy=abs(x1-x0),abs(y1-y0)
        sx=1 if x0<x1 else -1; sy=1 if y0<y1 else -1; err=dx-dy
        while True:
            cells.append((x0,y0))
            if x0==x1 and y0==y1: break
            e2=2*err
            if e2>-dy: err-=dy; x0+=sx
            if e2<dx: err+=dx; y0+=sy
        return cells
    def update(self, rx, ry, yaw, ranges, n_rays=72, max_range=10.0):
        gx0,gy0=self.world_to_grid(rx,ry)
        angles=np.linspace(0,2*np.pi,n_rays,endpoint=False)
        for i in range(n_rays):
            a=angles[i]+yaw; r=ranges[i]; hit=r<max_range-0.1
            ex,ey=self.world_to_grid(rx+r*np.cos(a),ry+r*np.sin(a))
            for cx,cy in self.bresenham(gx0,gy0,ex,ey)[:-1]:
                if 0<=cx<self.width and 0<=cy<self.height: self.log_odds[cy,cx]+=self.l_free
            if hit and 0<=ex<self.width and 0<=ey<self.height: self.log_odds[ey,ex]+=self.l_occ
        np.clip(self.log_odds,self.l_min,self.l_max,out=self.log_odds)
    def get_map_image(self):
        return 1.0-1.0/(1.0+np.exp(self.log_odds))
    def is_observed(self, x, y):
        gx,gy=self.world_to_grid(x,y)
        return abs(self.log_odds[gy,gx]) > 0.3  # not unknown
    def is_free(self, x, y, margin=0.3):
        gx,gy=self.world_to_grid(x,y); r=int(margin/self.resolution)
        for dx in range(-r,r+1):
            for dy in range(-r,r+1):
                cx,cy=gx+dx,gy+dy
                if not(0<=cx<self.width and 0<=cy<self.height): return False
                if self.log_odds[cy,cx]>=0.0: return False  # unknown(0) and occupied(>0) both blocked
        return True
    def is_line_free(self, x1, y1, x2, y2, margin=0.3):
        r=int(margin/self.resolution)
        for cx,cy in self.bresenham(*self.world_to_grid(x1,y1),*self.world_to_grid(x2,y2)):
            for ddx in range(-r,r+1):
                for ddy in range(-r,r+1):
                    nx_,ny_=cx+ddx,cy+ddy
                    if not(0<=nx_<self.width and 0<=ny_<self.height): return False
                    if self.log_odds[ny_,nx_]>=0.0: return False
        return True

def render_frame(og, traj, path=None, rrt_tree=None, robot_xy=None, robot_yaw=None,
                 ranges=None, title="", goal_discovered=False):
    fig, ax = plt.subplots(1, 1, figsize=(6, 6), dpi=100)
    m = og.get_map_image()
    ax.imshow(m, cmap='gray_r', origin='lower',
              extent=[og.x_min,og.x_max,og.y_min,og.y_max], vmin=0, vmax=1)
    for r in range(n_rows):
        for c in range(n_cols):
            if MAZE_MAP[r,c]==1:
                wx,wy=cell_to_world(r,c)
                ax.add_patch(Rectangle((wx-CELL_SIZE/2,wy-CELL_SIZE/2),
                    CELL_SIZE,CELL_SIZE,lw=0.5,ec='blue',fc='none',alpha=0.1))
    if len(traj)>1:
        t=np.array(traj); ax.plot(t[:,0],t[:,1],'-',color='lime',lw=1.5,alpha=0.5)
    # RRT* tree edges (green, faint)
    if rrt_tree is not None:
        rrt_obj, = rrt_tree  # unpack tuple
        pos={n:rrt_obj.get_node_point(n) for n in rrt_obj.get_nodes()}
        for (n1,n2) in rrt_obj.get_edges():
            p1,p2=pos[n1],pos[n2]
            ax.plot([p1[0],p2[0]],[p1[1],p2[1]],'-',color=[0.2,0.8,0.3,0.35],lw=0.7)
        # Root and nodes
        ax.plot(rrt_obj.point_root[0],rrt_obj.point_root[1],'o',color='green',ms=6,mew=1.5,mfc='none',zorder=8)
    if path is not None:
        ax.plot(path[:,0],path[:,1],'o-',color='magenta',lw=2.5,ms=4,mec='k',mfc='m',alpha=0.9,zorder=7)
    if robot_xy is not None:
        ax.plot(robot_xy[0],robot_xy[1],'o',color='lime',ms=8,mfc='lime',zorder=10)
        if robot_yaw is not None:
            ax.arrow(robot_xy[0],robot_xy[1],0.8*np.cos(robot_yaw),0.8*np.sin(robot_yaw),
                     head_width=0.3,head_length=0.2,fc='lime',ec='k',lw=0.5,zorder=10)
        if ranges is not None:
            n=len(ranges); angles=np.linspace(0,2*np.pi,n,endpoint=False)+robot_yaw
            hx=robot_xy[0]+ranges*np.cos(angles); hy=robot_xy[1]+ranges*np.sin(angles)
            idxs=np.where(ranges<9.9)[0]
            for k in range(len(idxs)):
                i0,i1=idxs[k],idxs[(k+1)%len(idxs)]
                if abs(i1-i0)<=2 or abs(i1-i0)>=n-2:
                    ax.plot([hx[i0],hx[i1]],[hy[i0],hy[i1]],'-',color='red',lw=0.8,alpha=0.35)
    ax.plot(start_x,start_y,'gs',ms=10,mew=2,mfc='none',zorder=9)
    goal_color = 'r' if goal_discovered else 'gray'
    goal_label = 'Goal' if goal_discovered else 'Goal (undiscovered)'
    ax.plot(goal_x,goal_y,'*',color=goal_color,ms=14,zorder=9)
    ax.set_xlim(og.x_min+0.5,og.x_max-0.5); ax.set_ylim(og.y_min+0.5,og.y_max-0.5)
    ax.set_aspect('equal'); ax.set_title(title,fontsize=11,fontweight='bold')
    ax.set_xlabel('x [m]',fontsize=8); ax.set_ylabel('y [m]',fontsize=8)
    plt.tight_layout()
    fig.canvas.draw()
    img=np.asarray(fig.canvas.buffer_rgba())[:,:,:3].copy()
    plt.close(fig)
    return Image.fromarray(img)

# ── MuJoCo ──
import mujoco
from utils.mujoco_parser import MuJoCoParserClass
from utils.util import r2rpy
from utils.rrt import RapidlyExploringRandomTreesStarClass

def gen_xml():
    walls=[]
    for r in range(n_rows):
        for c in range(n_cols):
            if MAZE_MAP[r,c]==1:
                x,y=cell_to_world(r,c)
                walls.append(f'    <geom name="wall_{len(walls)}" type="box" '
                    f'pos="{x:.3f} {y:.3f} {WALL_HEIGHT/2:.3f}" '
                    f'size="{CELL_SIZE/2:.3f} {CELL_SIZE/2:.3f} {WALL_HEIGHT/2:.3f}" '
                    f'rgba="0.6 0.65 0.7 1" contype="1" conaffinity="1"/>')
    xml=('<mujoco model="husky maze">\n  <include file="./husky_w_lidar.xml"/>\n'
         '  <include file="../object/floor_sky.xml"/>\n  <worldbody>\n'
         +'\n'.join(walls)+'\n  </worldbody>\n</mujoco>\n')
    with open('../../asset/husky/scene_husky_maze_slam.xml','w') as f: f.write(xml)
gen_xml()
env = MuJoCoParserClass(name='Husky',
    rel_xml_path='../../asset/husky/scene_husky_maze_slam.xml', VERBOSE=False)

N_LIDAR=72; MAX_RANGE=10.0; wheel_base=0.2854*2; wheel_radius=0.17775/2.0
def get_lidar(env):
    ranges=np.zeros(N_LIDAR)
    for i in range(N_LIDAR):
        v=env.get_sensor_value(f'lidar_{i:03d}')[0]
        ranges[i]=v if v>=0 else MAX_RANGE
    return ranges

margin=2.0
og = OccupancyGrid(x_range=(-margin, n_cols*CELL_SIZE+margin),
                    y_range=(-(n_rows*CELL_SIZE+margin), margin), resolution=0.15)
env.data.qpos[0]=start_x; env.data.qpos[1]=start_y; env.data.qpos[2]=0.1
env.data.qpos[3:7]=[1,0,0,0]; env.data.qvel[:]=0
mujoco.mj_forward(env.model, env.data)

frames=[]; trajectory=[]; v,w=0.0,0.0; ranges=get_lidar(env)
goal_discovered = False
goal_discovery_time = None
path_to_goal = None

# ═══════════════════════════════════════
# Phase 1: SLAM Exploration (wall-following)
# Explore until goal is discovered, then continue 40s more to fill the map
# ═══════════════════════════════════════
print("=== Phase 1: SLAM Exploration ===")
EXPLORE_AFTER_DISCOVERY = 40.0  # keep exploring 40s after finding goal

for step in range(200000):  # max ~400s
    p=env.get_p_body('base_husky')
    yaw=r2rpy(env.get_R_body('base_husky'))[2]

    if env.loop_every(HZ=5) or env.tick==1:
        ranges=get_lidar(env)
        og.update(p[0],p[1],yaw,ranges,N_LIDAR,MAX_RANGE)
        # Wall following
        front=np.min(ranges[list(range(0,7))+list(range(65,72))])
        right=np.min(ranges[49:61])
        if front<1.2: v,w=0.2,2.0
        elif right>1.1: v,w=1.0,-1.2
        elif right<0.5: v,w=1.2,0.8
        else: v,w=1.5,0.0
        trajectory.append([p[0],p[1]])

        # Check if goal region is observed
        if not goal_discovered and og.is_observed(goal_x, goal_y):
            goal_discovered = True
            goal_discovery_time = env.get_sim_time()
            print(f"  Goal discovered at t={goal_discovery_time:.0f}s! Continuing exploration...")
            frames.append(render_frame(og,trajectory,robot_xy=p[:2],robot_yaw=yaw,
                ranges=ranges, title=f'Goal Discovered!  t={goal_discovery_time:.0f}s',
                goal_discovered=True))

    vl=v-(w*wheel_base/2); vr=v+(w*wheel_base/2)
    ctrl=np.array([vl,vr,vl,vr])/wheel_radius
    np.clip(ctrl,-20,20,out=ctrl)
    env.step(ctrl=ctrl,ctrl_idxs=[0,1,2,3])

    if env.loop_every(HZ=1.0):  # 1 frame per second (faster capture)
        frames.append(render_frame(og,trajectory,robot_xy=p[:2],robot_yaw=yaw,
            ranges=ranges,
            title=f'Phase 1: SLAM Exploration  t={env.get_sim_time():.0f}s',
            goal_discovered=goal_discovered))
        if int(env.get_sim_time()) % 10 == 0:
            print(f"  t={env.get_sim_time():.0f}s, goal={goal_discovered}")

    # Stop condition: explored enough after discovery
    if goal_discovered and env.get_sim_time() > goal_discovery_time + EXPLORE_AFTER_DISCOVERY:
        print(f"  Exploration complete at t={env.get_sim_time():.0f}s")
        break
    if env.get_sim_time() > 200:  # safety timeout
        print("  Timeout — using partial map")
        break

# Hold on SLAM map result
for _ in range(3):
    frames.append(render_frame(og,trajectory,
        title='SLAM Map Complete — Planning...', goal_discovered=goal_discovered))
print(f"Phase 1: {len(frames)} frames")

# ═══════════════════════════════════════
# Phase 2: RRT* on SLAM map (goal now observed)
# ═══════════════════════════════════════
print("\n=== Phase 2: RRT* on SLAM Map (observed free space only) ===")
goal_xy=np.array([goal_x,goal_y])
rrt=RapidlyExploringRandomTreesStarClass(
    name='RRT',point_min=np.array([og.x_min+0.5,og.y_min+0.5]),
    point_max=np.array([og.x_max-0.5,og.y_max-0.5]),
    goal_select_rate=0.15,steer_len_max=1.0,search_radius=2.0,
    n_node_max=5000,SPEED_UP=True)
rrt.init_rrt_star(point_root=np.array([start_x,start_y]),point_goal=goal_xy,seed=42)

N_RRT_ITER=5000
RENDER_EVERY=500  # render tree growth every N iterations
goal_found_iter=None

for it in range(N_RRT_ITER):
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
            if goal_found_iter is None:
                goal_found_iter=it
                print(f"  Goal connected at iter {it}, nodes={rrt.get_n_node()}")

    # Render tree growth periodically
    if (it+1)%RENDER_EVERY==0:
        p_tmp,_=rrt.get_path_to_goal()
        frames.append(render_frame(og,trajectory,path=p_tmp,rrt_tree=(rrt,),
            title=f'RRT* Growing  iter={it+1}  nodes={rrt.get_n_node()}',
            goal_discovered=True))
        print(f"  iter={it+1}, nodes={rrt.get_n_node()}, path={'found' if p_tmp is not None else 'searching'}")

path_to_goal,_=rrt.get_path_to_goal()
if path_to_goal is not None:
    print(f"Path: {len(path_to_goal)} waypoints, cost={rrt.get_cost_goal():.1f}m")
    for _ in range(5):
        frames.append(render_frame(og,trajectory,path=path_to_goal,rrt_tree=(rrt,),
            title=f'RRT* Path Found! ({len(path_to_goal)} waypoints)',goal_discovered=True))
else:
    print("No path found!")
    for _ in range(3):
        frames.append(render_frame(og,trajectory,rrt_tree=(rrt,),
            title='RRT* — No Path Found',goal_discovered=True))

# ═══════════════════════════════════════
# Phase 3: Navigate to goal (demo_02 style, tuned)
# ═══════════════════════════════════════
if path_to_goal is not None:
    print("\n=== Phase 3: Navigation to Goal ===")
    env.data.qpos[:]=0; env.data.qvel[:]=0; env.data.ctrl[:]=0
    env.data.qpos[0]=start_x; env.data.qpos[1]=start_y; env.data.qpos[2]=0.1
    env.data.qpos[3:7]=[1,0,0,0]
    env.tick=0; env.render_tick=0
    mujoco.mj_forward(env.model, env.data)

    nav_og = OccupancyGrid(x_range=(-margin, n_cols*CELL_SIZE+margin),
                            y_range=(-(n_rows*CELL_SIZE+margin), margin), resolution=0.15)
    nav_traj=[]; prev_ci=1; stuck_count=0; last_ci=1

    for step in range(50000):
        p=env.get_p_body('base_husky')
        yaw=r2rpy(env.get_R_body('base_husky'))[2]

        if env.loop_every(HZ=5) or env.tick==1:
            ranges=get_lidar(env)
            nav_og.update(p[0],p[1],yaw,ranges,N_LIDAR,MAX_RANGE)

        if np.linalg.norm(p[:2]-goal_xy)<1.0:
            print(f"  Goal reached at t={env.get_sim_time():.1f}s!")
            for _ in range(5):
                frames.append(render_frame(nav_og,nav_traj,path=path_to_goal,
                    robot_xy=p[:2],robot_yaw=yaw,title='Goal Reached!',goal_discovered=True))
            break

        # Advance ci
        ci=prev_ci
        while ci<len(path_to_goal)-1 and np.linalg.norm(path_to_goal[ci]-p[:2])<0.8:
            ci+=1
        while ci<len(path_to_goal)-1:
            if np.linalg.norm(path_to_goal[ci+1]-p[:2])<np.linalg.norm(path_to_goal[ci]-p[:2]):
                ci+=1
            else: break
        prev_ci=ci

        # Target tracking (demo_02 style, v=5, w=50)
        target=path_to_goal[min(ci,len(path_to_goal)-1)]
        d_pos=target-p[:2]; dist=np.linalg.norm(d_pos)
        heading=np.arctan2(d_pos[1],d_pos[0])-yaw
        heading=(heading+np.pi)%(2*np.pi)-np.pi
        v_cmd = 5.0 * np.tanh(dist) * max(np.cos(heading), 0.1)
        w_cmd = 50.0 * np.tanh(heading)
        vl=v_cmd-(w_cmd*wheel_base/2); vr=v_cmd+(w_cmd*wheel_base/2)
        env.step(ctrl=np.array([vl,vr,vl,vr],dtype=np.float32),ctrl_idxs=[0,1,2,3])
        nav_traj.append([p[0],p[1]])

        if env.loop_every(HZ=0.5):
            frames.append(render_frame(nav_og,nav_traj,path=path_to_goal,
                robot_xy=p[:2],robot_yaw=yaw,ranges=ranges,
                title=f'Navigation + SLAM  wp={ci}/{len(path_to_goal)}',
                goal_discovered=True))
            print(f"  t={env.get_sim_time():.0f}s, ci={ci}/{len(path_to_goal)}")
            if ci==last_ci: stuck_count+=1
            else: stuck_count=0; last_ci=ci
            if stuck_count>=4:
                print(f"  Stopped at wp {ci}")
                break

# ── Save ──
gif_path=os.path.abspath(os.path.join(os.path.dirname(__file__),
    '../../asset/husky_slam_rrt_navigation.gif'))
frames[0].save(gif_path,save_all=True,append_images=frames[1:],
               duration=400,loop=0,optimize=True)
print(f"\nGIF: {gif_path}")
print(f"Frames: {len(frames)}, Size: {os.path.getsize(gif_path)/1024:.0f} KB")
