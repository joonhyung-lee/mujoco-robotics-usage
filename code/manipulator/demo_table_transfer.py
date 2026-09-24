"""Grasp a free upright bar on table A, navigate/contact a wall, and place on B."""

import argparse
from contextlib import ExitStack
import json
import os
from pathlib import Path
import time

if not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import mujoco
import numpy as np

from interaction_mpc import ForceObserver, InteractionMPC, dynamics, kinematics
from table_transfer import (BAR_HALF, CLOSED, GRASP_OFFSET, GRIPPER_JOINTS, LIFT, OPEN, TABLE_A,
    TABLE_B, TABLE_Z, contact_measurements, detour, inverse_kinematics, make_transfer_model)


class TransferTask:
    """Event-gated grasp/placement plus a known-wall route or reactive recovery."""

    def __init__(self, case, initial):
        self.case = case
        self.grasp = TABLE_A + [0, 0, BAR_HALF[2] + GRASP_OFFSET]
        self.destination = TABLE_B + [0, 0, BAR_HALF[2] + GRASP_OFFSET]
        self.events, self.route = [], []
        self.grasped = self.lifted = self.reacted = False
        self.failed = ""
        self.contact_since = None
        self.enter("approach", 0, initial, self.grasp, 3)

    def enter(self, phase, t, position, target, duration=None):
        self.phase, self.start_time = phase, t
        self.start, self.target = np.array(position), np.array(target)
        self.duration = duration if duration is not None else max(1.5, np.linalg.norm(self.target-self.start)/0.045)
        self.events.append({"phase": phase, "time_s": round(t, 3), "target": self.target.tolist()})
        print(f"  {t:5.2f}s {phase}", flush=True)

    def reference(self, times):
        progress = np.clip((np.asarray(times)-self.start_time)/self.duration, 0, 1)
        progress = (1-np.cos(np.pi*progress))/2
        return self.start + progress[:, None]*(self.target-self.start)

    def update(self, t, position, bar, grip, force):
        elapsed = t-self.start_time
        reached = np.linalg.norm(position-self.target) < 0.012
        finished = elapsed >= self.duration and reached
        if elapsed > self.duration + 6 and self.phase not in ("touch", "settle", "done"):
            self.failed = f"{self.phase}: target not reached"
        if self.phase == "approach" and finished:
            self.enter("close", t, position, self.grasp, 2)
        elif self.phase == "close" and elapsed > 2:
            if np.min(grip) > 2:
                self.grasped = True
                self.enter("lift", t, position, self.grasp+[0, 0, LIFT])
            elif elapsed > 4:
                self.failed = "grasp failed: bilateral finger contact was not established"
        elif self.phase == "lift" and finished:
            self.lifted = bool(bar[2]-BAR_HALF[2] > TABLE_Z + 0.65*LIFT)
            if not self.lifted:
                self.failed = "grasp failed: bar did not lift off table A"
            elif self.case == "avoid":
                self.route = detour(position, self.destination+[0, 0, LIFT])[1:]
                self.enter("detour_1", t, position, self.route.pop(0))
            else:
                self.enter("touch", t, position, [position[0], 0.025, position[2]], 5)
        elif self.phase == "touch":
            # Uses the encoder/torque observer, not simulator wall-contact truth.
            opposing = force[1] < -5 and self.target[1]-position[1] > 0.015
            self.contact_since = (t if self.contact_since is None else self.contact_since) if opposing else None
            if self.contact_since is not None and t-self.contact_since > 0.12:
                self.reacted = True
                self.enter("yield", t, position, self.reference(np.array([t]))[0], 1.0)
            elif elapsed > 7:
                self.failed = "wall reaction was not detected"
        elif self.phase == "yield" and elapsed > 1.2:
            self.enter("backoff", t, position, [position[0], -0.12, self.grasp[2]+LIFT], 2.5)
        elif self.phase == "backoff" and finished:
            self.route = detour(position, self.destination+[0, 0, LIFT])[1:]
            self.enter("detour_1", t, position, self.route.pop(0))
        elif self.phase.startswith("detour") and finished:
            if self.route:
                number = int(self.phase.split("_")[1])+1
                self.enter(f"detour_{number}", t, position, self.route.pop(0))
            else:
                self.enter("lower", t, position, self.destination, 4)
        elif self.phase == "lower" and finished:
            self.enter("release", t, position, self.destination, 2)
        elif self.phase == "release" and elapsed > 2:
            self.enter("withdraw", t, position, self.destination+[0, 0, 0.14], 3)
        elif self.phase == "withdraw" and finished:
            self.enter("settle", t, position, position, 2)
        elif self.phase == "settle" and elapsed > 2:
            self.enter("done", t, position, position, 1)

    def jaw_target(self):
        return OPEN if self.phase in ("approach", "release", "withdraw", "settle", "done") else CLOSED


def run_transfer(robot="ur5e", case="avoid", output=None, video=False, viewer=False,
                 mass=0.4, friction=1.2, max_time=70, grip_torque=2):
    plant, state, seed, limits = make_transfer_model(robot, mass=mass, friction=friction)
    nominal, estimate, _, _ = make_transfer_model(robot, scene=False)
    n = nominal.nv
    jaw_qpos = [plant.joint(name).qposadr[0] for name in GRIPPER_JOINTS]
    bar_dof = plant.joint("bar_free").dofadr[0]
    pregrasp = TABLE_A + [0, 0, BAR_HALF[2]+GRASP_OFFSET+0.12]
    home = inverse_kinematics(nominal, estimate, pregrasp, seed)
    # Check the planned Cartesian waypoints before starting physics.
    reach_seed = home.copy()
    high_a = TABLE_A + [0, 0, BAR_HALF[2]+GRASP_OFFSET+LIFT]
    high_b = TABLE_B + [0, 0, BAR_HALF[2]+GRASP_OFFSET+LIFT]
    for point in [TABLE_A+[0, 0, BAR_HALF[2]+GRASP_OFFSET], *detour(high_a, high_b),
                  TABLE_B+[0, 0, BAR_HALF[2]+GRASP_OFFSET]]:
        reach_seed = inverse_kinematics(nominal, estimate, point, reach_seed)
    state.qpos[:n], estimate.qpos[:] = home, home
    state.ctrl[n:] = OPEN
    plant.actuator_forcerange[n:] = [-grip_torque, grip_torque]
    mujoco.mj_forward(plant, state)
    mujoco.mj_forward(nominal, estimate)
    mpc = InteractionMPC(nominal, estimate, home, limits, orientation_weight=3000)
    observer = ForceObserver(n, wrench_dim=6)
    observed = mujoco.MjData(nominal)
    task = TransferTask(case, pregrasp)
    records = {key: [] for key in ["time", "phase", "q", "position", "reference", "bar_position",
        "bar_quaternion", "jaw_position", "grip_normal_n", "table_support_n", "wall_force",
        "wrench_estimate", "torque", "margin", "slack", "solver_ok"]}
    dt, stride, torque, jaw = plant.opt.timestep, 5, np.zeros(n), OPEN
    peak_wall, peak_other_wall, lift_height, minimum_margin = 0., 0., 0., 1.
    command_times = []
    output = Path(output) if output else None
    if output:
        output.mkdir(parents=True, exist_ok=True)
    with ExitStack() as stack:
        renderer = writer = live = None
        camera = mujoco.MjvCamera()
        camera.lookat[:] = [0.32, 0, 0.38]
        camera.distance, camera.azimuth, camera.elevation = 1.65, 135, -27
        if video:
            import imageio.v2 as imageio
            renderer = stack.enter_context(mujoco.Renderer(plant, height=720, width=960))
            writer = stack.enter_context(imageio.get_writer(str(output/"simulation.mp4"),
                                         fps=25, codec="libx264", quality=8))
        if viewer:
            import mujoco.viewer as mj_viewer
            live = stack.enter_context(mj_viewer.launch_passive(plant, state))
            live.cam.lookat[:], live.cam.distance = camera.lookat, camera.distance
            live.cam.azimuth, live.cam.elevation = camera.azimuth, camera.elevation
        wall_start = time.perf_counter()
        for step in range(int(np.ceil(max_time/dt))):
            t = step*dt
            observed.qpos[:], observed.qvel[:] = state.qpos[:n], state.qvel[:n]
            mujoco.mj_forward(nominal, observed)
            M, h = dynamics(nominal, observed)
            position, _, jp, jr = kinematics(nominal, observed)
            old_velocity = state.qvel[:n].copy()
            grip, support, wall_force, other_wall = contact_measurements(plant, state)
            peak_wall = max(peak_wall, float(np.linalg.norm(wall_force)))
            peak_other_wall = max(peak_other_wall, float(other_wall))
            bar_position = state.body("bar").xpos.copy()
            lift_height = max(lift_height, float(bar_position[2]-BAR_HALF[2]-TABLE_Z))
            if step % stride == 0:
                task.update(t, position, bar_position, grip, observer.force[:3])
                if task.lifted and task.phase in ("touch", "yield", "backoff", "detour_1", "detour_2", "detour_3"):
                    if np.linalg.norm(bar_position-(position-[0, 0, GRASP_OFFSET])) > 0.055:
                        task.failed = "grasp lost during transport"
                if task.failed or task.phase == "done":
                    break
                tick = time.perf_counter()
                torque, info = mpc.command(state.qpos[:n], state.qvel[:n], observer.force[:3],
                    task.reference(t+mpc.times), external_torque=observer.residual)
                command_times.append(1000*(time.perf_counter()-tick))
                minimum_margin = min(minimum_margin, float(np.min(1-np.abs(torque)/limits)))
                for key, value in {"time":t, "phase":task.phase, "q":state.qpos[:n],
                    "position":position, "reference":task.reference([t])[0],
                    "bar_position":bar_position, "bar_quaternion":state.body("bar").xquat,
                    "jaw_position":state.qpos[jaw_qpos], "grip_normal_n":grip,
                    "table_support_n":support, "wall_force":wall_force,
                    "wrench_estimate":observer.force, "torque":torque,
                    "margin":1-np.abs(torque)/limits, "slack":info["slack"],
                    "solver_ok":info["status"] in ("solved", "solved inaccurate")}.items():
                    records[key].append(np.array(value).copy())
            jaw += np.clip(task.jaw_target()-jaw, -0.7*dt, 0.7*dt)
            state.ctrl[:n], state.ctrl[n:] = torque, jaw
            state.mocap_pos[0] = task.reference([t])[0]
            mujoco.mj_step(plant, state)
            observer.update(M, h, np.vstack([jp, jr]), old_velocity, state.qvel[:n],
                            state.qfrc_actuator[:n], dt, nominal.dof_damping)
            mujoco.mj_forward(plant, state)
            if not np.all(np.isfinite(state.qpos)) or abs(state.time-(step+1)*dt) > dt/2:
                raise RuntimeError("Simulation became unstable or reset")
            if renderer and step % 20 == 0:
                renderer.update_scene(state, camera=camera)
                writer.append_data(renderer.render())
            if live and step % stride == 0:
                if not live.is_running():
                    task.failed = "viewer closed before task completion"
                    break
                live.sync()
                time.sleep(max(0, state.time-(time.perf_counter()-wall_start)))
    if task.phase != "done" and not task.failed:
        task.failed = "simulation timeout"
    grip, support, wall_force, _ = contact_measurements(plant, state)
    bar = state.body("bar")
    xy_error = float(np.linalg.norm(bar.xpos[:2]-TABLE_B[:2]))
    bottom_error = float(abs(bar.xpos[2]-BAR_HALF[2]-TABLE_Z))
    upright = float(bar.xmat.reshape(3, 3)[2, 2])
    speed = float(np.linalg.norm(state.qvel[bar_dof:bar_dof+6]))
    placed = (xy_error < 0.035 and bottom_error < 0.007 and upright > 0.98 and
              support[1] > mass*9.81*0.5 and np.max(grip) < 0.2 and speed < 0.15)
    success = task.phase == "done" and placed and task.lifted and (case == "avoid" or task.reacted)
    if not success and not task.failed:
        task.failed = "bar was not released upright and supported on table B"
    summary = {"robot":robot, "gripper":"repository_rg2", "case":case, "success":bool(success), "failure":task.failed,
        "duration_s":float(state.time), "mass_kg":mass, "friction":friction,
        "grip_motor_torque_limit_nm":grip_torque, "grasp_verified":task.grasped,
        "lift_verified":task.lifted, "reaction_detected":task.reacted,
        "peak_bar_wall_force_n":peak_wall, "peak_other_wall_force_n":peak_other_wall,
        "max_lift_height_m":lift_height, "final_xy_error_m":xy_error,
        "final_bottom_error_m":bottom_error, "final_upright_cosine":upright,
        "final_bar_speed":speed, "final_table_b_support_n":float(support[1]),
        "final_grip_normal_n":grip.tolist(), "minimum_torque_margin":minimum_margin,
        "solver_failures":mpc.failures, "command_p95_ms":float(np.percentile(command_times, 95)),
        "events":task.events}
    log = {key:np.asarray(value) for key, value in records.items()}
    if output:
        np.savez_compressed(output/"trace.npz", **log)
        (output/"summary.json").write_text(json.dumps(summary, indent=2)+"\n")
        plot_transfer(log, summary, output/"metrics.png")
    return log, summary


def plot_transfer(log, summary, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    from table_transfer import WALL_CENTER, WALL_HALF
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    t = log["time"]
    axes[0, 0].plot(log["bar_position"][:, 0], log["bar_position"][:, 1], label="bar COM")
    axes[0, 0].plot(log["reference"][:, 0], log["reference"][:, 1], "--", label="EE reference")
    axes[0, 0].add_patch(Rectangle(WALL_CENTER[:2]-WALL_HALF[:2], *(2*WALL_HALF[:2]), alpha=0.3))
    axes[0, 0].scatter([TABLE_A[0], TABLE_B[0]], [TABLE_A[1], TABLE_B[1]])
    axes[0, 0].set(xlabel="x [m]", ylabel="y [m]", title="Table A to B: top view", aspect="equal")
    axes[0, 0].legend()
    axes[0, 1].plot(t, log["bar_position"][:, 2]-BAR_HALF[2], label="bar bottom (upright approx.)")
    axes[0, 1].axhline(TABLE_Z, color="k", linestyle="--", label="table surface")
    axes[0, 1].set_ylabel("Height [m]")
    axes[0, 1].legend()
    axes[0, 2].plot(t, log["grip_normal_n"])
    axes[0, 2].set_ylabel("Left / right grip normal [N]")
    axes[1, 0].plot(t, np.linalg.norm(log["wall_force"], axis=1))
    axes[1, 0].set_ylabel("Bar-wall contact [N]")
    axes[1, 0].set_title(f"500 Hz peak: {summary['peak_bar_wall_force_n']:.1f} N")
    axes[1, 1].plot(t, log["margin"])
    axes[1, 1].set(ylabel="Torque margin", ylim=(-0.02, 1.02))
    axes[1, 2].plot(t, log["table_support_n"][:, 0], label="table A")
    axes[1, 2].plot(t, log["table_support_n"][:, 1], label="table B")
    axes[1, 2].set_ylabel("Bar support [N]")
    axes[1, 2].legend()
    for ax in axes.flat:
        ax.grid(alpha=0.25)
        if ax is not axes[0, 0]:
            ax.set_xlabel("Time [s]")
    fig.suptitle(f"{summary['robot']} / {summary['case']} / success={summary['success']}")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", choices=["ur5e", "franka", "both"], default="both")
    parser.add_argument("--case", choices=["avoid", "contact", "both"], default="both")
    parser.add_argument("--mass", type=float, default=0.4)
    parser.add_argument("--friction", type=float, default=1.2)
    parser.add_argument("--grip-torque", type=float, default=2, help="RG2 driving-joint torque limit [Nm]")
    parser.add_argument("--max-time", type=float, default=70)
    parser.add_argument("--output", type=Path, default=Path("outputs/table_transfer_rg2"))
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--viewer", action="store_true")
    args = parser.parse_args()
    for name in ("mass", "friction", "max_time", "grip_torque"):
        value = getattr(args, name)
        if not np.isfinite(value) or value < 0 or (value == 0 and name in ("mass", "max_time")):
            parser.error(f"invalid --{name.replace('_', '-')}")
    summaries = []
    for robot in (["ur5e", "franka"] if args.robot == "both" else [args.robot]):
        for case in (["avoid", "contact"] if args.case == "both" else [args.case]):
            print(f"Running {robot} / {case}", flush=True)
            _, summary = run_transfer(robot, case, args.output/f"{robot}_{case}", args.video,
                args.viewer, args.mass, args.friction, args.max_time, args.grip_torque)
            summaries.append(summary)
            print(json.dumps(summary, indent=2), flush=True)
    (args.output/"comparison.json").write_text(json.dumps(summaries, indent=2)+"\n")
    if not all(s["success"] for s in summaries):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
