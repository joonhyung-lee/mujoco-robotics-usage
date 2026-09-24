"""Run a torque-controlled UR5e / Franka interaction experiment, without RL."""

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

from interaction_mpc import (ForceObserver, InteractionMPC, ROBOTS, dynamics,
                             kinematics, make_model)


def trajectory(times, origin):
    """Reach, hold beyond an unknown wall, then withdraw; identical in all tasks."""
    times = np.atleast_1d(times)
    smooth = lambda x: (1 - np.cos(np.pi * np.clip(x, 0, 1))) / 2
    progress = smooth((times - 1) / 4) - smooth((times - 8) / 4)
    return origin + progress[:, None] * np.array([0.17, 0.035, 0.025])


def contact_force(model, data):
    """Evaluation ONLY: sum world-frame contact forces on the tool sphere."""
    total = np.zeros(3)
    tool = model.geom("tool_geom").id
    for i in range(data.ncon):
        contact = data.contact[i]
        if tool not in (contact.geom1, contact.geom2):
            continue
        wrench = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, wrench)
        force = contact.frame.reshape(3, 3).T @ wrench[:3]
        total += force if contact.geom2 == tool else -force
    return total


def run(robot="ur5e", scenario="combined", controller="adaptive", duration=13.0,
        payload_mass=1.5, torque_scale=1.0, output=None, video=False, viewer=False,
        force_input=None):
    loaded = scenario in ("payload", "combined")
    wall = scenario in ("wall", "combined")
    mass = payload_mass if loaded else 0.0
    plant, state, home, limits, origin = make_model(robot, mass, wall, torque_scale)
    nominal, nominal_data, _, _, _ = make_model(robot, torque_scale=torque_scale)
    observed = mujoco.MjData(nominal)
    mpc = InteractionMPC(nominal, nominal_data, home, limits, controller == "adaptive")
    observer = ForceObserver(nominal.nv)
    dt = plant.opt.timestep
    stride = 5  # 500 Hz physics/observer, 100 Hz receding-horizon control.
    torque = np.zeros(plant.nu)
    records = {key: [] for key in ["time", "q", "velocity", "position", "reference",
               "torque", "force_estimate", "force_input", "force_truth", "contact_force",
               "margin", "tolerance", "slack", "relaxation", "observer_fit_error",
               "command_ms", "solver_ok"]}
    output = Path(output) if output else None
    if output:
        output.mkdir(parents=True, exist_ok=True)
    start = time.perf_counter()
    peak_contact = 0.0
    minimum_joint_clearance = np.inf
    with ExitStack() as stack:
        renderer = writer = live = None
        camera = mujoco.MjvCamera()
        camera.lookat[:] = (origin + [0, 0, 0.35]) / 2
        camera.distance, camera.azimuth, camera.elevation = 1.9, 135, -24
        if video:
            import imageio.v2 as imageio
            renderer = stack.enter_context(mujoco.Renderer(plant, height=720, width=960))
            writer = stack.enter_context(imageio.get_writer(str(output / "simulation.mp4"),
                                         fps=25, codec="libx264", quality=8))
        if viewer:
            import mujoco.viewer as mj_viewer
            live = stack.enter_context(mj_viewer.launch_passive(plant, state))
            live.cam.lookat[:], live.cam.distance = camera.lookat, camera.distance
            live.cam.azimuth, live.cam.elevation = camera.azimuth, camera.elevation
        for step in range(int(np.ceil(duration / dt))):
            t = step * dt
            observed.qpos[:], observed.qvel[:] = state.qpos, state.qvel
            mujoco.mj_forward(nominal, observed)
            M, h = dynamics(nominal, observed)
            position, _, jac, _ = kinematics(nominal, observed)
            old_velocity = state.qvel.copy()
            used_force = observer.force.copy() if force_input is None else np.asarray(force_input)
            if step % stride == 0:
                tick = time.perf_counter()
                torque, info = mpc.command(state.qpos, state.qvel, used_force,
                    trajectory(t + mpc.times, origin))
                command_ms = (time.perf_counter() - tick) * 1000
                ref = trajectory(t, origin)[0]
                records["time"].append(t)
                for key, value in {"q": state.qpos, "velocity": state.qvel,
                    "position": position, "reference": ref, "torque": torque,
                    "force_estimate": observer.force, "force_input": used_force,
                    "force_truth": np.zeros(3) if step == 0 else truth,
                    "contact_force": np.zeros(3) if step == 0 else contact,
                    "margin": 1 - np.abs(torque) / limits,
                    "tolerance": info["tolerance"], "slack": info["slack"],
                    "relaxation": info["relaxation"],
                    "observer_fit_error": observer.fit_error,
                    "command_ms": command_ms,
                    "solver_ok": info["status"] in ("solved", "solved inaccurate")}.items():
                    records[key].append(np.array(value).copy())
            state.ctrl[:] = torque
            state.mocap_pos[0] = trajectory(t, origin)[0]
            mujoco.mj_step(plant, state)
            if not np.all(np.isfinite(state.qpos)) or abs(state.time - (step+1)*dt) > dt/2:
                raise RuntimeError("Simulation became unstable or reset")
            observer.update(M, h, jac, old_velocity, state.qvel,
                            state.qfrc_actuator, dt, nominal.dof_damping)
            # Below is ground-truth evaluation, isolated from the controller.
            contact = contact_force(plant, state)
            peak_contact = max(peak_contact, float(np.linalg.norm(contact)))
            minimum_joint_clearance = min(minimum_joint_clearance,
                float(np.min(state.qpos - plant.jnt_range[:, 0])),
                float(np.min(plant.jnt_range[:, 1] - state.qpos)))
            mujoco.mj_forward(plant, state)
            new_jac = kinematics(plant, state)[2]
            ee_acceleration = (new_jac @ state.qvel - jac @ old_velocity) / dt
            truth = contact + mass * (plant.opt.gravity - ee_acceleration)
            if renderer and step % 20 == 0:
                renderer.update_scene(state, camera=camera)
                writer.append_data(renderer.render())
            if live and step % stride == 0:
                if not live.is_running():
                    break
                live.sync()
                time.sleep(max(0, state.time - (time.perf_counter() - start)))
    log = {key: np.asarray(value) for key, value in records.items()}
    tracking_error = np.linalg.norm(log["position"] - log["reference"], axis=1)
    settled = (log["time"] >= 5.5) & (log["time"] < 7.5)
    observer_valid = log["time"] >= 0.5
    summary = {
        "robot": robot, "scenario": scenario, "controller": controller,
        "duration_s": float(state.time), "payload_kg": mass,
        "torque_limits_nm": limits.tolist(), "force_input_n": force_input,
        "tracking_rmse_m": float(np.sqrt(np.mean(tracking_error**2))),
        "peak_contact_n": peak_contact,
        "minimum_joint_clearance_rad": minimum_joint_clearance,
        "minimum_torque_margin": float(log["margin"].min()),
        "settled_contact_n": float(np.mean(np.linalg.norm(log["contact_force"][settled], axis=1))) if settled.any() else None,
        "settled_min_joint_margin": float(np.mean(log["margin"][settled].min(axis=1))) if settled.any() else None,
        "force_rmse_n": float(np.sqrt(np.mean(np.sum((log["force_estimate"][observer_valid]
                           - log["force_truth"][observer_valid])**2, axis=1)))) if observer_valid.any() else None,
        "final_tracking_error_m": float(tracking_error[-1]),
        "solver_failures": mpc.failures,
        "command_median_ms": float(np.median(log["command_ms"])),
        "command_p95_ms": float(np.percentile(log["command_ms"], 95)),
        "wall_time_s": time.perf_counter() - start,
    }
    if output:
        np.savez_compressed(output / "trace.npz", **log)
        (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        plot_results(log, summary, output / "metrics.png")
    return log, summary


def plot_results(log, summary, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 2, figsize=(13, 10), sharex=True)
    t = log["time"]
    for i, label in enumerate("xyz"):
        color = f"C{i}"
        axes[0, 0].plot(t, log["position"][:, i], color=color, label=label)
        axes[0, 0].plot(t, log["reference"][:, i], "--", color=color, alpha=0.5)
        axes[1, 0].plot(t, log["force_estimate"][:, i], color=color, label=f"estimate {label}")
        axes[1, 0].plot(t, log["force_truth"][:, i], "--", color=color, alpha=0.5)
        axes[2, 0].plot(t, log["tolerance"][:, i]*1000, label=label)
    axes[0, 0].set_ylabel("EE position [m]; dashed = requested")
    axes[0, 1].plot(t, np.linalg.norm(log["position"]-log["reference"], axis=1)*1000)
    axes[0, 1].set_ylabel("Tracking error [mm]")
    axes[1, 0].set_ylabel("External force [N]; dashed = truth")
    axes[1, 1].plot(t, log["margin"])
    axes[1, 1].set_ylabel("Torque margin (each joint)")
    axes[1, 1].set_ylim(-0.02, 1.02)
    axes[2, 0].set_ylabel("Allowed reference slack [mm]")
    axes[2, 1].plot(t, np.linalg.norm(log["contact_force"], axis=1))
    axes[2, 1].set_ylabel("Tool contact force [N]")
    axes[2, 1].set_title(f"500 Hz physics peak: {summary['peak_contact_n']:.1f} N")
    for ax in axes.flat:
        ax.grid(alpha=0.25)
        ax.set_xlabel("Time [s]")
    for ax in axes[:, 0]:
        ax.legend(loc="best")
    fig.suptitle(f"{summary['robot']} / {summary['scenario']} / {summary['controller']}")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", choices=[*ROBOTS, "both"], default="both")
    parser.add_argument("--scenario", choices=["free", "payload", "wall", "combined", "all"], default="combined")
    parser.add_argument("--controller", choices=["adaptive", "fixed", "compare"], default="adaptive")
    parser.add_argument("--duration", type=float, default=13.0)
    parser.add_argument("--payload-mass", type=float, default=1.5)
    parser.add_argument("--torque-scale", type=float, default=1.0)
    parser.add_argument("--force-input", type=float, nargs=3, metavar=("FX", "FY", "FZ"),
                        help="Optional given world-frame EE force [N], instead of observer feedback")
    parser.add_argument("--output", type=Path, default=Path("outputs/interaction_mpc"))
    parser.add_argument("--video", action="store_true", help="Write simulation.mp4 with EGL on a headless server")
    parser.add_argument("--viewer", action="store_true", help="Open MuJoCo's interactive desktop viewer")
    args = parser.parse_args()
    if not np.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration must be positive and finite")
    if not np.isfinite(args.payload_mass) or args.payload_mass < 0:
        parser.error("--payload-mass must be nonnegative and finite")
    if not np.isfinite(args.torque_scale) or not 0 < args.torque_scale <= 1:
        parser.error("--torque-scale must be in (0, 1]")
    if args.force_input is not None and not np.all(np.isfinite(args.force_input)):
        parser.error("--force-input must contain finite numbers")
    robots = list(ROBOTS) if args.robot == "both" else [args.robot]
    scenarios = ["free", "payload", "wall", "combined"] if args.scenario == "all" else [args.scenario]
    controllers = ["fixed", "adaptive"] if args.controller == "compare" else [args.controller]
    summaries = []
    for robot in robots:
        for scenario in scenarios:
            for controller in controllers:
                output = args.output / f"{robot}_{scenario}_{controller}"
                print(f"Running {output.name} ...", flush=True)
                _, summary = run(robot, scenario, controller, args.duration, args.payload_mass,
                                 args.torque_scale, output, args.video, args.viewer, args.force_input)
                summaries.append(summary)
                print(json.dumps(summary, indent=2), flush=True)
    (args.output / "comparison.json").write_text(json.dumps(summaries, indent=2) + "\n")


if __name__ == "__main__":
    main()
