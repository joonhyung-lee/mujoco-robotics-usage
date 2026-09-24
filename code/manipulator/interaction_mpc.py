"""Shared force-observer and torque-margin MPC for the UR5e and Franka Panda.

Run demo_interaction_mpc.py. All Cartesian quantities use the world frame;
external force is environment-on-robot, applied at mpc_ee, in newtons.
"""

from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import osqp
from scipy import sparse


ROOT = Path(__file__).resolve().parents[2]
ROBOTS = {
    "ur5e": ("asset/ur5e/ur5e.xml", [-1.57, -1.3, 1.7, -1.97, -1.57, 0],
             [80, 80, 60, 12, 12, 12]),
    "franka": ("asset/panda/panda_nohand.xml", [0, -0.4, 0, -2.0, 0, 1.6, 0.785],
               [87, 87, 87, 87, 12, 12, 12]),
}


def robot_xml(robot, torque_scale=1.0):
    """Existing arm assets with torque motors; return an editable in-memory XML."""
    path, home, limits = ROBOTS[robot]
    path = ROOT / path
    root = ET.parse(path).getroot()
    compiler = root.find("compiler")
    compiler.set("meshdir", str(path.parent / compiler.get("meshdir")))
    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.attrib.update(timestep="0.002", integrator="Euler", gravity="0 0 -9.81")
    # Position-servo damping (UR: 200 Nm s/rad) is unsuitable for torque control.
    for joint in root.findall(".//joint"):
        if "damping" in joint.attrib:
            joint.set("damping", "1")
    actuators = root.find("actuator")
    joints = [a.get("joint") for a in actuators]
    actuators.clear()
    limits = np.asarray(limits, dtype=float) * torque_scale
    for joint, limit in zip(joints, limits):
        ET.SubElement(actuators, "motor", name=f"torque_{joint}", joint=joint,
                      gear="1", ctrllimited="true", ctrlrange=f"{-limit} {limit}",
                      forcelimited="true", forcerange=f"{-limit} {limit}")
    keyframe = root.find("keyframe")
    if keyframe is not None:
        root.remove(keyframe)
    return root, np.asarray(home), limits


def make_model(robot, payload_mass=0.0, wall=False, torque_scale=1.0):
    """Unloaded nominal arm or plant with a rigid point payload and optional wall."""
    root, home, limits = robot_xml(robot, torque_scale)
    parent = next(b for b in root.findall(".//body")
                  if b.find("site[@name='attachment_site']") is not None)
    site = parent.find("site[@name='attachment_site']")
    tool = ET.SubElement(parent, "body", name="mpc_tool", pos=site.get("pos", "0 0 0"))
    ET.SubElement(tool, "inertial", pos="0 0 0", mass=str(0.08 + payload_mass),
                  diaginertia="0.0001 0.0001 0.0001")
    ET.SubElement(tool, "geom", name="tool_geom", type="sphere", size="0.035",
                  contype="2", conaffinity="4", rgba="0.15 0.8 0.45 1",
                  friction="0.05 0.005 0.0001", solref="0.012 1")
    ET.SubElement(tool, "site", name="mpc_ee", size="0.007", rgba="1 1 0 1")
    world = root.find("worldbody")
    ET.SubElement(world, "geom", name="ground", type="plane", size="2 2 0.1",
                  rgba="0.17 0.20 0.25 1", contype="1", conaffinity="1")
    ET.SubElement(world, "geom", name="wall", type="box", size="0.025 0.20 0.22",
                  pos="2 0 0.6", contype="4" if wall else "0", conaffinity="2" if wall else "0",
                  rgba="0.4 0.65 0.9 0.45" if wall else "0 0 0 0",
                  friction="0.05 0.005 0.0001", solref="0.012 1")
    # Reference is a visual-only marker, never a source of contact forces.
    target = ET.SubElement(world, "body", name="reference", mocap="true", pos="0 0 1")
    ET.SubElement(target, "geom", type="sphere", size="0.014", rgba="1 0.35 0.15 0.8",
                  contype="0", conaffinity="0")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="960", offheight="720")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    data.qpos[:] = home
    mujoco.mj_forward(model, data)
    origin = data.site("mpc_ee").xpos.copy()
    model.geom_pos[model.geom("wall").id] = origin + [0.115, 0, 0]
    mujoco.mj_forward(model, data)
    return model, data, np.asarray(home), limits, origin


def kinematics(model, data):
    site = model.site("mpc_ee").id
    jp, jr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jp, jr, site)
    return data.site_xpos[site].copy(), data.site_xmat[site].reshape(3, 3).copy(), jp, jr


def dynamics(model, data):
    mass = np.empty((model.nv, model.nv))
    mujoco.mj_fullM(model, mass, data.qM)
    return mass, data.qfrc_bias.copy() - data.qfrc_passive


class ForceObserver:
    """Inverse-dynamics residual from encoder velocity differences and motor torque.

    No contact forces, applied force, plant mass, or simulated acceleration is
    read here. Motor torque is assumed measurable (or ideal commanded torque).
    Use wrench_dim=6 and stack translational/rotational Jacobians for a wrench.
    A 3D point force cannot identify arbitrary EE moments or multiple contacts.
    """

    def __init__(self, nv, bandwidth=25.0, damping=0.015, wrench_dim=3):
        self.residual = np.zeros(nv)
        self.force = np.zeros(wrench_dim)
        self.bandwidth, self.damping = bandwidth, damping
        self.fit_error = 0.0

    def update(self, mass, bias, jac, old_velocity, velocity, motor_torque, dt,
               joint_damping):
        residual = mass @ ((velocity - old_velocity) / dt) + bias - motor_torque
        # MuJoCo Euler integrates joint damping implicitly.
        residual += joint_damping * (velocity - old_velocity)
        alpha = -np.expm1(-self.bandwidth * dt)
        self.residual += alpha * (residual - self.residual)
        self.force = np.linalg.solve(jac @ jac.T + self.damping**2 * np.eye(jac.shape[0]),
                                     jac @ self.residual)
        self.fit_error = float(np.linalg.norm(jac.T @ self.force - self.residual))
        return self.force.copy()


class InteractionMPC:
    """Receding-horizon QP, locally frozen rigid-body dynamics, OSQP solver.

    Variables: joint accelerations, Cartesian reference slack, maximum normalized
    joint torque at each horizon step. Dynamics: M a + h = tau + J.T f_hat.
    The SAME optimizer and force convention serve payload and contact scenarios.
    """

    def __init__(self, model, data, home, limits, adaptive=True, horizon=10, dt=0.04,
                 control_dt=0.01, orientation_weight=120):
        self.model, self.data = model, data
        self.home, self.limits, self.adaptive = home, limits, adaptive
        self.horizon, self.dt, self.n = horizon, dt, model.nv
        self.rotation = kinematics(model, data)[1]
        self.orientation_weight = orientation_weight
        n, N = self.n, horizon
        rows, cols = np.indices((N, N))
        self.Bq = np.kron(np.maximum(rows - cols + 0.5, 0) * dt**2, np.eye(n))
        self.Bv = np.kron(np.tril(np.ones((N, N))) * dt, np.eye(n))
        self.times = np.arange(1, N + 1) * dt
        self.control_dt = control_dt
        self.compliance_offset = np.zeros(3)
        self.previous_solution = None
        self.failures = 0
        self.last_status = "not run"

    def command(self, q, velocity, force, reference, external_torque=None):
        q, velocity = np.asarray(q, dtype=float), np.asarray(velocity, dtype=float)
        if q.shape != (self.n,) or velocity.shape != (self.n,) or not (
                np.all(np.isfinite(q)) and np.all(np.isfinite(velocity))):
            raise ValueError(f"q and velocity must be finite ({self.n},) arrays")
        force = np.asarray(force, dtype=float)
        reference = np.asarray(reference, dtype=float)
        if force.shape != (3,) or not np.all(np.isfinite(force)):
            raise ValueError("force must be a finite world-frame 3-vector [N]")
        N, n = self.horizon, self.n
        if reference.shape != (N, 3) or not np.all(np.isfinite(reference)):
            raise ValueError(f"reference must be a finite ({N}, 3) array [m]")
        if external_torque is not None:
            external_torque = np.asarray(external_torque, dtype=float)
            if external_torque.shape != (n,) or not np.all(np.isfinite(external_torque)):
                raise ValueError(f"external_torque must be a finite ({n},) array [Nm]")
        d, m = self.data, self.model
        d.qpos[:], d.qvel[:] = q, velocity
        mujoco.mj_forward(m, d)
        position, rotation, jac, jr = kinematics(m, d)
        mass, bias = dynamics(m, d)
        disturbance = jac.T @ force if external_torque is None else external_torque
        offset = bias - disturbance
        load = np.max(np.abs(disturbance) / self.limits)
        # Smooth directional compliance tied to consumed torque capacity.
        relaxation = load**4 / (load**4 + 0.25**4) if self.adaptive else 0.0
        direction = force / max(np.linalg.norm(force), 1e-8)
        tolerance = 0.003 + 0.20 * relaxation * np.abs(direction)
        slack_weight = 20000 * np.eye(3)
        # A compliant equilibrium is essential: a constant-force predictor alone
        # otherwise keeps cancelling contact force even after slack is relaxed.
        equilibrium = np.clip(relaxation * force / 150, -tolerance, tolerance)
        # First-order admittance: 15*s_dot + 150*s = relaxation*f.
        # Smooth withdrawal too; instantly shrinking the bound can cause re-impact.
        self.compliance_offset += -np.expm1(-self.control_dt / 0.1) * (equilibrium - self.compliance_offset)
        tolerance = np.maximum(tolerance, np.abs(self.compliance_offset) + 0.003)
        preferred_slack = self.compliance_offset
        na, ns, nz = N*n, N*3, N*(n+4)
        P, linear = np.eye(nz) * 1e-7, np.zeros(nz)

        def least_squares(A, error, weight):
            nonlocal P, linear
            P += 2 * weight * (A.T @ A)
            linear += 2 * weight * (A.T @ error)

        q_free = q + self.times[:, None] * velocity
        dq_free = (q_free - q).reshape(-1)
        cart = np.kron(np.eye(N), jac) @ self.Bq
        tracking = np.zeros((ns, nz))
        tracking[:, :na], tracking[:, na:na+ns] = cart, -np.eye(ns)
        free_error = (position + (q_free - q) @ jac.T - reference).reshape(-1)
        least_squares(tracking, free_error, 60000)
        P[na:na+ns, na:na+ns] += 2 * np.kron(np.eye(N), slack_weight)
        linear[na:na+ns] -= 2 * np.tile(slack_weight @ preferred_slack, N)
        # Hold initial EE orientation while allowing redundant posture changes.
        orientation_error = 0.5 * sum(np.cross(self.rotation[:, i], rotation[:, i])
                                      for i in range(3))
        angular = np.zeros((3*N, nz))
        angular[:, :na] = np.kron(np.eye(N), jr) @ self.Bq
        least_squares(angular, np.tile(orientation_error, N)
                      + np.kron(np.eye(N), jr) @ dq_free, self.orientation_weight)
        vel_cost = np.zeros((na, nz))
        vel_cost[:, :na] = self.Bv
        least_squares(vel_cost, np.tile(velocity, N), 15)
        posture = np.zeros((na, nz))
        posture[:, :na] = self.Bq
        least_squares(posture, (q_free - self.home).reshape(-1), 0.4)
        P[:na, :na] += np.eye(na) * 0.5
        # Explicit infinity-norm torque cost: rho >= |tau_i / limit_i|.
        torque = np.zeros((na, nz))
        torque[:, :na] = np.kron(np.eye(N), mass / self.limits[:, None])
        normalized_bias = np.tile(offset / self.limits, N)
        least_squares(torque, normalized_bias, 3)
        P[na+ns:, na+ns:] += np.eye(N) * 50
        rho = np.zeros_like(torque)
        rho[:, na+ns:] = np.kron(np.eye(N), np.ones((n, 1)))
        constraints, lower, upper = [], [], []

        def bound(A, lo, hi):
            constraints.append(A)
            lower.append(np.broadcast_to(lo, A.shape[0]))
            upper.append(np.broadcast_to(hi, A.shape[0]))

        bound(torque - rho, -np.inf, -normalized_bias)
        bound(-torque - rho, -np.inf, normalized_bias)
        bound(np.eye(nz), np.r_[np.full(na, -25), -np.tile(tolerance, N), np.zeros(N)],
              np.r_[np.full(na, 25), np.tile(tolerance, N), np.ones(N)])
        bound(vel_cost, np.tile(-1.5 - velocity, N), np.tile(1.5 - velocity, N))
        bound(posture, (m.jnt_range[:, 0] + 0.03 - q_free).reshape(-1),
              (m.jnt_range[:, 1] - 0.03 - q_free).reshape(-1))
        # ponytail: rebuild a small dense QP; cache its sparsity if profiling calls for it.
        solver = osqp.OSQP()
        solver.setup(P=sparse.csc_matrix(np.triu(P)), q=linear,
                     A=sparse.csc_matrix(np.vstack(constraints)),
                     l=np.concatenate(lower), u=np.concatenate(upper), verbose=False,
                     eps_abs=1e-4, eps_rel=1e-4, max_iter=6000, polishing=False)
        if self.previous_solution is not None:
            solver.warm_start(x=self.previous_solution)
        result = solver.solve(raise_error=False)
        self.last_status = result.info.status
        if result.info.status_val in (1, 2) and np.all(np.isfinite(result.x)):
            self.previous_solution = result.x
            acceleration = result.x[:n]
            command = mass @ acceleration + offset
            slack = result.x[na:na+3]
        else:
            self.failures += 1
            self.previous_solution = None
            # Bounded gravity compensation plus joint damping; do not chase a failed QP.
            command = bias - 8 * velocity
            slack = np.zeros(3)
        return np.clip(command, -self.limits, self.limits), {
            "tolerance": tolerance, "slack": slack, "load": load,
            "relaxation": relaxation, "status": self.last_status,
        }
