"""Repository RG2 gripper and upright bar transfer, shared by both arms."""

from copy import deepcopy
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from interaction_mpc import ROOT, kinematics, robot_xml


TABLE_Z = 0.18
BAR_HALF = np.array([0.018, 0.018, 0.15])
GRASP_OFFSET = 0.13  # leave the bar's top below the RG2 knuckles
TABLE_A = np.array([0.44, -0.18, TABLE_Z])
TABLE_B = np.array([0.44, 0.18, TABLE_Z])
WALL_CENTER = np.array([0.44, 0.0, 0.27])
WALL_HALF = np.array([0.055, 0.016, 0.27])
LIFT = 0.07
OPEN = 0.9  # RG2 driving-joint angle [rad]; increasing opens the fingers
CLOSED = 0.0
GRIPPER_JOINTS = [f"gripper_finger{i}_{part}" for i in (1, 2)
                  for part in ("joint", "inner_knuckle_joint", "finger_tip_joint")]
TIP_GEOMS = [f"ur_rg2_gripper_finger{i}_finger_tip_link_collision" for i in (1, 2)]


def rg2_configuration(angle):
    """Rest configuration satisfying the original five linkage equations."""
    return np.array([-angle, -angle/1.1, -angle, angle, angle/1.1, angle])


def numbers(values):
    return " ".join(str(float(x)) for x in values)


def make_transfer_model(robot, scene=True, mass=0.4, friction=1.2):
    """Arm + the original articulated RG2. The object is always a free rigid body.

    The RG2's mesh, inertia, joint and coupling definitions come from the repo.
    Its joint equalities only couple its fingers; nothing attaches the object.
    The nominal arm freezes the linkage at a representative grasp aperture.
    """
    root, home, limits = robot_xml(robot)
    if robot == "ur5e":
        home[0] += np.pi / 2  # face the tables in the +x workspace
    raw = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    raw_data = mujoco.MjData(raw)
    raw_data.qpos[:] = home
    mujoco.mj_forward(raw, raw_data)
    parent = next(b for b in root.findall(".//body")
                  if b.find("site[@name='attachment_site']") is not None)
    site = parent.find("site[@name='attachment_site']")
    parent_rotation = raw_data.body(parent.get("name")).xmat.reshape(3, 3)
    source_path = ROOT / "asset/ur5e/ur5e_rg2.xml"
    source = ET.parse(source_path).getroot()
    hand = deepcopy(source.find(".//body[@name='rg2_gripper_base_link']"))
    hand.remove(hand.find("body[@name='camera_center']"))
    hand.set("pos", site.get("pos", "0 0 0"))
    hand_rotation = parent_rotation.T @ np.diag([1., -1., -1.])
    hand.set("quat", numbers(np.roll(Rotation.from_matrix(hand_rotation).as_quat(), 1)))
    parent.append(hand)
    gripper_default = deepcopy(source.find("default/default[@class='GRIPPER']"))
    # Preserve the source's inherited rotor inertia. Lower viscous damping for
    # the added 2 Nm motor cap; the original 5 per hinge prevents timely closing.
    gripper_default.find("joint").attrib.update(armature="0.05", damping="0.2")
    root.find("default").append(gripper_default)
    for mesh in source.findall("asset/mesh"):
        if mesh.get("name", "").startswith("rg2_"):
            mesh = deepcopy(mesh)
            mesh.set("file", str(source_path.parent/"mesh"/mesh.get("file")))
            root.find("asset").append(mesh)
    for geom in hand.findall(".//geom"):
        geom.attrib.update(condim="6", friction=f"{friction} 0.015 0.0005",
                           solref="0.008 1", solimp="0.95 0.99 0.001")
    ET.SubElement(hand, "site", name="mpc_ee", pos="0 0.0013 0.205", quat="0 1 0 0",
                  size="0.004", rgba="0 1 0 1")
    root.append(deepcopy(source.find("contact")))
    if scene:
        equality = deepcopy(source.find("equality"))
        for constraint in equality:
            constraint.attrib.update(solref="0.004 1", solimp="0.999 0.9999 0.001")
        root.append(equality)
        actuator = deepcopy(source.find("actuator/position[@name='gripper']"))
        actuator.attrib.update(forcelimited="true", forcerange="-2 2")
        root.find("actuator").append(actuator)
    else:
        # Bake each hinge's relative transform, including its original axis sign.
        for body in hand.findall(".//body"):
            joint = body.find("joint")
            if joint is not None:
                angle = rg2_configuration(0.4)[GRIPPER_JOINTS.index(joint.get("name"))]
                quat = np.fromstring(body.get("quat", "1 0 0 0"), sep=" ")
                rotation = Rotation.from_quat(np.roll(quat, -1))
                axis = np.fromstring(joint.get("axis"), sep=" ")
                quat = np.roll((rotation*Rotation.from_rotvec(axis*angle)).as_quat(), 1)
                body.set("quat", numbers(quat))
                body.remove(joint)
    option = root.find("option")
    option.attrib.update(timestep="0.002", cone="elliptic", impratio="10", iterations="80")
    world = root.find("worldbody")
    if scene:
        ET.SubElement(world, "geom", name="floor", type="plane", size="2 2 0.1",
                      rgba="0.31 0.35 0.4 1")
        for name, position, color in [("A", TABLE_A, "0.2 0.45 0.7 1"),
                                       ("B", TABLE_B, "0.25 0.65 0.4 1")]:
            table = ET.SubElement(world, "body", name=f"table_{name}", pos=numbers(position))
            ET.SubElement(table, "geom", name=f"table_{name}_top", type="box",
                          pos="0 0 -0.025", size="0.115 0.10 0.025", rgba=color,
                          friction="0.8 0.005 0.0001")
            for x in [-0.085, 0.085]:
                for y in [-0.07, 0.07]:
                    ET.SubElement(table, "geom", type="box", size="0.012 0.012 0.075",
                                  pos=numbers([x, y, -0.105]), rgba="0.2 0.22 0.26 1")
        ET.SubElement(world, "geom", name="wall", type="box", pos=numbers(WALL_CENTER),
                      size=numbers(WALL_HALF), rgba="0.65 0.55 0.40 0.75",
                      friction="0.3 0.005 0.0001", solref="0.01 1")
        bar = ET.SubElement(world, "body", name="bar",
                            pos=numbers(TABLE_A + [0, 0, BAR_HALF[2] + 0.001]))
        ET.SubElement(bar, "freejoint", name="bar_free")
        ET.SubElement(bar, "geom", name="bar_geom", type="box", size=numbers(BAR_HALF),
                      mass=str(mass), rgba="0.94 0.48 0.10 1", friction=f"{friction} 0.015 0.0005",
                      condim="6", solref="0.008 1", solimp="0.95 0.99 0.001")
        marker = ET.SubElement(world, "body", name="reference", mocap="true")
        ET.SubElement(marker, "geom", type="sphere", size="0.008", rgba="1 0.2 0.2 0.5",
                      contype="0", conaffinity="0")
        ET.SubElement(world, "light", pos="0.3 -1 2", dir="0 0 -1", diffuse="0.7 0.7 0.7")
        visual = ET.SubElement(root, "visual")
        ET.SubElement(visual, "global", offwidth="960", offheight="720")
        ET.SubElement(visual, "headlight", ambient="0.45 0.45 0.45", diffuse="0.7 0.7 0.7")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    data.qpos[:len(home)] = home
    if scene:
        data.qpos[len(home):len(home)+6] = rg2_configuration(OPEN)
    mujoco.mj_forward(model, data)
    return model, data, home, limits


def inverse_kinematics(model, data, target, seed, rotation=np.eye(3)):
    """Damped least-squares IK for waypoint reachability and the initial pose."""
    data.qpos[:] = seed
    data.qvel[:] = 0
    for _ in range(600):
        mujoco.mj_forward(model, data)
        position, current, jp, jr = kinematics(model, data)
        error = np.r_[target-position, Rotation.from_matrix(rotation @ current.T).as_rotvec()]
        if np.linalg.norm(error[:3]) < 1e-4 and np.linalg.norm(error[3:]) < 2e-3:
            return data.qpos.copy()
        J = np.vstack([jp, jr])
        delta = J.T @ np.linalg.solve(J @ J.T + 1e-4*np.eye(6), error)
        delta *= min(1, 0.08 / max(np.max(np.abs(delta)), 1e-12))
        data.qpos[:] = np.clip(data.qpos + delta, model.jnt_range[:, 0]+0.04,
                              model.jnt_range[:, 1]-0.04)
    raise ValueError(f"Unreachable waypoint {target}; position error {np.linalg.norm(error[:3]):.3f} m")


def detour(start, goal, clearance=0.06):
    """Outboard route around the wall's +x end, inflated for gripper + bar.

    ponytail: one known box obstacle, use a scene planner for arbitrary obstacles.
    """
    x = WALL_CENTER[0] + WALL_HALF[0] + clearance
    y0 = WALL_CENTER[1] - WALL_HALF[1] - clearance
    y1 = WALL_CENTER[1] + WALL_HALF[1] + clearance
    return [np.asarray(start), np.array([x, y0, start[2]]),
            np.array([x, y1, start[2]]), np.asarray(goal)]


def contact_measurements(model, data):
    """Ground truth for grasp/placement checks and reporting; not the force observer."""
    bar, wall = model.geom("bar_geom").id, model.geom("wall").id
    pads = [model.geom(name).id for name in TIP_GEOMS]
    tables = [model.geom(f"table_{side}_top").id for side in ("A", "B")]
    grip, support = np.zeros(2), np.zeros(2)
    wall_force, other_wall = np.zeros(3), 0.0
    for i in range(data.ncon):
        c = data.contact[i]
        pair = (c.geom1, c.geom2)
        wrench = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, wrench)
        force = c.frame.reshape(3, 3).T @ wrench[:3]
        if bar in pair:
            force *= 1 if c.geom2 == bar else -1
            for k, pad in enumerate(pads):
                if pad in pair:
                    grip[k] += max(0, wrench[0])
            for k, table in enumerate(tables):
                if table in pair:
                    support[k] += force[2]
            if wall in pair:
                wall_force += force
        elif wall in pair:
            other_wall += np.linalg.norm(force)
    return grip, support, wall_force, other_wall
