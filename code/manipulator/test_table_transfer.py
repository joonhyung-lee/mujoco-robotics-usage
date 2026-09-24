"""Physical grasp, contact recovery, release and failure regression tests."""

import os
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
import unittest

import mujoco
import numpy as np

from demo_table_transfer import run_transfer
from interaction_mpc import ForceObserver, dynamics, kinematics
from table_transfer import GRIPPER_JOINTS, LIFT, TIP_GEOMS, make_transfer_model


class TransferTests(unittest.TestCase):
    def test_pick_transfer_place(self):
        for robot in ("ur5e", "franka"):
            for case in ("avoid", "contact"):
                with self.subTest(robot=robot, case=case):
                    log, result = run_transfer(robot, case)
                    self.assertTrue(result["success"], result["failure"])
                    self.assertTrue(result["grasp_verified"])
                    self.assertTrue(result["lift_verified"])
                    self.assertGreater(result["max_lift_height_m"], 0.8*LIFT)
                    self.assertLess(result["final_xy_error_m"], 0.01)
                    self.assertLess(result["final_bottom_error_m"], 0.002)
                    self.assertGreater(result["final_upright_cosine"], 0.99)
                    self.assertGreater(result["final_table_b_support_n"], 3)
                    self.assertLess(max(result["final_grip_normal_n"]), 0.1)
                    self.assertEqual(result["solver_failures"], 0)
                    self.assertGreater(result["minimum_torque_margin"], 0.2)
                    self.assertLess(result["peak_other_wall_force_n"], 1e-3)
                    self.assertTrue(np.all(np.isfinite(log["torque"])))
                    phases = [event["phase"] for event in result["events"]]
                    self.assertIn("release", phases)
                    if case == "avoid":
                        self.assertLess(result["peak_bar_wall_force_n"], 1e-3)
                        self.assertFalse(result["reaction_detected"])
                    else:
                        self.assertTrue(result["reaction_detected"])
                        self.assertGreater(result["peak_bar_wall_force_n"], 5)
                        self.assertLess(result["peak_bar_wall_force_n"], 40)
                        self.assertIn("yield", phases)
                        self.assertIn("backoff", phases)

    def test_gripper_cannot_fake_a_pick(self):
        for robot in ("ur5e", "franka"):
            with self.subTest(robot=robot):
                model, _, _, _ = make_transfer_model(robot)
                # The original RG2 has five joint couplers, all inside the hand.
                # None welds the free object or constrains its pose.
                joints = {model.joint(name).id for name in GRIPPER_JOINTS}
                self.assertEqual(model.neq, 5)
                for kind, first, second in zip(model.eq_type, model.eq_obj1id, model.eq_obj2id):
                    self.assertEqual(kind, mujoco.mjtEq.mjEQ_JOINT)
                    self.assertIn(first, joints)
                    self.assertIn(second, joints)
                for name in TIP_GEOMS:
                    self.assertEqual(model.geom_type[model.geom(name).id], mujoco.mjtGeom.mjGEOM_MESH)
                self.assertTrue(all(model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE for j in joints))
                _, result = run_transfer(robot, grip_torque=0, max_time=12)
                self.assertFalse(result["success"])
                self.assertFalse(result["lift_verified"])
                self.assertIn("grasp failed", result["failure"])

    def test_six_dimensional_observer(self):
        for robot in ("ur5e", "franka"):
            with self.subTest(robot=robot):
                m, d, _, _ = make_transfer_model(robot, scene=False)
                observer = ForceObserver(m.nv, damping=0.003, wrench_dim=6)
                expected = np.array([3., -4., -2., 0.2, -0.3, 0.1])
                for _ in range(300):
                    mujoco.mj_forward(m, d)
                    M, h = dynamics(m, d)
                    _, _, jp, jr = kinematics(m, d)
                    jac = np.vstack([jp, jr])
                    old_velocity = d.qvel.copy()
                    d.qfrc_applied[:] = jac.T @ expected
                    d.ctrl[:] = h-d.qfrc_applied
                    mujoco.mj_step(m, d)
                    observer.update(M, h, jac, old_velocity, d.qvel, d.qfrc_actuator,
                                    m.opt.timestep, m.dof_damping)
                np.testing.assert_allclose(observer.force, expected, atol=0.03)


if __name__ == "__main__":
    unittest.main()
