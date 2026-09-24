"""Behavioral regression checks; run with the repository's .venv Python."""

import os
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
import unittest

import mujoco
import numpy as np

from interaction_mpc import ForceObserver, InteractionMPC, ROBOTS, dynamics, kinematics, make_model
from demo_interaction_mpc import run


class InteractionTests(unittest.TestCase):
    def test_force_observer_sign_and_frame(self):
        for robot in ROBOTS:
            for force in [np.array([8., -4., 6.]), np.array([-5., 7., -9.])]:
                with self.subTest(robot=robot, force=force):
                    model, data, _, _, _ = make_model(robot)
                    observer = ForceObserver(model.nv)
                    for _ in range(250):
                        mujoco.mj_forward(model, data)
                        mass, bias = dynamics(model, data)
                        jac = kinematics(model, data)[2]
                        velocity = data.qvel.copy()
                        # World-frame point force at EE; compensate to hold still.
                        data.qfrc_applied[:] = jac.T @ force
                        data.ctrl[:] = bias - jac.T @ force
                        mujoco.mj_step(model, data)
                        observer.update(mass, bias, jac, velocity, data.qvel,
                                        data.qfrc_actuator, model.opt.timestep, model.dof_damping)
                    np.testing.assert_allclose(observer.force, force, atol=0.04)
                    self.assertLess(observer.fit_error, 0.02)

    def test_free_payload_and_contact(self):
        for robot in ROBOTS:
            for scenario in ("free", "payload", "wall", "combined"):
                with self.subTest(robot=robot, scenario=scenario):
                    log, summary = run(robot, scenario)
                    self.assertEqual(summary["solver_failures"], 0)
                    self.assertGreaterEqual(summary["minimum_torque_margin"], -1e-8)
                    self.assertGreater(summary["minimum_joint_clearance_rad"], 0)
                    self.assertLess(summary["final_tracking_error_m"], 0.012)
                    for key in log:
                        self.assertTrue(np.all(np.isfinite(log[key])), key)
                    if scenario in ("free", "payload"):
                        self.assertLess(summary["tracking_rmse_m"], 0.012)
                        self.assertEqual(summary["peak_contact_n"], 0)
                        self.assertLess(summary["force_rmse_n"], 0.5)
                    else:
                        self.assertGreater(summary["settled_contact_n"], 1)
                        self.assertLess(summary["settled_contact_n"], 45)
                        self.assertLess(summary["peak_contact_n"], 100)
                        self.assertGreater(summary["settled_min_joint_margin"], 0.35)
                        self.assertGreater(log["tolerance"][:, 0].max(), 0.03)
                    print(robot, scenario, "passed", flush=True)

    def test_given_force_and_validation(self):
        for robot in ROBOTS:
            model, data, home, limits, origin = make_model(robot)
            mpc = InteractionMPC(model, data, home, limits)
            force = np.array([0, 0, -14.715])
            reference = np.tile(origin, (mpc.horizon, 1))
            torque, info = mpc.command(home, np.zeros(model.nv), force, reference)
            self.assertTrue(np.all(np.abs(torque) <= limits))
            self.assertGreater(info["load"], 0)
            with self.assertRaises(ValueError):
                mpc.command(home, np.zeros(model.nv), [np.nan, 0, 0], reference)
            with self.assertRaises(ValueError):
                mpc.command(home, np.zeros(model.nv), [0, 0], reference)
            # An impossible force must trigger the bounded fallback, not send
            # an infeasible solver iterate to the motor.
            torque, _ = mpc.command(home, np.zeros(model.nv), [1e4]*3, reference)
            self.assertEqual(mpc.failures, 1)
            self.assertTrue(np.all(np.isfinite(torque)))
            self.assertTrue(np.all(np.abs(torque) <= limits))


if __name__ == "__main__":
    unittest.main()
