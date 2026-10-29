import unittest
from pathlib import Path
import numpy as np
from clothforge.repair import resolve_config
from clothforge.schema import read_json
from clothforge.diagnostics import diagnose_trajectory


class RepairContracts(unittest.TestCase):
    def setUp(self):
        self.original = {"simulation": {"substeps": 8, "iterations": 10},
                         "material": {"tri_ke": 3000}, "gates": {"maximum_principal_stretch": 1.15}, "mesh_sha256": "frozen"}

    def test_material_mesh_and_gates_are_preserved(self):
        cfg = resolve_config(self.original, {"id": "test", "overrides": {"substeps": 32}})
        self.assertEqual(cfg["material"], self.original["material"])
        self.assertEqual(cfg["gates"], self.original["gates"])
        self.assertEqual(cfg["mesh_sha256"], self.original["mesh_sha256"])
        self.assertEqual(self.original["simulation"], {"substeps": 8, "iterations": 10})

    def test_cannot_override_physics_target(self):
        for key in ["material", "gates", "tri_ke", "particle_radius_m", "duration_s", "mesh_sha256"]:
            with self.assertRaises(ValueError):
                resolve_config(self.original, {"id": "bad", "overrides": {key: 1}})

    def test_invalid_numerics_rejected(self):
        for value in [0, -1, 1.5, True, float("nan"), float("inf")]:
            with self.assertRaises(ValueError):
                resolve_config(self.original, {"id": "bad", "overrides": {"substeps": value}})

    def test_plan_has_disjoint_confirmation_and_holdout(self):
        plan = read_json(Path(__file__).parents[1] / "configs/repair_v2.json")
        self.assertFalse(set(plan["confirm_variants"]) & set(plan["holdout_variants"]))
        for profile in plan["profiles"]:
            resolve_config(self.original, profile)

    def test_freefall_reference_detects_slow_motion(self):
        rest = np.array([[[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]]])
        faces = np.array([[0, 1, 2]])
        times = np.arange(7) / 30
        dt = 1 / 240
        steps = np.rint(times / dt)
        dz = -.5 * 9.81 * dt**2 * steps * (steps + 1)
        q = np.repeat(rest, 7, axis=0) + [0, 0, 1]
        q[:, :, 2] += dz[:, None]
        exact = diagnose_trajectory(q, np.zeros_like(q), faces, rest, [], times, .22, dt, self.original["gates"], "drop")
        self.assertLess(exact["freefall"]["maximum_com_error_m"], 1e-10)
        q[:, :, 2] -= .5 * dz[:, None]
        slow = diagnose_trajectory(q, np.zeros_like(q), faces, rest, [], times, .22, dt, self.original["gates"], "drop")
        self.assertGreater(slow["freefall"]["maximum_com_error_m"], .09)
