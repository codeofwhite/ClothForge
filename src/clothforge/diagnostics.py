"""Supplemental physical diagnostics; these never override the frozen evaluator."""
import numpy as np
from .quality import rest_matrices


def diagnose_trajectory(positions, velocities, faces, rest, pins, times, density, dt, gates, scenario):
    q = np.asarray(positions)
    if not np.isfinite(q).all() or not np.isfinite(velocities).all():
        return {"available": False, "reason": "nonfinite state"}
    inv, areas = rest_matrices(rest)
    tri = q[:, faces]
    ds = np.stack((tri[:, :, 1] - tri[:, :, 0], tri[:, :, 2] - tri[:, :, 0]), axis=3)
    stretch = np.linalg.svd(ds @ inv, compute_uv=False)[:, :, 0]
    frame, triangle = np.unravel_index(stretch.argmax(), stretch.shape)
    failures = np.flatnonzero(stretch.max(axis=1) > gates["maximum_principal_stretch"])
    masses = np.bincount(faces.ravel(), weights=np.repeat(areas * density / 3, 3), minlength=q.shape[1])
    center = (q * masses[None, :, None]).sum(axis=1) / masses.sum()
    result = {"available": True, "peak_stretch_frame": int(frame), "peak_stretch_triangle": int(triangle),
              "peak_stretch_vertex_ids": faces[triangle].tolist(),
              "first_stretch_failure_frame": int(failures[0]) if len(failures) else None,
              "peak_stretch_touches_pin": bool(set(faces[triangle]) & set(pins)),
              "maximum_pin_drift_m": float(np.linalg.norm(q[:, pins] - q[0, pins], axis=2).max()) if len(pins) else 0.0,
              "center_of_mass": center.tolist()}
    if scenario == "drop":
        t = np.asarray(times)
        safe = (t <= 0.2 + 1e-8) & (q[:, :, 2].min(axis=1) > 0.02)
        steps = np.rint(t / dt)
        reference = -0.5 * 9.81 * dt * dt * steps * (steps + 1)
        errors = center[:, 2] - center[0, 2] - reference
        result["freefall"] = {"reference": "semi_implicit_gravity_no_drag_before_ground_contact",
                              "time_end_s": float(t[safe][-1]) if safe.any() else None,
                              "maximum_com_error_m": float(np.abs(errors[safe]).max()) if safe.any() else None,
                              "observed_displacement_m": float(center[safe][-1, 2] - center[0, 2]) if safe.any() else None,
                              "reference_displacement_m": float(reference[safe][-1]) if safe.any() else None}
    return result
