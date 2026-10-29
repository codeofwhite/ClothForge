# Backend setup and interpretation

The adapters preserve the existing research pipeline. They are not a universal runtime installer. Only the static viewer is independent of installed solvers.

| Path | Historical environment | Public configuration |
|---|---|---|
| Newton VBD / XPBD | Newton 1.4.0, Warp 1.16.0; old/internal APIs are used | Set `CUDA_VISIBLE_DEVICES` explicitly; worker uses logical `cuda:0` |
| Genesis IPC | Existing Genesis / pyuipc integration | Run from the compatible Genesis environment; compare initialization metadata |
| PhysX Surface | Isaac Sim 6.0.1 | Set `CLOTHFORGE_ISAAC_PYTHON` to the absolute `python.sh` launcher; include this source on `PYTHONPATH` |
| PPF | Official local image with historical image ID and manifest digest recorded in `ppf_worker.py` | Install Docker with the NVIDIA runtime and obtain/build the matching upstream image separately |

These versions describe earlier artifacts and development notes. They are not a claim that the current upstream releases expose the same APIs. No solver installs or GPU runs were performed for public packaging.

## Material semantics

`comparison.ipc_material` maps nominal membrane Lamé scales and numeric bending presets into IPC. It explicitly leaves some damping parameters unmapped. PhysX maps 2-D membrane scales to 3-D material units using a nominal thickness; XPBD uses mesh-edge springs, bending constraints and contact extensions; PPF has a separate mapping and strain barrier. These are **not calibrated equivalent constitutive models**.

The display deliberately shows resolved simulation settings and full snapshots. Material sliders that imply a universal equivalent stiffness would be misleading without calibration; this preview therefore selects existing cases rather than offering an editable live-physics slider.

## Output contract

- `request.json`: source configuration/profile and worker hashes.
- `initialization.json`: actual solver setup, fixtures, material mapping and native versions when available.
- `trajectory.npz`: positions, velocities, topology, timestamps and reference information.
- `validation.json`: physical checks and failed gates.
- `diagnostics.json`: complementary failure analysis.
- `replay.html` / `preview.png` / `final.glb`: derived visualizations.

The public case bundle includes the original trajectory and validation plus the configuration/initialization snapshot. Physical GPU allocation is omitted from public snapshots. The website uses rounded position JSON for display; scores are from the original saved trajectory, not recomputed from display JSON.

## Optional integrations

`export-studio` expects an external compatible Robot Data Studio registry. `export-lehome` expects Isaac Sim and a user-supplied LeHome baseline. `convert-garmentcodedata` expects a separately obtained upstream dataset. These integrations remain in the source for continuity but are outside the standalone demo path; their private inputs and applications are not redistributed.
