# ClothForge

**See how cloth behaves across engines and solvers.**

ClothForge is an open-source cloth simulation playground: parametric garment geometry, explicit solver adapters, inspectable parameter snapshots, sampled-frame physical checks, and a browser comparison viewer.

**Public preview v0.1:** the website replays **10 recorded simulations** of one synthetic T-shirt across five backends and two scenes. It runs without GPU access, a backend server, external JavaScript libraries, or a Robot Data Studio installation. It does **not** recompute physics when you change a selection.

[Explore the preview locally](#try-the-browser-preview) · [Run locally](#run-locally) · [Backend notes](docs/BACKENDS.md) · [Release evidence](docs/RELEASE.md)

## Try the browser preview

```bash
git clone https://github.com/codeofwhite/ClothForge.git
cd ClothForge
python3 -m http.server 8080 --bind 127.0.0.1 --directory docs
```

Open <http://127.0.0.1:8080>. Use an HTTP server instead of opening `index.html` directly: the viewer fetches local JSON files.

- Compare any two of five solver backends on **drop** or **two-point hanging** scenes.
- Share camera orientation, scale and simulation time across both views.
- Play, pause, scrub, orbit and zoom the saved mesh trajectories.
- Inspect the material parameters, numerical settings, validation outcome and failure gates.
- Download original NPZ trajectories, the source mesh, and resolved configuration snapshots.

To publish this folder with GitHub Pages, choose **Settings → Pages → Deploy from a branch → main → /docs**. The repository contains the website; Pages hosting must be enabled separately. Do not assume the default Pages URL is live before deployment succeeds.

## What is included

```text
src/clothforge/         Geometry, material mapping, workers, checks and exporters
configs/               Parameterized garment and controlled numerical-repair recipes
tests/                 Existing physical-invariant and adapter contract checks
docs/index.html        Static comparison viewer; no build step
docs/assets/catalog.json   Recorded case index, metrics and provenance
docs/assets/cases/     Original trajectories, display derivatives and snapshots
docs/assets/garment/   Shared T-shirt mesh and manifest
```

The original development workbench is integrated into Robot Data Studio. That application, its database, robot control services, and its interactive job scheduler are **not included** in this standalone preview. This repository exposes the existing Python pipeline and an independent static viewer.

## Backend support

| Backend ID | Engine / solver path | Included preview | Important limit |
|---|---|---|---|
| `newton_vbd` | Newton / VBD | Hanging + drop | Recorded cases passed sampled-frame checks |
| `genesis_ipc` | Genesis + libuipc / IPC FEM | Hanging + drop | Nominal material mapping, not calibrated equivalence |
| `newton_xpbd` | Newton / XPBD + ClothForge surface-contact extensions | Hanging + drop | Discrete projections; not certified CCD |
| `physx_surface` | Isaac Sim / PhysX Surface Deformable | Hanging + drop | Both selected cases failed checks; failures remain visible |
| `ppf` | PPF contact solver / isolated Docker runtime | Hanging + drop | Offline path; historical image ID is not a downloadable registry address |

This table describes the selected historical cases, not universal solver capabilities or a fresh runtime validation. Adapter source and recorded cases are included for all five paths. Runtime installation and compatibility remain backend-specific.

## Run locally

Python 3.11+ is required for the package. Backends use **separate environments**: do not install Isaac Sim, Genesis, Newton and all generation dependencies into one environment by default. No simulation runs automatically during installation or preview.

For a minimal editable package in your own virtual environment:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[preview]'
.venv/bin/python -m clothforge --help
```

This installs common analysis/preview dependencies, **not the GPU solvers or garment generator**. See [backend notes](docs/BACKENDS.md) for historical versions and requirements. Dependencies were reused from existing environments for release verification; a clean install was not tested.

### Reuse the included garment

A case folder contains `config-medium.json` with the actual resolved material, numerical settings and quality gates. Copy it next to the included mesh and manifest before a run. Example for the default VBD drop case:

```bash
mkdir -p outputs/tshirt-input
cp docs/assets/garment/mesh.npz docs/assets/garment/manifest.json outputs/tshirt-input/
cp docs/assets/cases/newton_vbd-drop/config-medium.json outputs/tshirt-input/

# Run in a compatible Newton environment with this package available.
# Replace 0 with an idle physical GPU you have selected and authorized.
CUDA_VISIBLE_DEVICES=0 PYTHONPATH=src \
  /path/to/newton-env/bin/python -B -m clothforge simulate \
  --garment outputs/tshirt-input --material medium --scenario drop \
  --engine newton_vbd --output outputs/my-vbd-drop
```

The output directory must be new. The run saves a request, initialization metadata, trajectory, diagnostics, physical verdict and replay. Historical hashes identify old artifacts; this public source snapshot is newer and does not promise bitwise reproduction of old trajectories.

For another engine, copy **that case's** configuration and select its corresponding `--engine`. Numerical settings and energy/contact models differ. In particular, XPBD needs the recorded `xpbd_surface_contact` settings; PPF needs a separately obtained compatible image.

### Generate new garments (optional)

The source supports T-shirts, trousers, coats and several simpler textile/garment variants. The public viewer currently includes only one T-shirt.

Generation requires GarmentCode, its dependencies and CGAL Python bindings, in addition to the common preview libraries. The original generation environment used NumPy 1.26.4. Follow the upstream installation instructions for the pinned revision rather than mixing generator and solver environments.

```bash
mkdir -p third_party
git clone https://github.com/maria-korosteleva/GarmentCode.git third_party/GarmentCode
git -C third_party/GarmentCode checkout d449629979028123a5c4dc9e732a2ec19b7fce31

# After configuring a compatible generator environment:
PYTHONPATH=src:third_party/GarmentCode \
  /path/to/generation-env/bin/python -B -m clothforge generate \
  --config configs/pilot_v1.json --output outputs/generated --limit 1
```

`--limit 1` creates at most one garment per category. `generate` only creates geometry/material configurations; it does not certify dynamics. `configs/pilot_v1.json` uses public starter numerical defaults adapted from the recorded VBD recipe. It is not the unchanged historical pilot specification, and those settings are not validated for every new garment.

## Parameters and evidence boundaries

- **Common inputs:** geometry, SI units, areal density, gravity, initial pose and fixture definition.
- **Nominal material controls:** membrane/bending values and friction are mapped into each backend. Identical numbers do not establish identical material response. Damping and contact laws may remain unmapped.
- **Numerical controls:** time step, substeps, iterations and contact settings remain explicit. Some settings also affect effective physical response.
- **Checks:** stretch, area, sampled-frame self-intersection, ground penetration and finite state; the PhysX path additionally checks early freefall motion when configured.

The included material is labelled `synthetic_unmeasured`. “Passed” means the historical **61 saved frames over two seconds** passed the recorded gates. It does not establish continuous nonpenetration, long-horizon stability, real-material calibration, manipulation success, or sim-to-real validity.

Historical solver time is shown for traceability only. Backend settings, startup costs and implementations differ; this is **not an equal-compute speed ranking**. Release verification checks the saved data and code contracts; it does not rerun the GPU simulations. Review status: `SELF_REVIEWED_PROVISIONAL`.

## Development checks

In an existing compatible generation environment (GarmentCode + NumPy/SciPy + CGAL and preview dependencies):

```bash
PYTHONPATH=src:third_party/GarmentCode PYTHONDONTWRITEBYTECODE=1 \
  /path/to/generation-env/bin/python -B -m unittest discover -s tests -p 'test_pipeline.py'
```

Other CPU contract suites are `test_comparison.py`, `test_native_io.py`, `test_physx_material.py` and `test_repair.py`. `test_xpbd_contact.py` requires its compatible Newton/Warp runtime; it is not part of CPU-only verification.

## Roadmap

- [x] Independent public source snapshot and static trajectory comparison.
- [x] Actual configuration downloads and visible failed cases.
- [ ] Parameter-sweep navigation with more materials and garments.
- [ ] Standalone local run service and editable parameter interface.
- [ ] Clean-environment setup checks for each backend.
- [ ] Measured material calibration and controlled physical experiments.

中文说明：这是 ClothForge 的第一版公开预览。网页可同步对比已有的真实仿真轨迹，查看参数、失败原因并下载输入；自由修改参数后重新计算，需要安装对应的本地求解器。当前材料未经实物标定，也没有把仿真检查通过当作真实布料精度证明。

## License

ClothForge code and included synthetic demo artifacts: [MIT](LICENSE). See [third-party notices](THIRD_PARTY_NOTICES.md) for dependencies and upstream terms. Third-party engines, containers and GarmentCodeData are not bundled.
