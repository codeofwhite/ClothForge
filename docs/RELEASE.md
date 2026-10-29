# Public preview v0.1 — release record

Prepared on 2026-10-29. Review status: `SELF_REVIEWED_PROVISIONAL`.

## Source boundary

This is a new public Git history, exported from the existing development checkout at commit `58973f1`, including its pre-existing README/CLI changes and optional LeHome exporter. The private development history, runtime environments, full result library and Robot Data Studio application are excluded.

Publication changes:

- Retained existing garment/solver/quality code and tests.
- Replaced the private README with standalone documentation.
- Replaced the hard-coded host Isaac launcher with a required environment variable and an actionable error.
- Changed dataset configuration paths to portable relative examples and starter GPU selections to physical GPU 0. Users must select their own available device before launching.
- Completed the public VBD starter simulation settings from the actual recorded recipe; material values and quality gates remain explicit. No validity is asserted for untested garment/configuration combinations.
- Added common dependency metadata and a CLI entry point; solver/generator runtimes remain separate.
- Added an independent static viewer with no runtime dependencies, shared camera/time, downloads and visible failure gates.
- Added MIT licensing and third-party notices.

No code or artifacts were removed from the development checkout.

## Included evidence

Ten existing `tshirt-000` / `medium` runs were selected from the historical contact-repair report: five backends × hanging/drop. Two selected PhysX cases failed their historical gates. They remain included; no gate was relaxed and no experiment was re-run to create this release.

Each catalog entry records its source run ID, original trajectory SHA-256, source request-file SHA-256, nominal material, numerical settings and historical metrics. Original NPZ and validation files are preserved byte-for-byte. Public configuration and initialization snapshots omit hardware allocation; those snapshots must not be mistaken for byte-identical source request files.

The source mesh, face topology and initial positions are checked across cases. Display position JSON is rounded to six decimal places and remains a derivative, not scoring ground truth. Saved frames do not prove collision-free behavior between frames.

## Verification

Actual checks and results are recorded below after packaging. Clean-machine installation, current GPU runtime compatibility and real-world material accuracy are not validated in this release.

### Completed release checks

- 23 CPU tests passed across `test_pipeline`, `test_comparison`, `test_native_io`, `test_physx_material` and `test_repair`, using the existing generation environment. The PhysX mock suite was given `CLOTHFORGE_ISAAC_PYTHON=/test-only/isaac/python.sh`; no native process was launched.
- `node --check docs/app.js` passed. All Python files compiled in memory, and all included JSON parsed.
- Chromium checked all 10 case selections, play/pause/seek, camera rendering, downloads, visible failure gates and a 390-pixel mobile viewport. No JavaScript exceptions or mobile horizontal overflow were observed.
- All 10 original trajectory SHA-256 values matched, original validation files were byte-identical, and the shared mesh fingerprint matched all case configurations.
- Maximum initial-position difference across backends/scenes: `2.9491e-8 m`; display rounding error: at most `5e-7 m`.
- A scan of all published text found no development host paths or recognized credential/private-key patterns. Environments, runtime logs, source history and third-party directories are excluded. This is a bounded scan, not a formal security audit.

No new GPU simulation, external reviewer, dependency installation or clean-machine setup test was performed. The XPBD runtime contact suite was not run in the CPU-only release environment. A clean runtime test for each backend remains future work.
