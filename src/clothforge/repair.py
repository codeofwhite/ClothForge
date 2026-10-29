"""Bounded, source-preserving numerical repair experiments."""
from copy import deepcopy
from pathlib import Path
import fcntl
import json
import math
import os
import shutil
import subprocess
import sys
import time

from .schema import canonical_hash, digest, read_json, write_json

SOLVER_DEFAULTS = {
    "topological_contact_filter_threshold": 2,
    "collision_detection_interval": 0,
    "soft_contact_ke": 1e4,
    "soft_contact_kd": 10.0,
    "vertex_contact_buffer_size": 64,
    "edge_contact_buffer_size": 128,
}
ALLOWED_OVERRIDES = set(SOLVER_DEFAULTS) | {"substeps", "iterations", "ground_contact_ke", "ground_contact_kd"}


def resolve_config(source, profile):
    """Only numerical/contact settings may change; material, geometry and gates stay fixed."""
    if set(profile) - {"id", "overrides"}:
        raise ValueError("unsupported profile fields")
    overrides = profile.get("overrides", {})
    unknown = set(overrides) - ALLOWED_OVERRIDES
    if unknown:
        raise ValueError("unsupported repair overrides: " + str(sorted(unknown)))
    for key, value in overrides.items():
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
            raise ValueError("finite numeric override required: " + key)
        if key in {"topological_contact_filter_threshold", "collision_detection_interval"}:
            if not isinstance(value, int) or value < 0:
                raise ValueError("nonnegative integer required: " + key)
        elif value <= 0:
            raise ValueError("positive override required: " + key)
        if key in {"substeps", "iterations", "vertex_contact_buffer_size", "edge_contact_buffer_size"} and not isinstance(value, int):
            raise ValueError("integer required: " + key)
    config = deepcopy(source)
    for key, value in SOLVER_DEFAULTS.items():
        config["simulation"].setdefault(key, value)
    config["simulation"].update(overrides)
    return config


def run_repair(generation, config_path, output, stage="diagnose", profile_id=None):
    root = Path(output).resolve()
    generation = Path(generation).resolve()
    plan = read_json(config_path)
    index = read_json(generation / "index.json")
    if index["failures"]:
        raise ValueError("generation is incomplete")
    expected = read_json(generation / "generation_config.json")
    if canonical_hash(expected) != index["config_sha256"]:
        raise ValueError("generation configuration changed")
    root.mkdir(parents=True, exist_ok=True)
    plan_file = root / "plan.json"
    if plan_file.exists():
        if read_json(plan_file) != plan:
            raise ValueError("repair plan changed; use a new output root")
    else:
        write_json(plan_file, plan)
    profiles = [p for p in plan["profiles"] if profile_id is None or p["id"] == profile_id]
    if not profiles:
        raise ValueError("unknown profile")
    if stage != "diagnose" and len(profiles) != 1:
        raise ValueError("confirm/holdout requires one selected profile")
    if stage == "diagnose":
        garments = plan["diagnostic_garments"]
    else:
        variants = plan["confirm_variants"] if stage == "confirm" else plan["holdout_variants"]
        garments = [g["id"] for g in index["garments"] if g["variant"] in variants]
    gpu = str(plan["gpu_physical_index"])
    used = subprocess.check_output(["nvidia-smi", "-i", gpu, "--query-gpu=memory.used", "--format=csv,noheader,nounits"], text=True).strip()
    if int(used) > 256:
        raise RuntimeError("selected GPU is busy")
    ledger_path = root / "launches.jsonl"
    previous = [json.loads(line) for line in ledger_path.read_text().splitlines()] if ledger_path.exists() else []
    spent = sum(row.get("wall_seconds", 0) for row in previous if row["event"] == "finished")
    launched = sum(row["event"] == "started" for row in previous)
    started = time.monotonic()
    results = []
    env = os.environ.copy()
    env.update({"CUDA_VISIBLE_DEVICES": gpu, "PYTHONDONTWRITEBYTECODE": "1",
                "CLOTHFORGE_CACHE_ROOT": str(root / "cache"),
                "MPLCONFIGDIR": str(root / "cache/matplotlib"), "TMPDIR": str(root / "cache/tmp")})
    Path(env["TMPDIR"]).mkdir(parents=True, exist_ok=True)

    def event(value):
        with ledger_path.open("a") as stream:
            stream.write(json.dumps(value, sort_keys=True) + "\n")

    with (root / "gpu.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        for profile in profiles:
            folder = root / stage / profile["id"]
            profile_path = folder / "profile.json"
            if profile_path.exists():
                if read_json(profile_path) != profile:
                    raise ValueError("profile changed")
            else:
                write_json(profile_path, profile)
            snapshot = folder / "source_snapshot"
            if not snapshot.exists():
                snapshot.mkdir()
                for source in Path(__file__).parent.glob("*.py"):
                    shutil.copy2(source, snapshot / source.name)
            for garment in garments:
                for scenario in plan["scenarios"]:
                    name = garment + "-" + plan["material_id"] + "-" + scenario
                    dest = folder / "runs" / name
                    result_path = dest / "validation.json"
                    if result_path.exists():
                        record = read_json(result_path)
                        request = read_json(dest / "request.json")
                        source_config = read_json(generation / "garments" / garment / ("config-" + plan["material_id"] + ".json"))
                        if (request["config"] != resolve_config(source_config, profile)
                                or digest(dest / "trajectory.npz") != record["trajectory_sha256"]):
                            raise ValueError("existing run provenance mismatch")
                        results.append(record)
                        continue
                    if dest.exists():
                        raise RuntimeError("incomplete run retained for diagnosis: " + str(dest))
                    remaining = plan["max_gpu_wall_seconds"] - spent - (time.monotonic() - started)
                    if remaining <= 0 or launched >= plan["max_runs"]:
                        raise RuntimeError("approved repair budget exhausted")
                    if sum(p.stat().st_size for p in root.rglob("*") if p.is_file()) >= plan["max_output_bytes"]:
                        raise RuntimeError("approved output budget exhausted")
                    dest.parent.mkdir(parents=True, exist_ok=True)
                    command = [sys.executable, "-B", "-m", "clothforge", "simulate", "--garment",
                               str(generation / "garments" / garment), "--material", plan["material_id"],
                               "--scenario", scenario, "--output", str(dest), "--run-config", str(profile_path)]
                    stamp = time.monotonic()
                    event({"event": "started", "profile": profile["id"], "stage": stage, "run": name, "command": command})
                    launched += 1
                    print("START", stage, profile["id"], name, flush=True)
                    with dest.with_suffix(".log").open("x") as log:
                        try:
                            process = subprocess.run(command, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=remaining)
                        finally:
                            event({"event": "finished", "profile": profile["id"], "stage": stage, "run": name,
                                   "wall_seconds": time.monotonic() - stamp})
                    if process.returncode or not result_path.exists():
                        raise RuntimeError("worker failed; inspect " + str(dest.with_suffix(".log")))
                    result = read_json(result_path)
                    results.append(result)
                    print("FINISH", profile["id"], name, result["outcome"], result["failed_gates"], flush=True)
        summary = {"stage": stage, "profiles": [p["id"] for p in profiles], "runs": len(results),
                   "passed": sum(r["outcome"] == "passed" for r in results), "results": results,
                   "review_status": "SELF_REVIEWED_PROVISIONAL", "real_world_validated": False}
        report = root / (stage + "-" + (profile_id or "all") + "-report.json")
        if report.exists():
            if read_json(report) != summary:
                raise ValueError("completed report differs")
        else:
            write_json(report, summary)
    return summary
