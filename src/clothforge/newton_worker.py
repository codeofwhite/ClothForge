"""Native Newton VBD worker. No generator dependency is imported by Newton."""
from pathlib import Path
import json
import os
import time
import subprocess
import sys
import fcntl
import numpy as np
from .schema import read_json,write_json,digest,canonical_hash,validate_material
from .quality import rest_matrices,evaluate_trajectory
from .mesh import save_preview,save_glb,save_viewer

def simulate(garment_dir,material_id,scenario,output,run_config=None,engine_id="newton_vbd"):
    if engine_id not in {"newton_vbd","newton_xpbd"}:raise ValueError("unsupported Newton solver")
    from importlib.metadata import version
    import warp as wp
    import newton
    directory=Path(garment_dir);output=Path(output);output.mkdir(parents=True,exist_ok=False)
    manifest=read_json(directory/"manifest.json")
    config=read_json(directory/("config-"+material_id+".json"))
    source_config_sha256=canonical_hash(config)
    from .repair import resolve_config
    profile=read_json(run_config) if run_config else {"id":"legacy-v1","overrides":{}}
    config=resolve_config(config,profile)
    if digest(directory/"mesh.npz")!=config["mesh_sha256"]:raise ValueError("mesh fingerprint mismatch")
    material=config["material"];validate_material(material)
    settings=config["simulation"];gates=config["gates"]
    write_json(output/"request.json",{"garment":manifest["id"],"garment_manifest_sha256":digest(directory/"manifest.json"),"config":config,"scenario":scenario,
               "source_configuration_sha256":source_config_sha256,"profile":profile,
               "worker_sources":{p.name:digest(p) for p in Path(__file__).parent.glob("*.py")}})
    data=np.load(directory/"mesh.npz",allow_pickle=False)
    v=data["vertices"].copy();f=data["faces"];rest=data["rest_triangles"];panel_ids=data["face_panels"]
    seam_edges={tuple(edge) for edge in data["seam_edges"].tolist()}
    v[:,2]+=0.35-v[:,2].min()
    inv,areas=rest_matrices(rest)
    wp.config.kernel_cache_dir=str(Path(os.environ.get("CLOTHFORGE_CACHE_ROOT",output.parents[1]/"cache"))/"warp")
    wp.init();wp.set_device("cuda:0")
    builder=newton.ModelBuilder()
    ground_cfg=builder.default_shape_cfg.copy()
    if "ground_contact_ke" in settings:ground_cfg.ke=settings["ground_contact_ke"]
    if "ground_contact_kd" in settings:ground_cfg.kd=settings["ground_contact_kd"]
    builder.add_ground_plane(cfg=ground_cfg)
    builder.add_cloth_mesh(pos=wp.vec3(0,0,0),rot=wp.quat_identity(),scale=1.0,vel=wp.vec3(0,0,0),
                           vertices=v.tolist(),indices=f.reshape(-1).tolist(),density=material["mass_per_area"],
                           tri_ke=material["tri_ke"],tri_ka=material["tri_ka"],tri_kd=material["tri_kd"],
                           edge_ke=material["edge_ke"],edge_kd=material["edge_kd"],
                           particle_radius=settings["particle_radius_m"],validate_mesh=True,
                           **({"add_springs":True,"spring_ke":material["tri_ke"],"spring_kd":material["tri_kd"] if settings.get("xpbd_surface_contact",{}).get("spring_damping_from_material",False) else 0.0} if engine_id=="newton_xpbd" else {}))
    if len(builder.tri_poses)!=len(f):raise ValueError("Newton rejected or reordered input triangles")
    # The 3D assembly pose is not a stress-free reference. Restore the actual
    # planar material metric and area-weighted masses before finalization.
    builder.tri_poses=inv.tolist();builder.tri_areas=areas.tolist()
    masses=np.bincount(f.reshape(-1),weights=np.repeat(areas*material["mass_per_area"]/3,3),minlength=len(v))
    if np.any(masses<=0):raise ValueError("unreferenced or massless mesh particle")
    builder.particle_mass=masses.tolist()
    builder.edge_rest_angle=[0.0]*len(builder.edge_rest_angle)
    seam_bending_edges=0
    for i,edge in enumerate(builder.edge_indices):
        if tuple(sorted([int(edge[2]),int(edge[3])])) in seam_edges:
            # Shared vertices are an ideal permanent seam, with a free hinge.
            builder.edge_bending_properties[i]=(0.,0.);seam_bending_edges+=1
    pins=[]
    if scenario=="hanging":
        if manifest.get("fixtures"):
            pins=manifest["fixtures"]["hanging_pins"]
        else:
            width=manifest["parameters"].get("width_m",manifest["parameters"].get("waist_width_m"))
            height=float(v[:,2].max())
            for x in [-width*.45,width*.45]:
                target=np.array([x,0,height])
                ids=np.argsort(np.linalg.norm(v-target,axis=1))[:2]
                pins.extend(int(i) for i in ids)
        pins=sorted(set(pins))
        for i in pins:builder.particle_mass[i]=0.
    elif scenario!="drop":raise ValueError("unsupported scenario")
    if engine_id=="newton_xpbd":
        from .comparison import initial_state
        _,flat,_=initial_state(data,manifest,scenario)
        # Mesh builder includes opposite-vertex springs across every hinge. These
        # would weld front/back panels through free seams; retain mesh edges only.
        pairs=np.asarray(builder.spring_indices).reshape(-1,2)
        edge_set={tuple(sorted(e)) for t in f for e in [(t[0],t[1]),(t[1],t[2]),(t[2],t[0])]}
        for i,(a,b) in enumerate(pairs):
            if tuple(sorted((a,b))) not in edge_set:builder.spring_stiffness[i]=0.
        builder.spring_rest_length=[float(np.linalg.norm(flat[a]-flat[b])) for a,b in pairs]
    builder.color(include_bending=True)
    model=builder.finalize()
    model.soft_contact_ke=settings["soft_contact_ke"];model.soft_contact_kd=settings["soft_contact_kd"];model.soft_contact_mu=material["friction"]
    if engine_id=="newton_xpbd":
        from .xpbd_contact import SurfaceXPBD
        solver=SurfaceXPBD(model,iterations=settings["iterations"])
    else:
        solver=newton.solvers.SolverVBD(model,iterations=settings["iterations"],particle_enable_self_contact=True,
                                        particle_self_contact_radius=settings["self_contact_radius_m"],
                                        particle_self_contact_margin=settings["self_contact_margin_m"],
                                        particle_vertex_contact_buffer_size=settings["vertex_contact_buffer_size"],
                                        particle_edge_contact_buffer_size=settings["edge_contact_buffer_size"],
                                        particle_topological_contact_filter_threshold=settings["topological_contact_filter_threshold"],
                                        particle_collision_detection_interval=settings["collision_detection_interval"])
    state0,state1=model.state(),model.state();control=model.control();contacts=model.contacts()
    surface_contact=None
    if engine_id=="newton_xpbd" and "xpbd_surface_contact" in settings:
        from .xpbd_contact import SurfaceContact
        surface_contact=SurfaceContact(model,f,inv,settings["xpbd_surface_contact"],area_stiffness=material["tri_ka"] if settings["xpbd_surface_contact"].get("area_stiffness_from_material",False) else 0.)
    write_json(output/"initialization.json",{
        "pins":pins,"fixture":"fixed mesh vertices at two shoulders/waist locations" if pins else "none",
        "material_rest_reference":"per-face planar pattern metric","material_area_m2":float(areas.sum()),
        "unconstrained_total_mass_kg":float(masses.sum()),"free_seam_hinge_edges":seam_bending_edges,
        "seam_representation":"shared vertices, permanent ideal stitching",
        "gravity_m_s2":[0,0,-9.81],"collision":"native particle contacts plus per-step mass-weighted vertex-face/edge-edge surface projections and explicit strain limit; discrete, not certified CCD" if surface_contact is not None else "particle contacts; no continuous surface self-contact guarantee" if engine_id=="newton_xpbd" else "Newton VBD vertex-triangle and edge-edge self contact",
        "unmapped_material_parameters":([key for key,flag in [("tri_ka","area_stiffness_from_material"),("tri_kd","spring_damping_from_material")] if not settings.get("xpbd_surface_contact",{}).get(flag,False)]) if engine_id=="newton_xpbd" else [],
        "solver_extensions":settings.get("xpbd_surface_contact"),
        "solver":engine_id,"material_mapping":"mesh-edge springs and bending constraints; opposite-vertex springs disabled to preserve free seams; area uses nominal tri_ka when area_stiffness_from_material is enabled; damping uses nominal tri_kd when spring_damping_from_material is enabled; not equivalent to triangular VBD elasticity" if engine_id=="newton_xpbd" else "native triangular VBD elasticity",
        "physical_gpu":os.environ.get("CUDA_VISIBLE_DEVICES"),"logical_gpu":"cuda:0",
        "native_versions":{"newton":version("newton"),"warp-lang":version("warp-lang"),"numpy":version("numpy")},
        "resolved_simulation":settings,"ground_material":{"ke":float(ground_cfg.ke),"kd":float(ground_cfg.kd),"mu":float(ground_cfg.mu)},
        "profile_id":profile["id"],
    })
    positions=[state0.particle_q.numpy()];velocities=[state0.particle_qd.numpy()]
    dt=1/settings["fps"]/settings["substeps"];frames=round(settings["duration_s"]*settings["fps"])
    started=time.monotonic()
    for frame in range(frames):
        for _ in range(settings["substeps"]):
            state0.clear_forces();model.collide(state0,contacts)
            solver.step(state0,state1,control,contacts,dt)
            if surface_contact is not None:surface_contact.step(solver,state1,dt)
            state0,state1=state1,state0
        positions.append(state0.particle_q.numpy());velocities.append(state0.particle_qd.numpy())
        if not np.isfinite(positions[-1]).all() or not np.isfinite(velocities[-1]).all():break
        if time.monotonic()-started>settings["max_gpu_wall_seconds"]:raise TimeoutError("approved GPU time budget exhausted")
        if (frame+1)%15==0:print("FRAME",frame+1,"/",frames,flush=True)
    positions=np.asarray(positions);velocities=np.asarray(velocities)
    physical_wall=time.monotonic()-started
    with (output/"trajectory.npz").open("xb") as stream:
        np.savez_compressed(stream,positions=positions,velocities=velocities,faces=f,rest_triangles=rest,
                            times=np.arange(len(positions))/settings["fps"],pins=np.asarray(pins,dtype=np.int32))
    result=evaluate_trajectory(positions,velocities,f,rest,gates)
    from .diagnostics import diagnose_trajectory
    diagnostics=diagnose_trajectory(positions,velocities,f,rest,pins,np.arange(len(positions))/settings["fps"],
                                    material["mass_per_area"],dt,gates,scenario)
    write_json(output/"diagnostics.json",diagnostics)
    if len(positions)!=frames+1:result["failed_gates"].append("incomplete_horizon");result["outcome"]="failed"
    result.update({"contract":"clothforge_validation_v1","garment_id":manifest["id"],"material_id":material_id,
                   "scenario":scenario,"engine_id":engine_id,"evaluator_version":"clothforge_sampled_surface_v1",
                   "scope":"synthetic_garment_hanging_drop_2s_saved_frame_checks",
                   "configuration_sha256":canonical_hash(config),"trajectory_sha256":digest(output/"trajectory.npz"),
                   "physical_wall_seconds":physical_wall,"physical_steps":(len(positions)-1)*settings["substeps"],
                   "calibration_status":"synthetic_unmeasured","real_world_validated":False})
    result["profile_id"]=profile["id"]
    write_json(output/"validation.json",result)
    if np.isfinite(positions).all():
        save_preview(output/"preview.png",positions[-1],f,manifest["id"]+" | "+scenario+" | "+result["outcome"],panel_ids)
        save_glb(output/"final.glb",positions[-1],f,panel_ids)
        save_viewer(output/"replay.html",positions,f,manifest["id"]+" | "+scenario+" | "+result["outcome"],settings["fps"])
    print("RESULT",json.dumps(result),flush=True)
    return result

def run_pilot(generation,config_path,output):
    from datetime import datetime,timezone
    generation=Path(generation);output=Path(output);output.mkdir(parents=True,exist_ok=True)
    config=read_json(config_path);settings=config["simulation"]
    index=read_json(generation/"index.json")
    if index["failures"]:raise ValueError("generation failures must be resolved before pilot")
    if index["config_sha256"]!=canonical_hash(config):raise ValueError("pilot config differs from generation config")
    gpu=str(settings["gpu_physical_index"])
    used=subprocess.check_output(["nvidia-smi","-i",gpu,"--query-gpu=memory.used","--format=csv,noheader,nounits"],text=True).strip()
    if int(used)>256:raise RuntimeError("selected GPU is busy; refusing to start")
    results=[];started=time.monotonic()
    with (output/"gpu.lock").open("a") as lock:
        fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        env=os.environ.copy();env["CUDA_VISIBLE_DEVICES"]=gpu;env["PYTHONDONTWRITEBYTECODE"]="1"
        env["MPLCONFIGDIR"]=str(output/"cache/matplotlib");env["TMPDIR"]=str(output/"cache/tmp")
        Path(env["TMPDIR"]).mkdir(parents=True,exist_ok=True)
        for garment in index["garments"]:
            if garment["variant"] not in settings["variants"]:continue
            for scenario in settings["scenarios"]:
                run_id=garment["id"]+"-"+settings["material_id"]+"-"+scenario
                destination=output/"runs"/run_id;destination.parent.mkdir(parents=True,exist_ok=True)
                command=[sys.executable,"-B","-m","clothforge","simulate","--garment",str(generation/"garments"/garment["id"]),
                         "--material",settings["material_id"],"--scenario",scenario,"--output",str(destination)]
                remaining=settings["max_gpu_wall_seconds"]-(time.monotonic()-started)
                if remaining<=0:raise TimeoutError("pilot GPU budget exhausted")
                size=sum(p.stat().st_size for p in output.parent.rglob("*") if p.is_file())
                if size>settings["max_output_bytes"]:raise RuntimeError("pilot storage budget exhausted")
                print("START",run_id,flush=True)
                log_path=destination.parent/(run_id+".log")
                with log_path.open("x") as log:
                    try:
                        process=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=remaining)
                        code=process.returncode;error=None
                    except subprocess.TimeoutExpired:
                        code=-1;error="approved GPU time budget exhausted"
                result_path=destination/"validation.json"
                if result_path.exists():result=read_json(result_path)
                else:
                    result={"contract":"clothforge_validation_v1","garment_id":garment["id"],"material_id":settings["material_id"],
                            "scenario":scenario,"engine_id":"newton_vbd","evaluator_version":"clothforge_sampled_surface_v1",
                            "scope":"synthetic_garment_hanging_drop_2s_saved_frame_checks","outcome":"failed",
                            "metrics":{},"failed_gates":["worker_error"],"returncode":code,"error":error or log_path.read_text()[-4000:]}
                    destination.mkdir(exist_ok=True);write_json(result_path,result)
                results.append(result);print("FINISH",run_id,result["outcome"],flush=True)
                # A launch failure is diagnostic; do not repeat the same broken
                # worker across the remaining batch.
                if "worker_error" in result["failed_gates"]:break
            if results and "worker_error" in results[-1]["failed_gates"]:break
    summary={"contract":"clothforge_pilot_report_v1","created_at":datetime.now(timezone.utc).isoformat(),
             "config_sha256":canonical_hash(config),"requested_runs":len(settings["variants"])*len(settings["scenarios"])*len(config["categories"]),
             "completed_records":len(results),"passed":sum(r["outcome"]=="passed" for r in results),
             "failed":sum(r["outcome"]=="failed" for r in results),"wall_seconds":time.monotonic()-started,
             "results":results,"review_status":"SELF_REVIEWED_PROVISIONAL","real_world_validated":False}
    write_json(output/"report.json",summary)
    return summary
