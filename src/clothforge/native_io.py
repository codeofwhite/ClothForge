"""Shared immutable input and sampled-trajectory evaluation for native adapters."""
from pathlib import Path
import json
import numpy as np
from .schema import read_json,write_json,digest,canonical_hash,validate_material
from .comparison import initial_state
from .quality import evaluate_trajectory

def load(directory,material_id,scenario,output,run_config=None):
    directory,output=Path(directory),Path(output)
    output.mkdir(parents=True,exist_ok=False)
    manifest=read_json(directory/'manifest.json');config=read_json(directory/f'config-{material_id}.json')
    from .repair import resolve_config
    profile=read_json(run_config) if run_config else {'id':'native-default','overrides':{}}
    config=resolve_config(config,profile);validate_material(config['material'])
    if digest(directory/'mesh.npz')!=config['mesh_sha256']:raise ValueError('mesh fingerprint mismatch')
    with np.load(directory/'mesh.npz',allow_pickle=False) as source:data={k:source[k].copy() for k in source.files}
    v,flat,pins=initial_state(data,manifest,scenario)
    write_json(output/'request.json',{'config':config,'scenario':scenario,'profile':profile,'worker_sources':{p.name:digest(p) for p in Path(__file__).parent.glob('*.py')}})
    np.savez(output/'native-input.npz',**data,initial_positions=v,flat_positions=flat,pins=np.array(pins,dtype=np.int32))
    return manifest,config,data,v,flat,pins

def finish(output,manifest,config,material_id,scenario,engine_id):
    output=Path(output)
    with np.load(output/'trajectory.npz',allow_pickle=False) as d:ps,vs,f,rest,times,pins=[d[k] for k in ['positions','velocities','faces','rest_triangles','times','pins']]
    result=evaluate_trajectory(ps,vs,f,rest,config['gates'])
    settings=config['simulation']
    if len(ps)!=round(settings['duration_s']*settings['fps'])+1:result['failed_gates'].append('incomplete_horizon')
    if len(pins) and np.max(np.linalg.norm(ps[:,pins]-ps[0,pins],axis=-1))>1e-4:result['failed_gates'].append('fixture_drift')
    if result['failed_gates']:result['outcome']='failed'
    init=read_json(output/'initialization.json')
    result.update(contract='clothforge_validation_v1',garment_id=manifest['id'],material_id=material_id,scenario=scenario,engine_id=engine_id,
        evaluator_version='clothforge_sampled_surface_v1',scope='synthetic_garment_hanging_drop_2s_saved_frame_checks',
        configuration_sha256=canonical_hash(config),trajectory_sha256=digest(output/'trajectory.npz'),physical_wall_seconds=init['physical_wall_seconds'],
        calibration_status='synthetic_unmeasured',real_world_validated=False)
    from .diagnostics import diagnose_trajectory
    diagnostics=diagnose_trajectory(ps,vs,f,rest,pins,times,config['material']['mass_per_area'],1/settings['fps']/settings['substeps'],config['gates'],scenario) if np.isfinite(ps).all() and np.isfinite(vs).all() else {'unavailable_reason':'nonfinite state: strain diagnostics cannot be computed; physical verdict failed'}
    write_json(output/'diagnostics.json',diagnostics)
    write_json(output/'validation.json',result)
    from .mesh import save_preview,save_glb,save_viewer
    if np.isfinite(ps).all():
        with np.load(output/'native-input.npz',allow_pickle=False) as d:panels=d['face_panels']
        save_preview(output/'preview.png',ps[-1],f,manifest['id']+' | '+engine_id+' | '+result['outcome'],panels)
        save_glb(output/'final.glb',ps[-1],f,panels);save_viewer(output/'replay.html',ps,f,manifest['id']+' | '+engine_id,settings['fps'])
    print('RESULT',json.dumps(result),flush=True)
    return result
