"""Genesis IPC sewn garments. Current pose and planar rest metric are distinct."""
from pathlib import Path
import json,os,time
import numpy as np
from .schema import read_json,write_json,digest,canonical_hash,validate_material
from .comparison import initial_state,ipc_material
from .quality import evaluate_trajectory,rest_matrices
from .mesh import save_preview,save_glb,save_viewer

def simulate(garment_dir,material_id,scenario,output,run_config=None):
    import genesis as gs
    import uipc
    from importlib.metadata import version
    from uipc.backend import SceneVisitor
    directory=Path(garment_dir);output=Path(output);output.mkdir(parents=True,exist_ok=False)
    manifest=read_json(directory/'manifest.json');config=read_json(directory/f'config-{material_id}.json')
    from .repair import resolve_config
    profile=read_json(run_config) if run_config else {'id':'ipc-default','overrides':{}}
    config=resolve_config(config,profile);material=config['material'];validate_material(material)
    if digest(directory/'mesh.npz')!=config['mesh_sha256']:raise ValueError('mesh fingerprint mismatch')
    write_json(output/'request.json',{'config':config,'scenario':scenario,'profile':profile,'worker_sources':{p.name:digest(p) for p in Path(__file__).parent.glob('*.py')}})
    data=np.load(directory/'mesh.npz',allow_pickle=False);v,flat,pins=initial_state(data,manifest,scenario)
    f=data['faces'];rest=data['rest_triangles'];mapping=ipc_material(material);settings=config['simulation']
    seams={tuple(e) for e in data['seam_edges'].tolist()}
    # Load the separated assembly so the mesh loader cannot weld opposite panels.
    with (output/'assembly.obj').open('x') as stream:
        for p in v:stream.write('v '+' '.join(map(str,p))+'\n')
        for t in f:stream.write('f '+' '.join(str(int(i)+1) for i in t)+'\n')
    gs.init(backend=gs.gpu,precision='64',seed=manifest['seed'],logging_level='warning')
    substeps=settings['substeps'];dt=1/settings['fps']/substeps
    scene=gs.Scene(sim_options=gs.options.SimOptions(dt=dt,substeps=1,gravity=(0,0,-9.81)),
        coupler_options=gs.options.IPCCouplerOptions(newton_max_iterations=settings['iterations'],newton_tolerance=1e-5,
          newton_translation_tolerance=1e-5,newton_semi_implicit_enable=False,newton_use_adaptive_tolerance=False,
          linear_system_tolerance=1e-8,contact_d_hat=.0005,contact_resistance=1e7,
          contact_eps_velocity=.001,contact_friction_enable=True,linesearch_report_energy=False),show_viewer=False)
    scene.add_entity(morph=gs.morphs.Plane(), material=gs.materials.Rigid(coup_type='ipc_only',coup_friction=material['friction']))
    scene.add_entity(morph=gs.morphs.Mesh(file=str(output/'assembly.obj'),convexify=False,decimate=False),
        material=gs.materials.FEM.Cloth(E=mapping['young_pa'],nu=mapping['poisson'],rho=mapping['density_kg_m3'],
          thickness=mapping['thickness_m'],bending_stiffness=mapping['bending_stiffness'],friction_mu=mapping['friction']))
    cp=scene.sim.coupler;state={};oldadd=cp._add_fem_entities_to_ipc
    def configure():
        oldadd();found=cp._ipc_objects.find('cloth_0_0')
        if len(found)!=1:raise ValueError('expected one cloth')
        cur,ref=cp._ipc_scene.geometries().find(int(list(found[0].geometries().ids())[0]))
        cg,rg=cur.geometry(),ref.geometry();actual=np.array(rg.positions().view()).reshape(-1,3)
        # Mapping uses unique separated current coordinates, never coincident rest vertices.
        order=np.linalg.norm(v[:,None,:]-actual[None,:,:],axis=2).argmin(1)
        if len(actual)!=len(v) or len(set(order))!=len(v) or not np.allclose(actual[order],v,atol=1e-8):raise ValueError('IPC changed vertices')
        inverse=np.argsort(order);native_faces=np.array(rg.triangles().topo().view()).reshape(-1,3)
        if sorted(map(tuple,np.sort(inverse[native_faces],axis=1)))!=sorted(map(tuple,np.sort(f,axis=1))):raise ValueError('IPC changed topology')
        uipc.view(rg.positions()).reshape(-1,3)[order]=flat
        uipc.view(cg.positions()).reshape(-1,3)[order]=v
        _,rest_areas=rest_matrices(rest)
        vertex_volume=np.bincount(f.reshape(-1),weights=np.repeat(rest_areas*2*mapping['thickness_m']/3,3),minlength=len(v))
        for geom in [cg,rg]:
            uipc.view(geom.vertices().find('volume')).reshape(-1)[order]=vertex_volume
            edges=np.array(geom.edges().topo().view()).reshape(-1,2)
            bend=uipc.view(geom.edges().find('bending_stiffness')).reshape(-1)
            for i,e in enumerate(inverse[edges]):
                if tuple(sorted(e)) in seams:bend[i]=0
            uipc.view(geom.vertices().find(uipc.builtin.is_fixed)).reshape(-1)[order[pins]]=1
        state.update(geom=cg,order=order)
    cp._add_fem_entities_to_ipc=configure
    scene.build()
    def positions():
        g,=uipc.geometry.apply_transform(state['geom'])
        return np.array(g.positions().view()).reshape(-1,3)[state['order']].copy()
    if not np.allclose(positions(),v,atol=1e-8):raise ValueError('IPC initial pose mismatch')
    _,areas=rest_matrices(rest)
    actual_mass=float(np.array(state['geom'].vertices().find('volume').view()).sum()*mapping['density_kg_m3'])
    if not np.isclose(actual_mass,areas.sum()*material['mass_per_area'],rtol=1e-8):raise ValueError('IPC native mass mismatch')
    write_json(output/'initialization.json',{'pins':pins,'material_rest_reference':'per-face planar pattern metric',
        'material_area_m2':float(areas.sum()),'unconstrained_total_mass_kg':actual_mass,'mass_source':'native vertex volumes times density; IPC thickness is half-thickness',
        'gravity_m_s2':[0,0,-9.81],'ground_z_m':0,'seam_representation':'shared vertices, free seam hinges',
        'material_mapping':mapping,'solver':'StrainLimitingBaraffWitkinShell + DiscreteShellBending / IPC',
        'substeps':substeps,'dt':dt,'native_versions':{'genesis':version('genesis-world'),'pyuipc':version('pyuipc')},
        'physical_gpu':os.environ.get('CUDA_VISIBLE_DEVICES'),'initial_pose_max_error':float(abs(positions()-v).max())})
    ps=[positions()];vs=[np.zeros_like(v)];frames=round(settings['duration_s']*settings['fps']);started=time.monotonic()
    for frame in range(frames):
        before=None
        for _ in range(substeps):before=positions();scene.step()
        now=positions();ps.append(now);vs.append((now-before)/dt)
        if not np.isfinite(now).all():break
        if time.monotonic()-started>settings['max_gpu_wall_seconds']:raise TimeoutError('GPU budget exhausted')
        if (frame+1)%15==0:print('FRAME',frame+1,'/',frames,flush=True)
    ps=np.asarray(ps);vs=np.asarray(vs)
    np.savez_compressed(output/'trajectory.npz',positions=ps,velocities=vs,faces=f,rest_triangles=rest,times=np.arange(len(ps))/settings['fps'],pins=np.array(pins,dtype=np.int32))
    result=evaluate_trajectory(ps,vs,f,rest,config['gates'])
    if len(ps)!=frames+1:result['failed_gates'].append('incomplete_horizon');result['outcome']='failed'
    result.update(contract='clothforge_validation_v1',garment_id=manifest['id'],material_id=material_id,scenario=scenario,
        engine_id='genesis_ipc',evaluator_version='clothforge_sampled_surface_v1',scope='synthetic_garment_hanging_drop_2s_saved_frame_checks',
        configuration_sha256=canonical_hash(config),trajectory_sha256=digest(output/'trajectory.npz'),physical_wall_seconds=time.monotonic()-started,
        calibration_status='synthetic_unmeasured',real_world_validated=False,material_mapping=mapping)
    write_json(output/'validation.json',result)
    if np.isfinite(ps).all():
        save_preview(output/'preview.png',ps[-1],f,manifest['id']+' | IPC | '+result['outcome'],data['face_panels'])
        save_glb(output/'final.glb',ps[-1],f,data['face_panels']);save_viewer(output/'replay.html',ps,f,manifest['id']+' | IPC',settings['fps'])
    print('RESULT',json.dumps(result),flush=True)
    return result
