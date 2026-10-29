"""PPF native CUDA solver isolated in a checksum-pinned official image."""
from pathlib import Path
import os,sys,subprocess,time,shutil
import numpy as np
from .schema import read_json,write_json
IMAGE='sha256:2a1fdc6da8775c05d5f57e3441e59dde0b7a0f99dfe03676abaf024c80e2e8ae'
MANIFEST='sha256:3be1692bf1cf29f6a4e0435e746ca43382c69ce049c304bf501e9c943062320b'

def simulate(garment_dir,material_id,scenario,output,run_config=None):
    from .native_io import load,finish
    manifest,config,*_=load(garment_dir,material_id,scenario,output,run_config)
    output=Path(output).resolve();name='clothforge-'+output.parent.name
    command=['docker','run','--rm','--name',name,'--network','none','--gpus','device='+os.environ['CUDA_VISIBLE_DEVICES'],
        '-v',str(Path(__file__).resolve().parents[1])+':/cf:ro','-v',str(output)+':/work',
        '-e','PYTHONPATH=/cf:/root/ppf-contact-solver','-e','CUDA_VISIBLE_DEVICES=0','-w','/root/ppf-contact-solver',
        '--entrypoint','/root/.local/share/ppf-cts/venv/bin/python',IMAGE,'-u','-B','-m','clothforge.ppf_worker','/work']
    try:subprocess.run(command,check=True)
    finally:subprocess.run(['docker','rm','-f',name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=20)
    return finish(output,manifest,config,material_id,scenario,'ppf')

def native(output):
    output=Path(output);config=read_json(output/'request.json')['config'];mat=config['material'];settings=config['simulation']
    with np.load(output/'native-input.npz',allow_pickle=False) as d:v,flat,f,rest,pins=[d[k].copy() for k in ['initial_positions','flat_positions','faces','rest_triangles','pins']]
    contact=settings['ppf_parameters']
    if contact['image_id']!=IMAGE:raise ValueError('PPF configured image differs from native image')
    from frontend import App
    app=App.create('clothforge',cache_dir=str(output/'ppf-cache'));app.asset.add.tri('garment',v,f)
    scene=app.scene.create();obj=scene.add('garment')
    if len(pins):obj.pin(pins.tolist())
    obj.param.set('density',mat['mass_per_area']).set('young-mod',mat['tri_ke']/mat['mass_per_area']).set('bend',mat['edge_ke'])
    obj.param.set('friction',mat['friction']).set('contact-gap',contact['contact_gap_m']).set('strain-limit',contact['strain_limit'])
    obj.set_bend_rest_vert(flat)
    scene.add.invisible.wall([0,0,0],[0,0,1]).param.set('friction',contact['ground_friction'])
    fixed=scene.build()
    if not hasattr(fixed,'_concat_rest_vert'):raise RuntimeError('Pinned PPF rest-shape API changed')
    actual=np.asarray(fixed.vertex())
    order=np.linalg.norm(v[:,None,:]-actual[None,:,:],axis=2).argmin(1)
    if len(actual)!=len(v) or len(set(order))!=len(v) or not np.allclose(actual[order],v,atol=1e-6):raise ValueError('PPF vertex mapping failed')
    inverse=np.argsort(order)
    if sorted(map(tuple,np.sort(inverse[np.asarray(fixed._tri)],axis=1)))!=sorted(map(tuple,np.sort(f,axis=1))):raise ValueError('PPF changed topology')
    fixed._concat_rest_vert=flat[inverse].copy();fixed._rest_vert_mask=np.ones(len(v),dtype=np.uint8)
    # PPF's native mass builder uses assembled triangle areas, independently
    # of its rest-shape elastic metric. Compensate per face to preserve mass.
    nf=np.asarray(fixed._tri);ref=flat[inverse][nf];cur=actual[nf]
    area=lambda t:np.linalg.norm(np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0]),axis=1)/2
    ra,ca=area(ref),area(cur)
    if np.any(ca<=0):raise ValueError('PPF degenerate current triangle')
    fixed._tri_param['density']=(mat['mass_per_area']*ra/ca).tolist()
    actual_mass=float(np.dot(ca,fixed._tri_param['density']))
    session=app.session.create(fixed)
    frames=round(settings['duration_s']*settings['fps']);dt=1/settings['fps']/settings['substeps']
    session.param.set('frames',frames).set('fps',settings['fps']).set('dt',dt).set('gravity',[0.,0.,-9.81])
    session.param.set('air-density',0.).set('max-newton-steps',settings['iterations'])
    session=session.build();started=time.monotonic()
    try:session.start(blocking=True)
    finally:
        log=Path(session.info.path)/'stdout.log'
        if log.is_file():shutil.copyfile(log,output/'native-solver.log')
    if not session.finished():raise RuntimeError('PPF session did not finish')
    ps=[]
    for frame in range(frames+1):
        value=session.get.vertex(frame)
        if value is None or value[1]!=frame:raise ValueError('PPF missing frame '+str(frame))
        ps.append(value[0][order])
    ps=np.asarray(ps)
    if ps.shape!=(frames+1,len(v),3) or not np.allclose(ps[0],v,atol=1e-6):raise ValueError('PPF changed initial positions or vertex order')
    vs=np.concatenate([np.zeros_like(ps[:1]),np.diff(ps,axis=0)*settings['fps']],axis=0)
    np.savez_compressed(output/'trajectory.npz',positions=ps,velocities=vs,faces=f,rest_triangles=rest,pins=pins,times=np.arange(len(ps))/settings['fps'])
    from .quality import rest_matrices
    _,areas=rest_matrices(rest)
    write_json(output/'initialization.json',{'pins':pins.tolist(),'unconstrained_total_mass_kg':actual_mass,
        'mass_source':'native assembled areas times per-face density corrected by rest/current area ratio; runtime mass buffer not exported',
        'material_area_m2':float(areas.sum()),'material_rest_reference':'explicit per-face planar rest shape',
        'solver':'PPF contact solver','image_id':IMAGE,'image_manifest':MANIFEST,'physical_wall_seconds':time.monotonic()-started,
        'material_mapping':'young-mod=tri_ke/areal_density; numeric bend; 10 percent strain barrier; uniform seam bending; not calibrated equivalence',
        'native_parameters':{'dt':dt,'max_newton_steps':settings['iterations'],'contact_gap_m':contact['contact_gap_m'],'strain_limit':contact['strain_limit'],'ground_friction':contact['ground_friction'],'air_density':0.},'velocity_source':'saved-frame finite differences','gravity_m_s2':[0,0,-9.81],'ground_z_m':0,'resolved_simulation':settings})
    print('FRAME',frames,'/',frames,flush=True)

if __name__=='__main__':native(sys.argv[1])
