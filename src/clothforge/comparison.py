"""Common geometry and explicit, uncalibrated cross-engine material mapping."""
import numpy as np
from .quality import rest_matrices

def initial_state(data,manifest,scenario):
    v=data['vertices'].copy();v[:,2]+=0.35-v[:,2].min()
    f=data['faces'];rest=data['rest_triangles']
    # Current pilot garments share planar x/z coordinates at sewn vertices.
    flat=np.column_stack([data['vertices'][:,0],np.zeros(len(v)),data['vertices'][:,2]])
    expected=np.linalg.norm(rest[:,:,None,:]-rest[:,None,:,:],axis=-1)
    got=np.linalg.norm(flat[f][:,:,None,:]-flat[f][:,None,:,:],axis=-1)
    if not np.allclose(expected,got,atol=1e-9):raise ValueError('garment cannot preserve per-face rest metric in this IPC adapter')
    pins=[]
    if scenario=='hanging':
        width=manifest['parameters'].get('width_m',manifest['parameters'].get('waist_width_m'))
        for x in [-width*.45,width*.45]: pins.extend(np.argsort(np.linalg.norm(v-[x,0,v[:,2].max()],axis=1))[:2].tolist())
    elif scenario!='drop':raise ValueError('unsupported scenario')
    return v,flat,sorted(set(pins))

def ipc_material(material):
    # Match membrane Lame parameters where possible; different energy models,
    # damping and contact laws remain explicitly different, not real calibration.
    mu=float(material['tri_ke']);lam=float(material['tri_ka']);h=.0006
    nu=lam/(2*mu+lam)
    E=2*mu*(1+nu)/h
    return {'young_pa':E,'poisson':nu,'thickness_m':h,'density_kg_m3':material['mass_per_area']/(2*h),
        'bending_stiffness':material['edge_ke'],'friction':material['friction'],
        'mapping':'nominal membrane scale and numeric bending preset; different energies, not calibrated equivalence',
        'unmapped':['tri_kd','edge_kd'],'calibration_status':'synthetic_unmeasured'}
