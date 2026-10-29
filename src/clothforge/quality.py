"""Mesh and trajectory measurements; thresholds come from the frozen pilot."""
import numpy as np
from scipy.spatial import cKDTree

def triangle_areas(vertices, faces):
    tri = np.asarray(vertices)[np.asarray(faces)]
    return np.linalg.norm(np.cross(tri[:,1]-tri[:,0], tri[:,2]-tri[:,0]), axis=1)*0.5

def edge_table(faces):
    edges = np.sort(np.concatenate([faces[:,[0,1]], faces[:,[1,2]], faces[:,[2,0]]]), axis=1)
    return np.unique(edges, axis=0, return_counts=True)

def mesh_metrics(vertices, faces):
    v, f = np.asarray(vertices), np.asarray(faces)
    if v.ndim != 2 or v.shape[1] != 3 or f.ndim != 2 or f.shape[1] != 3:
        raise ValueError("expected Nx3 vertices and Mx3 triangle indices")
    if not np.isfinite(v).all() or f.min() < 0 or f.max() >= len(v):
        raise ValueError("nonfinite vertices or invalid face indices")
    area = triangle_areas(v, f)
    edges, counts = edge_table(f)
    return {"vertices":len(v), "triangles":len(f), "area_m2":float(area.sum()),
            "minimum_triangle_area_m2":float(area.min()),
            "degenerate_triangles":int((area <= 1e-12).sum()),
            "nonmanifold_edges":int((counts>2).sum()), "boundary_edges":int((counts==1).sum()),
            "duplicate_faces":int(len(f)-len(np.unique(np.sort(f, axis=1), axis=0))),
            "aabb_size_m":np.ptp(v, axis=0).tolist()}

def rest_matrices(rest_triangles):
    p = np.asarray(rest_triangles)
    a, b = p[:,1]-p[:,0], p[:,2]-p[:,0]
    al = np.linalg.norm(a,axis=1)
    e = a/al[:,None]
    bx = (b*e).sum(axis=1)
    by = np.sqrt(np.maximum((b*b).sum(axis=1)-bx*bx,0))
    if np.any(al<=1e-12) or np.any(by<=1e-12):
        raise ValueError("degenerate rest triangles")
    dm = np.zeros((len(p),2,2))
    dm[:,0,0],dm[:,0,1],dm[:,1,1] = al,bx,by
    return np.linalg.inv(dm), al*by*0.5

def strain_metrics(vertices, faces, rest_inverse):
    p=np.asarray(vertices)[faces]
    ds=np.stack([p[:,1]-p[:,0],p[:,2]-p[:,0]],axis=2)
    singular=np.linalg.svd(ds@rest_inverse,compute_uv=False)
    return {"maximum_principal_stretch":float(singular.max()),
            "minimum_principal_stretch":float(singular.min()),
            "minimum_area_ratio":float(singular.prod(axis=1).min()),
            "maximum_area_ratio":float(singular.prod(axis=1).max())}

def self_intersections(vertices, faces):
    """Proper non-adjacent edge/triangle intersections, including coplanar overlap.
    Shared-vertex triangle pairs are excluded; this is a sampled surface check,
    not continuous collision detection or a minimum-thickness certificate.
    """
    tri=np.asarray(vertices)[faces]
    centers=tri.mean(axis=1)
    radii=np.linalg.norm(tri-centers[:,None],axis=2).max(axis=1)
    pairs=np.asarray(list(cKDTree(centers).query_pairs(float(2*radii.max()))),dtype=int).reshape(-1,2)
    if not len(pairs): return 0
    a,b=pairs[:,0],pairs[:,1]
    lo,hi=tri.min(axis=1),tri.max(axis=1)
    keep=np.all(lo[a]<=hi[b]+1e-9,axis=1)&np.all(lo[b]<=hi[a]+1e-9,axis=1)
    keep &= ~(faces[a,:,None]==faces[b,None,:]).any(axis=(1,2))
    pairs=pairs[keep]
    if not len(pairs): return 0
    a,b=pairs[:,0],pairs[:,1]
    hit=np.zeros(len(a),dtype=bool)
    eps=1e-9
    for source,target in [(tri[a],tri[b]),(tri[b],tri[a])]:
        e1,e2=target[:,1]-target[:,0],target[:,2]-target[:,0]
        for i,j in [(0,1),(1,2),(2,0)]:
            start=source[:,i]; direction=source[:,j]-start
            h=np.cross(direction,e2); det=(e1*h).sum(axis=1)
            valid=np.abs(det)>eps
            inv=np.divide(1.,det,out=np.zeros_like(det),where=valid)
            s=start-target[:,0];u=inv*(s*h).sum(axis=1)
            q=np.cross(s,e1); vv=inv*(direction*q).sum(axis=1); t=inv*(e2*q).sum(axis=1)
            hit |= valid&(u>=-eps)&(vv>=-eps)&(u+vv<=1+eps)&(t>=-eps)&(t<=1+eps)
    # Coplanar, disjoint-index triangles: separating-axis test in their plane.
    na=np.cross(tri[a,1]-tri[a,0],tri[a,2]-tri[a,0])
    nb=np.cross(tri[b,1]-tri[b,0],tri[b,2]-tri[b,0])
    na/=np.linalg.norm(na,axis=1)[:,None]
    nb/=np.linalg.norm(nb,axis=1)[:,None]
    planar=(np.linalg.norm(np.cross(na,nb),axis=1)<1e-7)&(np.abs(((tri[b,0]-tri[a,0])*na).sum(axis=1))<1e-8)
    overlap=planar.copy()
    for source in [tri[a],tri[b]]:
        for i,j in [(0,1),(1,2),(2,0)]:
            axis=np.cross(na,source[:,j]-source[:,i])
            pa=(tri[a]*axis[:,None]).sum(axis=2)
            pb=(tri[b]*axis[:,None]).sum(axis=2)
            overlap &= (pa.max(axis=1)>=pb.min(axis=1)-eps)&(pb.max(axis=1)>=pa.min(axis=1)-eps)
    return int((hit|overlap).sum())

def evaluate_trajectory(positions, velocities, faces, rest_triangles, gates):
    finite=bool(np.isfinite(positions).all() and np.isfinite(velocities).all())
    result={"state_finite":finite,"saved_frames":len(positions),"self_contact_check":"all_saved_frames_nonadjacent_triangle_intersections"}
    if not finite:
        return {"outcome":"failed","metrics":result,"failed_gates":["nonfinite_state"]}
    inv,_=rest_matrices(rest_triangles)
    strains=[strain_metrics(v,faces,inv) for v in positions]
    result.update({
        "maximum_principal_stretch":max(s["maximum_principal_stretch"] for s in strains),
        "minimum_area_ratio":min(s["minimum_area_ratio"] for s in strains),
        "maximum_area_ratio":max(s["maximum_area_ratio"] for s in strains),
        "maximum_ground_penetration_m":float(max(0.,-positions[:,:,2].min())),
        "maximum_speed_m_s":float(np.linalg.norm(velocities,axis=2).max()),
        "maximum_self_intersection_pairs":max(self_intersections(v,faces) for v in positions),
        "seam_gap_m":0.0,
        "seam_representation":"shared_vertex_topology",
    })
    checks={
        "excessive_stretch":result["maximum_principal_stretch"]>gates["maximum_principal_stretch"],
        "area_collapse":result["minimum_area_ratio"]<gates["minimum_area_ratio"],
        "area_expansion":result["maximum_area_ratio"]>gates["maximum_area_ratio"],
        "ground_penetration":result["maximum_ground_penetration_m"]>gates["maximum_ground_penetration_m"],
        "excessive_speed":result["maximum_speed_m_s"]>gates["maximum_speed_m_s"],
        "self_intersection":result["maximum_self_intersection_pairs"]>gates["maximum_self_intersection_pairs"],
    }
    failed=[k for k,v in checks.items() if v]
    return {"outcome":"failed" if failed else "passed","metrics":result,"failed_gates":failed}
