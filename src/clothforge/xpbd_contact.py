"""Surface contact projections for Newton XPBD; executed inside every time step.

Mass-weighted unilateral vertex/face and edge/edge constraints preserve the
previous-step side. This is a discrete proximity solver, not certified CCD.
"""
import numpy as np
import warp as wp
from newton._src.geometry.kernels import triangle_closest_point_barycentric


@wp.func
def project4(i: int, j: int, k: int, l: int, a: float, b: float, c: float, d: float,
             n: wp.vec3, violation: float, mass: wp.array(dtype=float),
             delta: wp.array(dtype=wp.vec3), count: wp.array(dtype=float)):
    denom = a*a*mass[i]+b*b*mass[j]+c*c*mass[k]+d*d*mass[l]
    if violation > 0.0 and denom > 0.0:
        impulse = n*(violation/denom)
        wp.atomic_add(delta,i,impulse*(a*mass[i]))
        wp.atomic_add(delta,j,impulse*(b*mass[j]))
        wp.atomic_add(delta,k,impulse*(c*mass[k]))
        wp.atomic_add(delta,l,impulse*(d*mass[l]))
        wp.atomic_add(count,i,1.0)
        wp.atomic_add(count,j,1.0)
        wp.atomic_add(count,k,1.0)
        wp.atomic_add(count,l,1.0)


@wp.kernel
def vertex_face(mesh: wp.uint64, faces: wp.array(dtype=wp.int32),
                old: wp.array(dtype=wp.vec3), q: wp.array(dtype=wp.vec3),
                mass: wp.array(dtype=float), adjacent: wp.array2d(dtype=wp.int32), gap: float, margin: float,
                delta: wp.array(dtype=wp.vec3), count: wp.array(dtype=float)):
    i=wp.tid()
    lo=wp.min(old[i],q[i])-wp.vec3(margin)
    hi=wp.max(old[i],q[i])+wp.vec3(margin)
    query=wp.mesh_query_aabb(mesh,lo,hi)
    face=int(0)
    while wp.mesh_query_aabb_next(query,face):
        j=faces[3*face]; k=faces[3*face+1]; l=faces[3*face+2]
        if i!=j and i!=k and i!=l and adjacent[i,j]==0 and adjacent[i,k]==0 and adjacent[i,l]==0:
            bary=triangle_closest_point_barycentric(q[j],q[k],q[l],q[i])
            diff=q[i]-q[j]*bary[0]-q[k]*bary[1]-q[l]*bary[2]
            prior=old[i]-old[j]*bary[0]-old[k]*bary[1]-old[l]*bary[2]
            distance=wp.length(diff)
            if distance<margin and wp.length(prior)>1.e-8:
                n=wp.normalize(prior)
                # Only near-surface candidates; the previous side prevents a
                # small step through a face from pushing to the wrong side.
                signed=wp.dot(diff,n)
                project4(i,j,k,l,1.0,-bary[0],-bary[1],-bary[2],n,gap-signed,mass,delta,count)


@wp.kernel
def edge_edge(mesh: wp.uint64, faces: wp.array(dtype=wp.int32), edges: wp.array(dtype=wp.vec2i), owners: wp.array(dtype=wp.int32),
              old: wp.array(dtype=wp.vec3), q: wp.array(dtype=wp.vec3),
              mass: wp.array(dtype=float), adjacent: wp.array2d(dtype=wp.int32), gap: float, margin: float,
              delta: wp.array(dtype=wp.vec3), count: wp.array(dtype=float)):
    edge_id=wp.tid();e=edges[edge_id];i=e[0];j=e[1]
    lo=wp.min(wp.min(old[i],old[j]),wp.min(q[i],q[j]))-wp.vec3(margin)
    hi=wp.max(wp.max(old[i],old[j]),wp.max(q[i],q[j]))+wp.vec3(margin)
    query=wp.mesh_query_aabb(mesh,lo,hi)
    face=int(0)
    while wp.mesh_query_aabb_next(query,face):
        for z in range(3):
            k=faces[3*face+z];l=faces[3*face+(z+1)%3]
            # One owner per edge includes boundary edges of either winding.
            if owners[3*face+z]>edge_id and i!=k and i!=l and j!=k and j!=l and adjacent[i,k]==0 and adjacent[i,l]==0 and adjacent[j,k]==0 and adjacent[j,l]==0:
                st=wp.closest_point_edge_edge(q[i],q[j],q[k],q[l],1.e-12)
                a=1.0-st[0];b=st[0];c=-(1.0-st[1]);d=-st[1]
                diff=a*q[i]+b*q[j]+c*q[k]+d*q[l]
                prior=a*old[i]+b*old[j]+c*old[k]+d*old[l]
                if wp.length(diff)<margin and wp.length(prior)>1.e-8:
                    n=wp.normalize(prior)
                    project4(i,j,k,l,a,b,c,d,n,gap-wp.dot(diff,n),mass,delta,count)


@wp.kernel
def strain(faces: wp.array(dtype=wp.int32), inverse: wp.array(dtype=wp.mat22),
           q: wp.array(dtype=wp.vec3), mass: wp.array(dtype=float), limit: float,
           delta: wp.array(dtype=wp.vec3), count: wp.array(dtype=float)):
    t=wp.tid();i=faces[3*t];j=faces[3*t+1];k=faces[3*t+2]
    inv=inverse[t];u=q[j]-q[i];v=q[k]-q[i]
    f0=u*inv[0,0]+v*inv[1,0];f1=u*inv[0,1]+v*inv[1,1]
    aa=wp.dot(f0,f0);bb=wp.dot(f0,f1);cc=wp.dot(f1,f1)
    eig=0.5*(aa+cc+wp.sqrt((aa-cc)*(aa-cc)+4.0*bb*bb))
    sigma=wp.sqrt(wp.max(eig,1.e-20))
    if sigma>limit:
        direction=wp.vec2(bb,eig-aa)
        if wp.length(direction)<1.e-10:
            direction=wp.vec2(1.0,0.0)
            if cc>aa:direction=wp.vec2(0.0,1.0)
        direction=wp.normalize(direction)
        n=(f0*direction[0]+f1*direction[1])/sigma
        g=inv*direction
        a=-(g[0]+g[1]);b=g[0];c=g[1]
        denom=mass[i]*a*a+mass[j]*b*b+mass[k]*c*c
        if denom>0.0:
            corr=n*((limit-sigma)/denom)
            wp.atomic_add(delta,i,corr*(mass[i]*a));wp.atomic_add(count,i,1.0)
            wp.atomic_add(delta,j,corr*(mass[j]*b));wp.atomic_add(count,j,1.0)
            wp.atomic_add(delta,k,corr*(mass[k]*c));wp.atomic_add(count,k,1.0)


@wp.kernel
def area_elasticity(faces: wp.array(dtype=wp.int32), inverse: wp.array(dtype=wp.mat22),
                    q: wp.array(dtype=wp.vec3), mass: wp.array(dtype=float), stiffness: float, dt: float,
                    delta: wp.array(dtype=wp.vec3), count: wp.array(dtype=float)):
    t=wp.tid();i=faces[3*t];j=faces[3*t+1];k=faces[3*t+2]
    u=q[j]-q[i];v=q[k]-q[i];cross=wp.cross(u,v);length=wp.length(cross)
    if length>1.e-12:
        rest_area=0.5/wp.abs(wp.determinant(inverse[t]))
        n=cross/length
        gj=wp.cross(v,n)/(2.0*rest_area);gk=wp.cross(n,u)/(2.0*rest_area);gi=-gj-gk
        denom=mass[i]*wp.dot(gi,gi)+mass[j]*wp.dot(gj,gj)+mass[k]*wp.dot(gk,gk)
        alpha=1.0/(stiffness*rest_area*dt*dt)
        impulse=-(0.5*length/rest_area-1.0)/(denom+alpha)
        wp.atomic_add(delta,i,gi*(mass[i]*impulse));wp.atomic_add(count,i,1.0)
        wp.atomic_add(delta,j,gj*(mass[j]*impulse));wp.atomic_add(count,j,1.0)
        wp.atomic_add(delta,k,gk*(mass[k]*impulse));wp.atomic_add(count,k,1.0)


@wp.kernel
def count_max(count: wp.array(dtype=float), maximum: wp.array(dtype=float)):
    wp.atomic_max(maximum,0,count[wp.tid()])


@wp.kernel
def apply(q: wp.array(dtype=wp.vec3), delta: wp.array(dtype=wp.vec3), count: wp.array(dtype=float),
          maximum: wp.array(dtype=float), total: wp.array(dtype=wp.vec3), symmetric: bool):
    i=wp.tid()
    if count[i]>0.0:
        divisor=count[i]
        if symmetric:divisor=maximum[0]
        correction=delta[i]/divisor
        q[i]=q[i]+correction
        total[i]=total[i]+correction


@wp.kernel
def velocity(old: wp.array(dtype=wp.vec3), q: wp.array(dtype=wp.vec3),
             v: wp.array(dtype=wp.vec3), total: wp.array(dtype=wp.vec3), dt: float, symmetric: bool):
    i=wp.tid()
    correction=q[i]-old[i]
    if symmetric:correction=total[i]
    v[i]=v[i]+correction/dt


class SurfaceContact:
    def __init__(self,model,faces,inverse,parameters,area_stiffness=0.):
        self.model=model;self.parameters=parameters;self.area_stiffness=area_stiffness
        self.faces=wp.array(np.asarray(faces,dtype=np.int32).reshape(-1),dtype=wp.int32,device=model.device)
        edges=sorted({tuple(sorted((int(t[a]),int(t[b])))) for t in faces for a,b in [(0,1),(1,2),(2,0)]})
        self.edges=wp.array(edges,dtype=wp.vec2i,device=model.device)
        edge_ids={e:i for i,e in enumerate(edges)};seen=set();owners=[]
        for t in faces:
            for a,b in [(0,1),(1,2),(2,0)]:
                e=tuple(sorted((int(t[a]),int(t[b]))))
                owners.append(-1 if e in seen else edge_ids[e]);seen.add(e)
        self.owners=wp.array(owners,dtype=wp.int32,device=model.device)
        adjacency=np.zeros((model.particle_count,model.particle_count),dtype=np.int32)
        if parameters.get('topological_exclusion_hops',0)==1:
            for a,b in edges:adjacency[a,b]=adjacency[b,a]=1
        self.adjacent=wp.array(adjacency,dtype=wp.int32,device=model.device)
        self.maximum=wp.zeros(1,dtype=float,device=model.device)
        self.total=wp.zeros(model.particle_count,dtype=wp.vec3,device=model.device)
        self.inverse=wp.array(inverse,dtype=wp.mat22,device=model.device)
        self.delta=wp.zeros(model.particle_count,dtype=wp.vec3,device=model.device)
        self.count=wp.zeros(model.particle_count,dtype=float,device=model.device)
        self.mesh=None
        self.predicted=wp.empty(model.particle_count,dtype=wp.vec3,device=model.device)

    def step(self,old,state,dt):
        if self.mesh is None or self.mesh.points.ptr!=state.particle_q.ptr:
            # Maintain one BVH per ping-pong state, without allocating per step.
            if not hasattr(self,'meshes'):self.meshes={}
            key=state.particle_q.ptr
            if key not in self.meshes:self.meshes[key]=wp.Mesh(points=state.particle_q,indices=self.faces)
            self.mesh=self.meshes[key]
        old_q=old.particle_q_init if hasattr(old,"particle_q_init") else old.particle_q
        p=self.parameters;mass=self.model.particle_inv_mass;q=state.particle_q
        wp.copy(self.predicted,q);self.total.zero_()
        symmetric=p.get("symmetric_projection",False)
        for _ in range(p['iterations']):
            self.mesh.refit();self.delta.zero_();self.count.zero_()
            wp.launch(vertex_face,self.model.particle_count,[self.mesh.id,self.faces,old_q,q,mass,self.adjacent,p['gap_m'],p['margin_m'],self.delta,self.count],device=self.model.device)
            wp.launch(edge_edge,len(self.edges),[self.mesh.id,self.faces,self.edges,self.owners,old_q,q,mass,self.adjacent,p['gap_m'],p['margin_m'],self.delta,self.count],device=self.model.device)
            wp.launch(strain,len(self.inverse),[self.faces,self.inverse,q,mass,p['maximum_stretch'],self.delta,self.count],device=self.model.device)
            if self.area_stiffness>0.:
                wp.launch(area_elasticity,len(self.inverse),[self.faces,self.inverse,q,mass,self.area_stiffness,dt,self.delta,self.count],device=self.model.device)
            self.maximum.zero_()
            wp.launch(count_max,self.model.particle_count,[self.count,self.maximum],device=self.model.device)
            wp.launch(apply,self.model.particle_count,[q,self.delta,self.count,self.maximum,self.total,symmetric],device=self.model.device)
        wp.launch(velocity,self.model.particle_count,[self.predicted,q,state.particle_qd,self.total,dt,symmetric],device=self.model.device)


@wp.kernel
def apply_native_delta(q: wp.array(dtype=wp.vec3), v: wp.array(dtype=wp.vec3),
                       delta: wp.array(dtype=wp.vec3), flags: wp.array(dtype=wp.int32),
                       dt: float, q_out: wp.array(dtype=wp.vec3), v_out: wp.array(dtype=wp.vec3)):
    i=wp.tid()
    if (flags[i] & 1)!=0:
        q_out[i]=q[i]+delta[i]
        # Preserve sub-ULP integration velocity at fine dt. Reconstructing from
        # world-space float32 positions creates spurious acceleration or sleep.
        v_out[i]=v[i]+delta[i]/dt


from newton.solvers import SolverXPBD


class SurfaceXPBD(SolverXPBD):
    """Native XPBD constraints with an incremental float32 velocity update."""
    def _apply_particle_deltas(self,model,state_in,state_out,particle_deltas,dt):
        if state_in.requires_grad:
            raise ValueError('cloth surface XPBD adapter does not support autodiff')
        if self._particle_delta_counter==0:
            current,target=state_out,state_in
        else:
            current,target=state_in,state_out
        self._particle_delta_counter=1-self._particle_delta_counter
        wp.launch(apply_native_delta,model.particle_count,
                  [current.particle_q,current.particle_qd,particle_deltas,model.particle_flags,dt,target.particle_q,target.particle_qd],device=model.device)
        return target.particle_q,target.particle_qd
