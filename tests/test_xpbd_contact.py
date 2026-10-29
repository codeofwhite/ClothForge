"""Small mechanical regressions independent of the garment acceptance suite."""
import unittest
from types import SimpleNamespace
import numpy as np
try:
    import warp as wp
    from clothforge.xpbd_contact import SurfaceContact
except ModuleNotFoundError as error:
    raise unittest.SkipTest('native Newton environment required: '+str(error))
from clothforge.quality import rest_matrices


class ContactTests(unittest.TestCase):
    def make(self,old,new,faces,weights=None,velocities=None):
        old=np.asarray(old,dtype=np.float32);new=np.asarray(new,dtype=np.float32)
        faces=np.asarray(faces,dtype=np.int32)
        weights=np.ones(len(old)) if weights is None else weights
        model=SimpleNamespace(device='cpu',particle_count=len(old),particle_inv_mass=wp.array(weights,dtype=float,device='cpu'))
        inv,_=rest_matrices(old[faces])
        p={'gap_m':.001,'margin_m':.02,'iterations':12,'maximum_stretch':1.10,'symmetric_projection':True,'topological_exclusion_hops':1}
        contact=SurfaceContact(model,faces,inv,p)
        s0=SimpleNamespace(particle_q=wp.array(old,dtype=wp.vec3,device='cpu'))
        s1=SimpleNamespace(particle_q=wp.array(new,dtype=wp.vec3,device='cpu'),particle_qd=wp.array(np.zeros_like(old) if velocities is None else velocities,dtype=wp.vec3,device='cpu'))
        return contact,s0,s1

    def test_vertex_crossing_fixed_triangle(self):
        old=[[-1,-1,0],[1,-1,0],[0,1,0],[0,0,.005]]
        new=np.array(old);new[3,2]=-.002
        c,a,b=self.make(old,new,[[0,1,2]],weights=[0,0,0,1])
        c.step(a,b,.01);q=b.particle_q.numpy()
        self.assertGreaterEqual(q[3,2],.00099)
        np.testing.assert_array_equal(q[:3],np.array(old,dtype=np.float32)[:3])

    def test_free_translation_preserves_native_velocity(self):
        old=np.array([[0,0,1],[1,0,1],[0,1,1]],dtype=np.float32)
        new=old.copy();new[:,2]-=1.e-8  # position rounds away; velocity must not.
        v=np.tile([0,0,-.0006],(3,1))
        c,a,b=self.make(old,new,[[0,1,2]],velocities=v)
        c.step(a,b,1/15360)
        np.testing.assert_allclose(b.particle_qd.numpy(),v,rtol=1.e-6)
        np.testing.assert_array_equal(b.particle_q.numpy(),new)

    def test_edge_edge_projection_does_not_need_a_vertex_inside_a_face(self):
        from clothforge.xpbd_contact import edge_edge,apply,count_max
        old=np.array([[-1,0,.005],[1,0,.005],[0,-2,.005],
                      [0,-1,0],[0,1,0],[2,0,0]],dtype=np.float32)
        new=old.copy();new[:3,2]=-.002
        c,a,b=self.make(old,new,[[0,1,2],[3,4,5]],weights=[1,1,1,0,0,0])
        mesh=wp.Mesh(points=b.particle_q,indices=c.faces)
        for _ in range(12):
            mesh.refit();c.delta.zero_();c.count.zero_()
            wp.launch(edge_edge,len(c.edges),[mesh.id,c.faces,c.edges,c.owners,a.particle_q,b.particle_q,c.model.particle_inv_mass,c.adjacent,.001,.02,c.delta,c.count],device='cpu')
            c.maximum.zero_();wp.launch(count_max,6,[c.count,c.maximum],device='cpu')
            wp.launch(apply,6,[b.particle_q,c.delta,c.count,c.maximum,c.total,True],device='cpu')
        q=b.particle_q.numpy()
        self.assertGreater(float((q[0,2]+q[1,2])*.5),.0009)
        np.testing.assert_array_equal(q[3:],old[3:])

    def test_stretch_constraint_reduces_strain(self):
        old=np.array([[0,0,0],[1,0,0],[0,1,0]],dtype=np.float32)
        new=old*1.3
        c,a,b=self.make(old,new,[[0,1,2]])
        c.step(a,b,.01)
        q=b.particle_q.numpy()
        self.assertLess(np.linalg.norm(q[1]-q[0]),1.15)
        np.testing.assert_allclose(q.mean(axis=0),new.mean(axis=0),atol=1.e-6)

    def test_material_area_stiffness_resists_compression_without_translation_force(self):
        old=np.array([[0,0,0],[1,0,0],[0,1,0]],dtype=np.float32)
        new=old*.5
        c,a,b=self.make(old,new,[[0,1,2]])
        c.area_stiffness=3000.
        c.step(a,b,.01)
        q=b.particle_q.numpy()
        self.assertGreater(np.linalg.norm(np.cross(q[1]-q[0],q[2]-q[0])),.8)
        np.testing.assert_allclose(q.mean(axis=0),new.mean(axis=0),atol=1.e-6)

    def test_shared_triangle_constraints_preserve_center_of_mass_and_momentum(self):
        old=np.array([[0,0,0],[1,0,0],[0,1,0],[1,1,0],[2,0,0]],dtype=np.float32)
        new=old.copy();new[4,0]=1.5;new[0,1]=-.2
        c,a,b=self.make(old,new,[[0,1,2],[1,3,2],[1,4,3]])
        c.area_stiffness=3000.
        c.step(a,b,.01)
        np.testing.assert_allclose(b.particle_q.numpy().mean(axis=0),new.mean(axis=0),atol=1.e-6)
        np.testing.assert_allclose(b.particle_qd.numpy().sum(axis=0),[0,0,0],atol=1.e-5)

    def test_native_substep_gravity_is_not_reconstructed_from_rounded_positions(self):
        import newton
        from clothforge.xpbd_contact import SurfaceXPBD
        builder=newton.ModelBuilder()
        builder.add_particle(pos=wp.vec3(0,0,2),vel=wp.vec3(0,0,0),mass=1.)
        model=builder.finalize(device='cpu')
        solver=SurfaceXPBD(model,iterations=2)
        a,b=model.state(),model.state();dt=1/15360
        for _ in range(512):
            a.clear_forces();solver.step(a,b,model.control(),None,dt);a,b=b,a
        self.assertAlmostEqual(float(a.particle_qd.numpy()[0,2]),-9.81/30,places=4)
        self.assertAlmostEqual(float(a.particle_q.numpy()[0,2]),2-.5*9.81*(1/30)**2,places=4)

    def test_boundary_edges_have_owners_for_both_windings(self):
        old=[[0,0,0],[1,0,0],[0,1,0],[2,0,0],[3,0,0],[2,1,0]]
        c,_,_=self.make(old,old,[[0,1,2],[5,4,3]])
        self.assertEqual(sorted(c.owners.numpy().tolist()),list(range(6)))


if __name__=='__main__':unittest.main()
