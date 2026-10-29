"""Independent invariants and deliberately broken geometry, not snapshot tests."""
import unittest
import tempfile
from pathlib import Path
import numpy as np
from clothforge.quality import rest_matrices,strain_metrics,self_intersections,mesh_metrics,evaluate_trajectory
from clothforge.schema import validate_material,write_json,canonical_hash
from clothforge.generate import make_pattern,sample_parameters
from clothforge.mesh import build_mesh

class PhysicalContracts(unittest.TestCase):
    def setUp(self):
        self.v=np.array([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]])
        self.f=np.array([[0,1,2]])
        self.inv,_=rest_matrices(self.v[self.f])
    def test_rigid_motion_has_unit_stretch(self):
        rotation=np.array([[0,-1,0],[1,0,0],[0,0,1]])
        q=self.v@rotation.T+np.array([4,5,6])
        self.assertAlmostEqual(strain_metrics(q,self.f,self.inv)["maximum_principal_stretch"],1.)
    def test_double_length_is_not_stress_free(self):
        q=self.v.copy();q[:,0]*=2
        self.assertAlmostEqual(strain_metrics(q,self.f,self.inv)["maximum_principal_stretch"],2.)
    def test_intersecting_disjoint_triangles_detected(self):
        v=np.vstack([self.v,[[.25,.25,-1],[.25,.25,1],[.8,.2,0]]])
        self.assertEqual(self_intersections(v,np.array([[0,1,2],[3,4,5]])),1)
    def test_coplanar_overlap_detected(self):
        v=np.vstack([self.v,self.v*.25+np.array([.1,.1,0])])
        self.assertEqual(self_intersections(v,np.array([[0,1,2],[3,4,5]])),1)
    def test_separated_triangles_not_intersections(self):
        v=np.vstack([self.v,self.v+np.array([0,0,.05])])
        self.assertEqual(self_intersections(v,np.array([[0,1,2],[3,4,5]])),0)
    def test_shared_edge_is_valid(self):
        v=np.vstack([self.v,[[1,1,0]]])
        self.assertEqual(self_intersections(v,np.array([[0,1,2],[1,3,2]])),0)
    def test_bad_mass_units_rejected(self):
        with self.assertRaises(ValueError):validate_material({"calibration_status":"synthetic_unmeasured","mass_per_area_unit":"kg/m3"})
    def test_measured_label_cannot_be_invented(self):
        with self.assertRaises(ValueError):validate_material({"calibration_status":"calibrated"})
    def test_immutable_artifact(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/"x.json";write_json(p,{"a":1})
            with self.assertRaises(FileExistsError):write_json(p,{"a":2})
    def test_hash_independent_of_key_order(self):
        self.assertEqual(canonical_hash({"a":1,"b":2}),canonical_hash({"b":2,"a":1}))

class GarmentTopology(unittest.TestCase):
    def test_three_garments_have_expected_openings_and_area(self):
        for category,count in [("tshirt",4),("trousers",3),("open_coat",3)]:
            params=sample_parameters(category,np.random.default_rng(20260917))
            pattern=make_pattern(category,params)
            v,f,rest,_,_,mapping,m=build_mesh(pattern,.05,.012)
            self.assertEqual(m["boundary_loops"],count)
            self.assertEqual(m["nonmanifold_edges"],0)
            self.assertEqual(m["initial_self_intersection_pairs"],0)
            self.assertGreater(m["area_m2"],.2)
            inv,area=rest_matrices(rest)
            self.assertTrue((area>0).all())
            self.assertLess(strain_metrics(v,f,inv)["maximum_principal_stretch"],1.15)
    def test_generation_parameters_are_seeded(self):
        self.assertEqual(sample_parameters("tshirt",np.random.default_rng(7)),sample_parameters("tshirt",np.random.default_rng(7)))
        self.assertNotEqual(sample_parameters("tshirt",np.random.default_rng(7)),sample_parameters("tshirt",np.random.default_rng(8)))

if __name__=="__main__":unittest.main()
