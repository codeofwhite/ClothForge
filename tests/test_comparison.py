import unittest
import numpy as np
from clothforge.comparison import initial_state,ipc_material

class ComparisonContracts(unittest.TestCase):
    def test_per_face_metric_is_preserved_and_incompatible_assets_rejected(self):
        vertices=np.array([[0.,0,0],[1.,.1,0],[0.,0,1]])
        data={'vertices':vertices,'faces':np.array([[0,1,2]]),'rest_triangles':np.array([[[0.,0,0],[1.,0,0],[0.,1,0]]])}
        v,flat,pins=initial_state(data,{'parameters':{'width_m':1}},'drop')
        self.assertAlmostEqual(v[:,2].min(),.35);self.assertEqual(pins,[])
        data['rest_triangles'][0,1,0]=2
        with self.assertRaises(ValueError):initial_state(data,{'parameters':{'width_m':1}},'drop')
    def test_ipc_density_accounts_for_native_half_thickness(self):
        m=ipc_material({'tri_ke':3000,'tri_ka':3000,'mass_per_area':.22,'edge_ke':.005,'friction':.5})
        self.assertAlmostEqual(m['density_kg_m3']*2*m['thickness_m'],.22)
        self.assertIn('tri_kd',m['unmapped'])
