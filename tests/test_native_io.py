import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from clothforge.native_io import finish

class NativeVerdicts(unittest.TestCase):
    def test_completed_worker_cannot_hide_missing_frames_or_fixture_drift(self):
        for mode,gate in [('complete',None),('short','incomplete_horizon'),('moving_pin','fixture_drift'),('nonfinite','nonfinite_state')]:
            with self.subTest(mode=mode),tempfile.TemporaryDirectory() as folder:
                output=Path(folder)
                rest=np.array([[[0.,0.,0.],[.1,0.,0.],[0.,.1,0.]]]);faces=np.array([[0,1,2]])
                frames=2 if mode=='short' else 3
                positions=np.repeat(rest,frames,axis=0)+[0,0,1]
                if mode=='moving_pin':positions[1:]+=[.01,0,0]
                if mode=='nonfinite':positions[1,1,0]=np.nan
                config={'simulation':{'duration_s':2/30,'fps':30,'substeps':8},'material':{'mass_per_area':.22},
                    'gates':{'maximum_principal_stretch':1.15,'minimum_area_ratio':.5,'maximum_area_ratio':1.3,
                             'maximum_ground_penetration_m':.002,'maximum_speed_m_s':10.,'maximum_self_intersection_pairs':0}}
                np.savez(output/'trajectory.npz',positions=positions,velocities=np.zeros_like(positions),faces=faces,rest_triangles=rest,times=np.arange(frames)/30,pins=np.array([0]))
                np.savez(output/'native-input.npz',face_panels=np.array([0]))
                (output/'initialization.json').write_text(json.dumps({'physical_wall_seconds':1.}))
                with patch('clothforge.mesh.save_preview'),patch('clothforge.mesh.save_glb'),patch('clothforge.mesh.save_viewer'):
                    result=finish(output,{'id':'test'},config,'medium','hanging','ppf')
                if gate:self.assertEqual(result['outcome'],'failed');self.assertIn(gate,result['failed_gates'])
                else:self.assertEqual(result['outcome'],'passed')
                self.assertFalse(result['real_world_validated'])
