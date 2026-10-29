import unittest
from clothforge.physx_worker import material_parameters

class MaterialUnits(unittest.TestCase):
    def test_membrane_and_areal_mass_survive_unit_conversion(self):
        m={'tri_ke':3000.,'tri_ka':3000.,'edge_ke':.005,'mass_per_area':.22}
        p=material_parameters(m);h=p['thickness_m'];nu=p['poisson']
        self.assertAlmostEqual(p['density_kg_m3']*h,m['mass_per_area'])
        self.assertAlmostEqual(p['young_pa']*h/(2*(1+nu)),m['tri_ke'])
        self.assertAlmostEqual(p['young_pa']*h*nu/(1-nu*nu),m['tri_ka'])
        self.assertAlmostEqual(p['bend_pa']*h**3,m['edge_ke'])

class MotionVerdicts(unittest.TestCase):
    def test_shape_pass_cannot_hide_slow_or_missing_freefall(self):
        from clothforge.physx_worker import motion_verdict
        for error in [.001,.08,None,float('nan')]:
            result={'outcome':'passed','failed_gates':[]}
            diag={'freefall':{'maximum_com_error_m':error,'time_end_s':.2}}
            checked=motion_verdict(result,diag,'drop',.005)
            self.assertEqual(checked['outcome'],'passed' if error==.001 else 'failed')
        result=motion_verdict({'outcome':'failed','failed_gates':['self_intersection']},{},'hanging',.005)
        self.assertEqual(result['failed_gates'],['self_intersection'])

    def test_motion_verdict_replaces_current_run_validation_without_create_conflict(self):
        import json,tempfile
        from pathlib import Path
        from unittest.mock import patch
        from clothforge.physx_worker import simulate
        with tempfile.TemporaryDirectory() as directory:
            out=Path(directory)
            original={'outcome':'failed','failed_gates':['self_intersection']}
            (out/'validation.json').write_text(json.dumps(original))
            (out/'diagnostics.json').write_text(json.dumps({'freefall':{'maximum_com_error_m':.08,'time_end_s':.2}}))
            config={'simulation':{'physx_parameters':{'motion_tolerance_m':.005}}}
            with patch('clothforge.native_io.load',return_value=({},config)), patch('clothforge.native_io.finish',return_value=original), patch('clothforge.physx_worker.subprocess.run'):
                result=simulate('unused','medium','drop',out)
            self.assertEqual(json.loads((out/'validation.json').read_text()),result)
            self.assertEqual(result['failed_gates'],['self_intersection','freefall_motion_mismatch'])
            self.assertFalse((out/'validation-motion.json').exists())
