from pathlib import Path
import tempfile
import unittest
import numpy as np
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))

from calibration_fixtures import result_files
from piper_eye_to_hand_calibration.artifacts import read_yaml,write_yaml
from piper_eye_to_hand_calibration.composition import compose_results
from piper_eye_to_hand_calibration.intrinsics import load_intrinsics
from piper_eye_to_hand_calibration.stereo import bind_handeye


class CompositionTest(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name)
        self.sp,self.hp,self.base,self.relative=result_files(self.root)

    def test_matrix_direction_and_optical_link(self):
        result=compose_results(self.sp,self.hp,self.root/'out')
        for name,expected in [('rs1',self.base),('rs3',self.base@self.relative)]:
            item=result[name]
            np.testing.assert_allclose(item['calibration']['base_to_camera_optical']['matrix_4x4'],expected,atol=1e-5)
            link=np.array(item['tf_publication']['base_to_camera_link']['matrix_4x4'])
            internal=np.array(item['tf_publication']['camera_link_to_optical']['matrix_4x4'])
            np.testing.assert_allclose(link@internal,expected,atol=1e-5)
            inverse=item['calibration']['camera_optical_to_base']['matrix_4x4']
            np.testing.assert_allclose(expected@inverse,np.eye(4),atol=1e-5)
            self.assertFalse(item['automatic_tf_publication'])
        with self.assertRaises(FileExistsError):compose_results(self.sp,self.hp,self.root/'out')

    def test_handeye_failure_hash_frame_identity_rejected(self):
        original=read_yaml(self.hp)
        for key in ['quality','hash','frame','identity','intrinsics','link']:
            d=read_yaml(self.hp)
            if key=='quality': d['quality']['translation_rms_m']=.02
            elif key=='hash': d['stereo_binding']['sha256']='0'*64
            elif key=='frame': d['calibration']['base_to_camera_optical']['parent_frame']='base_link'
            elif key=='identity': d['camera']['serial']='wrong'
            elif key=='intrinsics': d['intrinsics']['image_height']=360
            elif key=='link': d['tf_publication']['camera_link_to_optical']['matrix_4x4'][0][3]+=.01
            write_yaml(self.hp,d)
            with self.subTest(key=key), self.assertRaises(ValueError): compose_results(self.sp,self.hp,self.root/'bad')
            write_yaml(self.hp,original)

    def test_binding_requires_exact_intrinsics(self):
        p=load_intrinsics(self.root/'rs1.yaml')
        bind_handeye(self.sp,p)
        p['sha256']='0'*64
        with self.assertRaises(ValueError):bind_handeye(self.sp,p)
