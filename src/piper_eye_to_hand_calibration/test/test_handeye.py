import importlib.util
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import numpy as np
import rospy

sys.path.insert(0,str(Path(__file__).resolve().parent))
from calibration_fixtures import result_files,transform
from piper_eye_to_hand_calibration.artifacts import read_yaml,sha256
from piper_eye_to_hand_calibration.intrinsics import load_intrinsics
from piper_eye_to_hand_calibration.composition import compose_results

path=Path(__file__).resolve().parents[1]/'scripts/calibration_server_node.py'
spec=importlib.util.spec_from_file_location('handeye_capture_test',str(path));module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class HandeyeTest(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name)
        sp,_,base,_=result_files(self.root)
        n=module.CalibrationServer.__new__(module.CalibrationServer)
        self.n=n;n.stereo_path=str(sp);n.stereo=read_yaml(sp)
        n.stereo_binding=dict(path=str(sp),sha256=sha256(sp));n.intrinsics=load_intrinsics(self.root/'rs1.yaml')
        n.lock=threading.Lock();n.samples=[];n.min_samples=15;n.camera='rs1';n.serial=n.intrinsics['serial']
        n.result_dir=str(self.root/'handeye');n.session_dir=Path(n.result_dir)/'session';n.session_dir.mkdir(parents=True)
        n.base_frame='arm_base';n.camera_frame='rs1_color_optical_frame';n.camera_link_frame='rs1_link'
        n.marker_id=18;n.marker_size_m=.100;n.translation_rms_limit_m=.010;n.rotation_rms_limit_deg=1.5
        n.robot_pose_topic='/end_pose';n.image_topic='/rs1/color/image_raw';n.camera_info_topic='/rs1/color/camera_info';n.marker_pose_topic='marker_pose'
        n.max_pair_age_sec=.5;n.duplicate_translation_m=.01;n.duplicate_rotation_deg=5.
        internal=np.array(n.stereo['cameras']['rs1']['link_T_optical'])
        n._lookup_link_to_optical=lambda:internal
        mount=transform([.1,.2,.3],[.02,.03,.1]);random=np.random.RandomState(7)
        for _ in range(20):
            robot=transform(random.uniform(-1,1,3),random.uniform(-.3,.3,3))
            n.samples.append(dict(base_to_tool=robot,camera_to_target=np.linalg.inv(base)@robot@mount,robot_stamp=100.,marker_stamp=100.))

    def test_real_solver_output_composes(self):
        n=self.n
        with patch.object(module.rospy,'loginfo'),patch.object(module.rospy,'resolve_name',side_effect=lambda s:s):
            response=n.solve(None)
        self.assertTrue(response.success,response.message)
        outputs=[p for p in n.session_dir.glob('*.yaml')]
        self.assertEqual(len(outputs),1)
        result=read_yaml(outputs[0]);self.assertEqual(result['stereo_binding'],n.stereo_binding)
        compose_results(n.stereo_path,outputs[0],self.root/'composed')

    def test_binding_change_rejects_capture(self):
        (self.root/'rs1.yaml').write_text('changed')
        response=self.n.capture(None)
        self.assertFalse(response.success);self.assertIn('intrinsics file changed',response.message)

    def test_stale_source_pair_rejected_even_if_just_received(self):
        self.n.latest_pair=dict(received_monotonic=time.monotonic(),robot_stamp=10.,marker_stamp=10.)
        with patch.object(module.rospy.Time,'now',return_value=rospy.Time.from_sec(100)):
            response=self.n.capture(None)
        self.assertFalse(response.success);self.assertIn('source timestamps',response.message)
