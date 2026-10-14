from copy import deepcopy
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import tempfile
import time
import unittest
from unittest.mock import patch

import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))

from calibration_fixtures import dataset
from piper_eye_to_hand_calibration.intrinsics import load_intrinsics
from piper_eye_to_hand_calibration.ros_support import matrix_to_transform_stamped
from piper_eye_to_hand_calibration.stereo import board_pose
from piper_eye_to_hand_calibration.artifacts import read_yaml

path=Path(__file__).resolve().parents[1]/'scripts/stereo_calibration_node.py'
spec=importlib.util.spec_from_file_location('stereo_capture_test',str(path));module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class CaptureTest(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.data,_=dataset(self.tmp.name)
        self.node=module.StereoCapture.__new__(module.StereoCapture)
        n=self.node;n.root=Path(self.tmp.name)/'session';n.root.mkdir()
        n.profiles={name:load_intrinsics(Path(self.tmp.name)/(name+'.yaml')) for name in module.CAMERAS}
        n.cameras=deepcopy(self.data['cameras']);n.driver_infos={};n.bridge=CvBridge()
        n.samples=[];n.frames=None;n.attempt=None;n.origins={};n.anchors={};n.views={};n.reason='';n.last_failure=''
        n.tf_buffer=SimpleNamespace(lookup_transform=lambda a,b,c,d:matrix_to_transform_stamped(
            np.array(n.cameras[a.split('_')[0]]['link_T_optical']),a,b,rospy.Time.from_sec(100)))
        for name in module.CAMERAS:
            points=np.array(self.data['samples'][0]['cameras'][name]['corners_px'])
            pose,_=board_pose(points,n.cameras[name]['camera_info'])
            n.origins[name]=dict(point_px=points[0].tolist(),click_px=points[0].tolist(),stamp_s=100.)
            n.anchors[name]=dict(corners_px=points,camera_info=n.cameras[name]['camera_info'],camera_T_board=pose)
            n.views[name]=dict(processed=time.monotonic(),raw_corners=points,stamp=100.)

    def messages(self,stamp):
        result=[]
        for name in module.CAMERAS:
            im=self.node.bridge.cv2_to_imgmsg(np.full((480,640,3),255,np.uint8),'bgr8')
            im.header.stamp=rospy.Time.from_sec(stamp);im.header.frame_id=name+'_color_optical_frame'
            info=CameraInfo();info.header=deepcopy(im.header);info.width=640;info.height=480;info.distortion_model='plumb_bob'
            info.K=self.node.cameras[name]['camera_info']['K'];info.D=[0.]*5
            result.extend([im,info])
        return result

    def test_thirty_unique_pairs_saved_then_selection_cleared(self):
        n=self.node;n.start_capture()
        attempt=n.attempt
        points=[n.anchors[name]['corners_px'] for name in module.CAMERAS]
        with patch.object(module,'detect_corners',side_effect=points*30), patch.object(module.rospy,'get_param',side_effect=lambda key,default: module.SERIALS[key.split('/')[1]]):
            for i in range(30):
                stamp=100.+i*.033
                with patch.object(module.rospy.Time,'now',return_value=rospy.Time.from_sec(stamp+.01)):
                    n.process(self.messages(stamp))
        self.assertEqual(len(n.samples),1);self.assertEqual(n.origins,{})
        self.assertEqual(len(list(attempt.glob('*.png'))),60)
        self.assertEqual(len(read_yaml(n.root/'dataset.yaml')['samples'][0]['frames']),30)

    def test_failure_keeps_evidence_and_requires_reselection(self):
        n=self.node;n.start_capture();attempt=n.attempt
        with patch.object(module.rospy,'logwarn'):
            n.reject('board moved')
        self.assertTrue((attempt/'failure.yaml').exists());self.assertEqual(n.origins,{})
        self.assertIsNone(n.frames)
        with self.assertRaises(ValueError):n.start_capture()

    def test_expired_click_and_noncorner_clear_selection(self):
        n=self.node
        n.views['rs1']['processed']=time.monotonic()-2
        with patch.object(module.rospy,'logwarn'):
            n.click(module.cv2.EVENT_LBUTTONDOWN,100,100,0,None)
        self.assertEqual(n.origins,{})

    def test_write_failure_still_invalidates_selection(self):
        n=self.node;n.start_capture()
        with patch.object(module,'write_yaml',side_effect=OSError('disk full')), patch.object(module.rospy,'logerr'), patch.object(module.rospy,'logwarn'):
            n.reject('image save failed')
        self.assertIsNone(n.frames);self.assertEqual(n.origins,{})
