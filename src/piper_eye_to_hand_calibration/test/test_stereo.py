from copy import deepcopy
from pathlib import Path
import tempfile
import unittest
import cv2
import numpy as np

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))

from calibration_fixtures import dataset
from piper_eye_to_hand_calibration.artifacts import write_yaml
from piper_eye_to_hand_calibration.stereo import (
    ordered_corners, solve_dataset, load_stereo, check_frame_stamps, check_stationary, detect_corners)


class StereoTest(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.data,self.truth=dataset(self.tmp.name)

    def test_known_transform_and_inverse(self):
        r=solve_dataset(self.data)
        self.assertTrue(r['validation']['passed'])
        np.testing.assert_allclose(r['rs1_optical_T_rs3_optical'],self.truth,atol=1e-5)
        np.testing.assert_allclose(np.array(r['rs1_optical_T_rs3_optical'])@r['rs3_optical_T_rs1_optical'],np.eye(4),atol=1e-9)

    def test_holdout_not_used_for_fit(self):
        before=solve_dataset(self.data)
        p=np.array(self.data['samples'][5]['cameras']['rs3']['corners_px']);p[:,0]+=12
        self.data['samples'][5]['cameras']['rs3']['corners_px']=p.tolist()
        after=solve_dataset(self.data)
        np.testing.assert_array_equal(before['rs1_optical_T_rs3_optical'],after['rs1_optical_T_rs3_optical'])
        self.assertFalse(after['validation']['passed'])
        path=Path(self.tmp.name)/'bad.yaml';write_yaml(path,after)
        with self.assertRaises(ValueError): load_stereo(path)

    def test_split_and_wrong_origin_rejected(self):
        bad=deepcopy(self.data);bad['samples'][5]['split']='optimization'
        with self.assertRaises(ValueError): solve_dataset(bad)
        bad=deepcopy(self.data)
        p=np.array(bad['samples'][2]['cameras']['rs3']['corners_px']).reshape(8,11,2)[:,::-1]
        bad['samples'][2]['cameras']['rs3']['corners_px']=p.reshape(88,2).tolist()
        with self.assertRaises(ValueError): solve_dataset(bad)
        bad=deepcopy(self.data)
        p=np.array(bad['samples'][5]['cameras']['rs3']['corners_px'])[::-1]
        bad['samples'][5]['cameras']['rs3']['corners_px']=p.tolist()
        self.assertFalse(solve_dataset(bad)['validation']['passed'])

    def test_clicks_only_select_order_under_rotation(self):
        p=np.array(self.data['samples'][0]['cameras']['rs1']['corners_px'])
        for turns in range(4):
            angle=turns*np.pi/2
            rot=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
            expected=p@rot.T
            for grid in (expected.reshape(8,11,2),expected.reshape(8,11,2)[::-1],expected.reshape(8,11,2)[:,::-1],expected.reshape(8,11,2)[::-1,::-1]):
                got=ordered_corners(grid.reshape(88,2),expected[0]+[1.0,-1.0])
                np.testing.assert_array_equal(got,expected)
        with self.assertRaises(ValueError): ordered_corners(p,p[30])

    def test_stamps_and_movement(self):
        check_frame_stamps([10,10.01],[10,10.01],10.1)
        for stamps,now in (([10,10.2],10.21),([10,10],11),([11,11],10)):
            with self.assertRaises(ValueError): check_frame_stamps(stamps,stamps,now)
        with self.assertRaises(ValueError): check_frame_stamps([10,10],[10,10],10,[10,10])
        ref={'rs1':dict(camera_info={},camera_T_board=np.eye(4),corners_px=np.zeros((88,2)))}
        current=deepcopy(ref);current['rs1']['corners_px']+=2
        with self.assertRaises(ValueError):check_stationary(ref,current)

    def test_real_opencv_detection_on_rotated_pattern(self):
        rows,cols=np.indices((9,12));board=np.uint8((rows+cols)%2*255)
        board=cv2.resize(board,(384,288),interpolation=cv2.INTER_NEAREST)
        for turns in range(4):
            rotated=np.rot90(board,turns)
            canvas=np.full((480,640),255,np.uint8);h,w=rotated.shape
            canvas[(480-h)//2:(480+h)//2,(640-w)//2:(640+w)//2]=rotated
            self.assertEqual(detect_corners(cv2.cvtColor(canvas,cv2.COLOR_GRAY2BGR)).shape,(88,2))
