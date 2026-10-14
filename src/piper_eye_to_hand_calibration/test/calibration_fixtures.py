from copy import deepcopy
from pathlib import Path
import cv2
import numpy as np
from piper_eye_to_hand_calibration.artifacts import SERIALS, camera_record, write_yaml
from piper_eye_to_hand_calibration.intrinsics import load_intrinsics
from piper_eye_to_hand_calibration.stereo import TARGET, project, solve_dataset
from piper_eye_to_hand_calibration.composition import transform_record


def transform(rotation, translation):
    t = np.eye(4)
    t[:3, :3] = cv2.Rodrigues(np.array(rotation, float))[0]
    t[:3, 3] = translation
    return t


def profile(name='rs1'):
    return dict(camera=name, camera_serial_number=SERIALS[name],
                camera_name=name+'_'+SERIALS[name]+'_640x480', image_width=640, image_height=480,
                distortion_model='plumb_bob',
                camera_matrix=dict(rows=3, cols=3, data=[510.,0,320,0,505.,240,0,0,1]),
                distortion_coefficients=dict(rows=1, cols=5, data=[0.,0,0,0,0]),
                rectification_matrix=dict(rows=3, cols=3, data=np.eye(3).ravel().tolist()),
                projection_matrix=dict(rows=3, cols=4, data=[510.,0,320,0,0,505.,240,0,0,0,1,0]))


def dataset(root):
    root=Path(root)
    cameras={}
    for name in SERIALS:
        path=root/(name+'.yaml')
        write_yaml(path, profile(name))
        cameras[name]=camera_record(load_intrinsics(path), transform([.01,.02,.03],[.01,0,0]))
    relative=transform([.015,-.035,.025], [.12,.01,.015])
    samples=[]
    for i in range(6):
        pose=transform([.18+.035*i,-.24+.03*i,.045*i],[-.13+.009*i,-.08+.007*i,.8+.025*i])
        p3=np.linalg.inv(relative)@pose
        samples.append(dict(id=i+1,split='validation' if i==5 else 'optimization',cameras={
            'rs1':dict(corners_px=project(pose,cameras['rs1']['camera_info']).tolist()),
            'rs3':dict(corners_px=project(p3,cameras['rs3']['camera_info']).tolist())}))
    return dict(schema_version=1,target=deepcopy(TARGET),complete=True,cameras=cameras,samples=samples),relative


def result_files(root):
    from piper_eye_to_hand_calibration.artifacts import sha256
    data, relative=dataset(root)
    stereo=solve_dataset(data)
    sp=Path(root)/'stereo.yaml';write_yaml(sp,stereo)
    base=transform([.2,-.1,.15],[.35,-.2,.6])
    internal=np.array(data['cameras']['rs1']['link_T_optical'])
    p=load_intrinsics(Path(root)/'rs1.yaml')
    handeye=dict(schema_version=1,status='PASS',camera=dict(name='rs1',serial=SERIALS['rs1']),
        target=dict(dictionary='DICT_4X4_50',marker_id=18,marker_size_m=.100),
        stereo_binding=dict(path=str(sp),sha256=sha256(sp)),
        intrinsics=dict(camera_serial_number=SERIALS['rs1'],image_width=640,image_height=480,sha256=p['sha256'],
                        camera_matrix=p['camera_matrix'].tolist(),distortion_coefficients=p['distortion'].tolist()),
        quality=dict(passed=True,sample_count=20,translation_rms_m=.002,rotation_rms_deg=.3),
        calibration=dict(base_to_camera_optical=transform_record('arm_base','rs1_color_optical_frame',base)),
        tf_publication=dict(base_to_camera_link=transform_record('arm_base','rs1_link',base@np.linalg.inv(internal)),
                            camera_link_to_optical=transform_record('rs1_link','rs1_color_optical_frame',internal)))
    hp=Path(root)/'handeye.yaml';write_yaml(hp,handeye)
    return sp,hp,base,relative
