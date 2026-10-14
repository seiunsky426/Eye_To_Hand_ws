import io
from pathlib import Path
import tarfile
import tempfile
import unittest
import yaml

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))

from calibration_fixtures import profile
from piper_eye_to_hand_calibration.intrinsics import load_intrinsics
from piper_eye_to_hand_calibration.intrinsics_import import import_archive
from piper_eye_to_hand_calibration.artifacts import SERIALS, sha256


class IntrinsicsTest(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(); self.root=Path(self.tmp.name)
        self.addCleanup(self.tmp.cleanup)

    def archive(self, data, extra=None):
        path=self.root/'calibrationdata.tar.gz'
        with tarfile.open(path,'w:gz') as tar:
            contents={'ost.yaml':yaml.safe_dump(data).encode(),'left-0000.png':b'original-image-bytes'}
            if extra: contents.update(extra)
            for name, raw in contents.items():
                m=tarfile.TarInfo(name);m.size=len(raw);tar.addfile(m,io.BytesIO(raw))
        return path

    def test_import_preserves_original_and_metadata(self):
        archive=self.archive(profile()); digest=sha256(archive)
        target=import_archive(archive,'rs1',self.root/'result')
        p=load_intrinsics(target,'rs1',SERIALS['rs1'])
        self.assertEqual((p['width'],p['height']),(640,480))
        self.assertEqual(digest,sha256(target.parent/'calibrationdata.tar.gz'))
        self.assertEqual((target.parent/'ros_originals/left-0000.png').read_bytes(),b'original-image-bytes')
        self.assertTrue((target.parent/'manifest.yaml').is_file())
        with self.assertRaises(FileExistsError): import_archive(archive,'rs1',target.parent)
        self.assertEqual(sha256(archive),digest)

    def test_wrong_identity_resolution_and_matrix(self):
        for change in ({'camera_name':'rs1_wrong_640x480'},{'image_height':360},
                       {'camera_matrix':{'rows':3,'cols':3,'data':[0]*9}},
                       {'rectification_matrix':{'rows':3,'cols':3,'data':[0]*9}}):
            with self.subTest(change=change):
                data=profile(); data.update(change)
                with self.assertRaises(ValueError): import_archive(self.archive(data),'rs1',self.root/'bad')
                self.assertFalse((self.root/'bad').exists())

    def test_other_camera_and_archive_traversal_rejected(self):
        with self.assertRaises(ValueError): import_archive(self.archive(profile('rs3')),'rs1',self.root/'bad')
        with self.assertRaises(ValueError): import_archive(self.archive(profile(),{'../bad':b'x'}),'rs1',self.root/'bad')
