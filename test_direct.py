import struct
import unittest
import numpy as np
from extract_inspvax import crc32,ecef
from map_direct import parse,ROT

class DirectTests(unittest.TestCase):
 def test_crc_known_vector(self):
  self.assertEqual(crc32(b'123456789'),0x2dfd2d88)
 def test_livox_wire_fields(self):
  header=struct.pack('<IIII',1,2,3,0)
  body=struct.pack('<QIB3B',123456789,1,0,0,0,0)+struct.pack('<I',1)
  point=struct.pack('<IfffBBB',12,1.,2.,3.,55,0,4)
  base,pts=parse(header+body+point)
  self.assertEqual(base,123456789);self.assertEqual(len(pts),1);self.assertEqual(pts['offset'][0],12);self.assertEqual(pts['reflectivity'][0],55)
 def test_geodesy_and_candidate_rotation(self):
  self.assertTrue(np.allclose(ecef(0,0,0),[6378137,0,0]))
  self.assertTrue(np.allclose(ROT.T@ROT,np.eye(3),atol=1e-6));self.assertAlmostEqual(np.linalg.det(ROT),1,places=6)

if __name__=='__main__':unittest.main()

