#!/usr/bin/env python

import unittest
from klampt.math import so3

class so3Test(unittest.TestCase):

    def setUp(self):
        pass

    def test_from_rpy(self):
        R = so3.from_rpy((0,0,0))

    def test_rpy(self):
        R = so3.from_quaternion((-4.32978e-17, -0.707107,4.32978e-17,0.707107))
        r,p,y = so3.rpy(R)
        self.assertAlmostEqual(r,0.0)
        self.assertAlmostEqual(p,1.5707963267948966)
        self.assertAlmostEqual(y,3.141592653589793)

if __name__ == '__main__':
    unittest.main()