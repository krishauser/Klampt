#!/usr/bin/env python

import unittest
from klampt.math import so3

class so3Test(unittest.TestCase):

    def setUp(self):
        pass

    def test_from_rpy(self):
        R = so3.from_rpy((0,0,0))

if __name__ == '__main__':
    unittest.main()