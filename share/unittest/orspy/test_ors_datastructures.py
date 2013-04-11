import sys
sys.path.append('../../lib/')
import orspy

import unittest2 as unittest


class Test_VectorOperatorOverloading(unittest.TestCase):
    def test_add(self):
        v1 = orspy.Vector(1, 2, 3)
        v2 = orspy.Vector(2, 2, 2)
        v3 = v1 + v2
        # check each value
        self.assertAlmostEqual(v3.x, 3.)
        self.assertAlmostEqual(v3.y, 4.)
        self.assertAlmostEqual(v3.z, 5.)

    def test_sub(self):
        v1 = orspy.Vector(1, 2, 3)
        v2 = orspy.Vector(2, 2, 2)
        v3 = v2 - v1
        # check each value
        self.assertAlmostEqual(v3.x, 1.)
        self.assertAlmostEqual(v3.y, 0.)
        self.assertAlmostEqual(v3.z, -1.)


class Test_MatrixOperatorOverloading(unittest.TestCase):
    def test_add(self):
        m1, m2 = orspy.Matrix(), orspy.Matrix()
        m1.set([1.] * 9)
        m2.set([2.] * 9)
        m3 = m1 + m2

        # check each value
        self.assertAlmostEqual(m3.m00, 3.)
        self.assertAlmostEqual(m3.m01, 3.)
        self.assertAlmostEqual(m3.m02, 3.)

        self.assertAlmostEqual(m3.m10, 3.)
        self.assertAlmostEqual(m3.m11, 3.)
        self.assertAlmostEqual(m3.m12, 3.)

        self.assertAlmostEqual(m3.m20, 3.)
        self.assertAlmostEqual(m3.m21, 3.)
        self.assertAlmostEqual(m3.m22, 3.)


# run tests
if __name__ == '__main__':
    unittest.main()
