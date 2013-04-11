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


# run tests
if __name__ == '__main__':
    unittest.main()
