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

    def test_equals(self):
        v1 = orspy.Vector(1, 2, 3)
        v2 = orspy.Vector(2, 2, 2)

        self.assertTrue(v1 == v1)
        self.assertTrue(v2 == v2)
        self.assertFalse(v1 == v2)

        v3 = orspy.Vector(2, 2, 2)
        self.assertTrue(v2 == v3)

    def test_unequals(self):
        v1 = orspy.Vector(1, 2, 3)
        v2 = orspy.Vector(2, 2, 2)

        self.assertFalse(v1 != v1)
        self.assertFalse(v2 != v2)
        self.assertTrue(v1 != v2)

        v3 = orspy.Vector(2, 2, 2)
        self.assertFalse(v2 != v3)

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

    def test_equals(self):
        m1, m2 = orspy.Matrix(), orspy.Matrix()
        m1.set([1.] * 9)
        m2.set([2.] * 9)

        self.assertTrue(m1 == m1)
        self.assertTrue(m2 == m2)
        self.assertFalse(m1 == m2)

        m3 = orspy.Matrix()
        m3.set([2.] * 9)
        self.assertTrue(m2 == m3)

    def test_unequals(self):
        m1, m2 = orspy.Matrix(), orspy.Matrix()
        m1.set([1.] * 9)
        m2.set([2.] * 9)

        self.assertFalse(m1 != m1)
        self.assertFalse(m2 != m2)
        self.assertTrue(m1 != m2)
        

        m3 = orspy.Matrix()
        m3.set([2.] * 9)
        self.assertFalse(m2 != m3)


class Test_QuaternionOperatorOverload(unittest.TestCase):
    def setUp(self):
        self.maxDiff = None

    def test_equals(self):
        q1 = orspy.Quaternion(1, 2, 3, 4)
        q2 = orspy.Quaternion(4, 3, 2, 1)

        self.assertTrue(q1 == q1)
        self.assertTrue(q2 == q2)
        self.assertFalse(q1 == q2)

        q3 = orspy.Quaternion(4, 3, 2, 1)
        self.assertTrue(q2 == q3)

    def test_unequals(self):
        q1 = orspy.Quaternion(1, 2, 3, 4)
        q2 = orspy.Quaternion(4, 3, 2, 1)

        self.assertFalse(q1 != q1)
        self.assertFalse(q2 != q2)
        self.assertTrue(q1 != q2)

        q3 = orspy.Quaternion(4, 3, 2, 1)
        self.assertFalse(q2 != q3)


class Test_Graph_serialization(unittest.TestCase):
    def setUp(self):
        self.maxDiff = None

    def test_graph_de_serialization(self):
        graph1 = orspy.Graph("world.ors")
        serialized1 = str(graph1)

        graph2 = orspy.Graph()
        graph2.read(serialized1)
        serialized2 = str(graph2)

        self.assertMultiLineEqual(serialized1, serialized2)


if __name__ == '__main__':
    unittest.main()
