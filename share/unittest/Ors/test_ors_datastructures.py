from corepy import Vector, Matrix, Quaternion
import orspy

import os
import unittest


class Test_VectorOperatorOverloading(unittest.TestCase):
    def test_add(self):
        v1 = Vector(1, 2, 3)
        v2 = Vector(2, 2, 2)
        v3 = v1 + v2
        # check each value
        assert (v3.x - 3.) < 0.001
        assert (v3.y - 4.) < 0.001
        assert (v3.z - 5.) < 0.001

    def test_sub(self):
        v1 = Vector(1, 2, 3)
        v2 = Vector(2, 2, 2)
        v3 = v2 - v1
        # check each value
        assert (v3.x - 1.) < 0.001
        assert (v3.y - 0.) < 0.001
        assert (v3.z - -1.) < 0.001

    def test_equals(self):
        v1 = Vector(1, 2, 3)
        v2 = Vector(2, 2, 2)

        self.assertTrue(v1 == v1)
        self.assertTrue(v2 == v2)
        self.assertFalse(v1 == v2)
        
        v3 = Vector(2, 2, 2)
        self.assertTrue(v2 == v3)

    def test_unequals(self):
        v1 = Vector(1, 2, 3)
        v2 = Vector(2, 2, 2)

        self.assertFalse(v1 != v1)
        self.assertFalse(v2 != v2)
        self.assertTrue(v1 != v2)

        v3 = Vector(2, 2, 2)
        self.assertFalse(v2 != v3)


class Test_ListOfOrsDatastuctures(unittest.TestCase):
    def test_vector(self):
        v1 = Vector(1, 2, 3)
        v2 = Vector(3, 2, 1)
        l = []
        l.append(v1)
        l.append(v2)

        self.assertTrue(v1 == l[0])
        self.assertTrue(v2 == l[1])
        self.assertTrue(v1 != l[1])
        self.assertTrue(v2 != l[0])


class Test_MatrixOperatorOverloading(unittest.TestCase):
    def test_add(self):
        m1, m2 = Matrix(), Matrix()
        m1.set([1.] * 9)
        m2.set([2.] * 9)
        m3 = m1 + m2

        # check each value
        assert (m3.m00 - 3.) < 0.001
        assert (m3.m01 - 3.) < 0.001
        assert (m3.m02 - 3.) < 0.001

        assert (m3.m10 - 3.) < 0.001
        assert (m3.m11 - 3.) < 0.001
        assert (m3.m12 - 3.) < 0.001

        assert (m3.m20 - 3.) < 0.001
        assert (m3.m21 - 3.) < 0.001
        assert (m3.m22 - 3.) < 0.001

    def test_equals(self):
        m1, m2 = Matrix(), Matrix()
        m1.set([1.] * 9)
        m2.set([2.] * 9)

        self.assertTrue(m1 == m1)
        self.assertTrue(m2 == m2)
        self.assertFalse(m1 == m2)

        m3 = Matrix()
        m3.set([2.] * 9)
        self.assertTrue(m2 == m3)

    def test_unequals(self):
        m1, m2 = Matrix(), Matrix()
        m1.set([1.] * 9)
        m2.set([2.] * 9)

        self.assertFalse(m1 != m1)
        self.assertFalse(m2 != m2)
        self.assertTrue(m1 != m2)

        m3 = Matrix()
        m3.set([2.] * 9)
        self.assertFalse(m2 != m3)


class Test_QuaternionOperatorOverload(unittest.TestCase):
    def setUp(self):
        self.maxDiff = None

    def test_equals(self):
        q1 = Quaternion(1, 2, 3, 4)
        q2 = Quaternion(4, 3, 2, 1)

        self.assertTrue(q1 == q1)
        self.assertTrue(q2 == q2)
        self.assertFalse(q1 == q2)

        q3 = Quaternion(4, 3, 2, 1)
        self.assertTrue(q2 == q3)

    def test_unequals(self):
        q1 = Quaternion(1, 2, 3, 4)
        q2 = Quaternion(4, 3, 2, 1)

        self.assertFalse(q1 != q1)
        self.assertFalse(q2 != q2)
        self.assertTrue(q1 != q2)

        q3 = Quaternion(4, 3, 2, 1)
        self.assertFalse(q2 != q3)


class Test_Graph_serialization(unittest.TestCase):
    def test_graph_de_serialization(self):
        p = os.path.dirname(os.path.realpath(__file__))
        f = os.path.join(p, "world.ors")
        graph1 = orspy.Graph(f)
        g1_str = str(graph1)

        graph2 = orspy.Graph()
        graph2.read(g1_str)
        g2_str = str(graph2)

        # sadly the string representation is not unambiguous.
        # sometimes a '-0', and sometimes a '0' is used. Therefore, we
        # have to split the string representation in words, try to convert
        # them into float and then compare them.
        for line_a, line_b in zip(g1_str.split('\n'), g2_str.split('\n')):
            for a, b in zip(line_a.split(' '), line_b.split(' ')):
                print a, b
                try:
                    a, b = float(a), float(b)
                    assert (a, b) < 0.001
                except:
                    assert a == b


if __name__ == '__main__':
    unittest.main()
