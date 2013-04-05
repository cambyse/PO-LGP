import sys
sys.path.append('../../lib/')
import orspy

import unittest2 as unittest


class TestArray_MatlabFunctions(unittest.TestCase):
    """
    Note: this testclass also tests the magic method __getitem__.
    """

    def test_creation_of_matrix_of_ones(self):
        a = orspy.ones(2, 2)
        # check dimensions
        self.assertEqual(a.N, 4)
        self.assertEqual(a.d0, 2)
        self.assertEqual(a.d1, 2)
        # check each value
        self.assertAlmostEqual(a[0, 0], 1.)
        self.assertAlmostEqual(a[0, 1], 1.)
        self.assertAlmostEqual(a[1, 0], 1.)
        self.assertAlmostEqual(a[1, 1], 1.)

    def test_creation_of_matrix_of_zeros(self):
        a = orspy.zeros(2, 3)
        # check dimensions
        self.assertEqual(a.N, 6)
        self.assertEqual(a.d0, 2)
        self.assertEqual(a.d1, 3)
        # check each value
        self.assertAlmostEqual(a[0, 0], 0.)
        self.assertAlmostEqual(a[0, 1], 0.)
        self.assertAlmostEqual(a[0, 2], 0.)
        self.assertAlmostEqual(a[1, 0], 0.)
        self.assertAlmostEqual(a[1, 1], 0.)
        self.assertAlmostEqual(a[1, 2], 0.)

    def test_creation_of_identity_matrix(self):
        a = orspy.eye(2, 2)
        # check dimensions
        self.assertEqual(a.N, 4)
        self.assertEqual(a.d0, 2)
        self.assertEqual(a.d1, 2)
        # check each value
        self.assertAlmostEqual(a[0, 0], 1.)
        self.assertAlmostEqual(a[0, 1], 0.)
        self.assertAlmostEqual(a[1, 0], 0.)
        self.assertAlmostEqual(a[1, 1], 1.)


class TestArray_setFromPythonList(unittest.TestCase):
    """
    Note: this testclass also tests the magic method __getitem__.
    """
    def test_setWithPythonList1D_one_element(self):
        a = orspy.ArrayDouble()
        a.setWithList([1.])
        # check dimensions
        self.assertEqual(a.N, 1)
        self.assertEqual(a.d0, 1)
        # check each value
        self.assertAlmostEqual(a[0], 1.)

    def test_setWithPythonList1D_multiple_elements(self):
        a = orspy.ArrayDouble()
        a.setWithList([1.] * 10)
        # check dimensions
        self.assertEqual(a.N, 10)
        self.assertEqual(a.d0, 10)
        # check each value
        self.assertAlmostEqual(a[0], 1.)
        self.assertAlmostEqual(a[4], 1.)
        self.assertAlmostEqual(a[9], 1.)

    def test_setWithPythonList2D(self):
        a = orspy.ArrayDouble()
        a.setWithList([[1., 1.], [2., 2.]])
        # check dimensions
        self.assertEqual(a.d0, 2)
        self.assertEqual(a.d1, 2)
        # check each value
        self.assertAlmostEqual(a[0, 0], 1.)
        self.assertAlmostEqual(a[0, 1], 1.)
        self.assertAlmostEqual(a[1, 0], 2.)
        self.assertAlmostEqual(a[1, 1], 2.)


class TestArray_setitem(unittest.TestCase):
    def test_setitem_with_multiple_dimensions(self):
        a = orspy.zeros(2, 2)
        a[0, 0] = 1.
        a[0, 1] = 2.
        a[1, 0] = 3.
        a[1, 1] = 4.

        self.assertAlmostEqual(a[0, 0], 1.)
        self.assertAlmostEqual(a[0, 1], 2.)
        self.assertAlmostEqual(a[1, 0], 3.)
        self.assertAlmostEqual(a[1, 1], 4.)
        self.assertAlmostEqual(a[1, 1], 4.)

    def test_setitem_with_single_dimensions(self):
        a = orspy.ArrayDouble()
        a.setWithList([0] * 10)

        a[0] = 1.
        self.assertAlmostEqual(a[0], 1.)

        a[7] = 1.
        self.assertAlmostEqual(a[7], 1.)

# run tests
if __name__ == '__main__':
    unittest.main()
