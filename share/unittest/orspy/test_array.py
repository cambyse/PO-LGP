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

    def test_setitem_with_single_dimensions(self):
        a = orspy.ArrayDouble()
        a.setWithList([0] * 10)

        a[0] = 1.
        self.assertAlmostEqual(a[0], 1.)

        a[7] = 1.
        self.assertAlmostEqual(a[7], 1.)


class TestArray_slicing(unittest.TestCase):
    """
    eye() allows to easily identify if the correct columns/rows were selected.
    """

    def test_get_full_sub_matrix(self):
        n = 5
        a = orspy.eye(n, n)
        tmp = a[0:n, 0:n]
        print tmp
        self.assertEqual(tmp.N, n*n)
        self.assertEqual(tmp.d0, n)
        self.assertEqual(tmp.d1, n)

    def test_get_full_sub_matrix_with_index_out_of_bounds(self):
        """python it pretty forgiving when indices are out of bounds"""
        n = 5
        a = orspy.eye(n, n)
        tmp = a[0:1000, 0:1000]
        print tmp
        self.assertEqual(tmp.N, n*n)
        self.assertEqual(tmp.d0, n)
        self.assertEqual(tmp.d1, n)

    def test_get_full_sub_matrix_with_slice_without_first_indices(self):
        n = 5
        a = orspy.eye(n, n)
        tmp = a[:n, :n]
        print tmp
        self.assertEqual(tmp.N, n*n)
        self.assertEqual(tmp.d0, n)
        self.assertEqual(tmp.d1, n)

    def test_get_full_sub_matrix_with_slice_without_last_indices(self):
        n = 5
        a = orspy.eye(n, n)
        tmp = a[0:, 0:]
        print tmp
        self.assertEqual(tmp.N, n*n)
        self.assertEqual(tmp.d0, n)
        self.assertEqual(tmp.d1, n)

    def test_get_full_sub_matrix_with_slice_without_any_indices(self):
        n = 5
        a = orspy.eye(n, n)
        tmp = a[:, :]
        print tmp
        self.assertEqual(tmp.N, n*n)
        self.assertEqual(tmp.d0, n)
        self.assertEqual(tmp.d1, n)
        self.assertAlmostEqual(tmp[0, 0], 1.)
        self.assertAlmostEqual(tmp[2, 2], 1.)
        self.assertAlmostEqual(tmp[4, 4], 1.)

    def test_get_one_row(self):
        n = 5
        a = orspy.eye(n, n)
        tmp = a[0:1, 0:n]
        print tmp
        self.assertEqual(tmp.N, n)
        self.assertEqual(tmp.d0, 1)
        self.assertEqual(tmp.d1, n)
        self.assertAlmostEqual(tmp[0, 0], 1.)

    def test_get_one_row_with_explicet_index(self):
        n = 5
        a = orspy.eye(n, n)
        tmp = a[0, :]
        print tmp
        self.assertEqual(tmp.N, n)
        self.assertEqual(tmp.d0, 1)
        self.assertEqual(tmp.d1, n)
        self.assertAlmostEqual(tmp[0, 0], 1.)

        tmp = a[4, :]
        self.assertAlmostEqual(tmp[0, 4], 1.)

    def test_get_multiple_rows(self):
        n = 5
        a = orspy.eye(n, n)
        tmp = a[0:3, :]
        print tmp
        self.assertEqual(tmp.N, 3 * n)
        self.assertEqual(tmp.d0, 3)
        self.assertEqual(tmp.d1, n)
        self.assertAlmostEqual(tmp[0, 0], 1.)
        self.assertAlmostEqual(tmp[1, 1], 1.)
        self.assertAlmostEqual(tmp[2, 2], 1.)

    def test_get_with_one_column(self):
        n = 5
        a = orspy.eye(n, n)
        tmp = a[:, 0:1]
        print tmp
        self.assertEqual(tmp.N, n)
        self.assertEqual(tmp.d0, n)
        self.assertEqual(tmp.d1, 1)
        self.assertAlmostEqual(tmp[0, 0], 1.)

    def test_get_with_one_column_with_explicit_index(self):
        n = 5
        a = orspy.eye(n, n)
        tmp = a[:, 3]
        print tmp
        self.assertEqual(tmp.N, n)
        self.assertEqual(tmp.d0, n)
        self.assertEqual(tmp.d1, 1)
        self.assertAlmostEqual(tmp[3, 0], 1.)

    def test_get_with_multiple_column(self):
        n = 5
        a = orspy.eye(n, n)
        tmp = a[:, 0:3]
        print tmp
        self.assertEqual(tmp.N, 3 * n)
        self.assertEqual(tmp.d0, n)
        self.assertEqual(tmp.d1, 3)
        self.assertAlmostEqual(tmp[0, 0], 1.)
        self.assertAlmostEqual(tmp[2, 2], 1.)


if __name__ == '__main__':
    unittest.main()
