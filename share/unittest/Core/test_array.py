import corepy
import numpy

class TestArray_SWIGTypemaps():
    """
    Note: this testclass also tests the magic method __getitem__.
    """

    def test_return_arr(self):
        a = corepy.return_arr()    # should return [[1. 3]. [5. 7]]

        # check dimensions
        assert a.size == 4
        assert a.shape == (2, 2)

        assert a.dtype == numpy.float64

        # check each value
        assert (a[0, 0] - 1.2) < 0.01
        assert (a[0, 1] - 3.4) < 0.01
        assert (a[1, 0] - 5.6) < 0.01
        assert (a[1, 1] - 7.8) < 0.01

    def test_return_intA(self):
        a = corepy.return_intA()    # should return [[1. 3]. [5. 7]]

        # check dimensions
        assert a.size == 4
        assert a.shape == (2, 2)

        assert a.dtype == numpy.int32

        # check each value
        assert (a[0, 0] + 1) < 0.01
        assert (a[0, 1] - 3) < 0.01
        assert (a[1, 0] + 5) < 0.01
        assert (a[1, 1] - 7) < 0.01

    def test_return_uintA(self):
        a = corepy.return_uintA()    # should return [[1. 3]. [5. 7]]

        # check dimensions
        assert a.size == 4
        assert a.shape == (2, 2)

        assert a.dtype == numpy.uint32

        # check each value
        assert (a[0, 0] - 1) < 0.01
        assert (a[0, 1] - 3) < 0.01
        assert (a[1, 0] - 5) < 0.01
        assert (a[1, 1] - 7) < 0.01

    def test_argument_arr_value_1D(self):
        a = numpy.array([1, 2, 3]);
        b = corepy.identity_arr_value(a)
        assert (a == b).all()

    def test_argument_arr_value_2D(self):
        a = numpy.array([[1, 2, 3], [4, 5, 6]]);
        b = corepy.identity_arr_value(a)
        assert (a == b).all()

    def test_argument_arr_reference_1D(self):
        a = numpy.array([1, 2, 3]);
        b = corepy.identity_arr_reference(a)
        assert (a == b).all()

    def test_argument_arr_reference_2D(self):
        a = numpy.array([[1, 2, 3], [4, 5, 6]]);
        b = corepy.identity_arr_reference(a)
        assert (a == b).all()

    def test_argument_arr_pointer_1D(self):
        a = numpy.array([1, 2, 3]);
        b = corepy.identity_arr_pointer(a)
        assert (a == b).all()

    def test_argument_arr_pointer_2D(self):
        a = numpy.array([[1, 2, 3], [4, 5, 6]]);
        b = corepy.identity_arr_pointer(a)
        assert (a == b).all()

    def test_argument_intA_value_1D(self):
        a = numpy.array([1, 2, 3], dtype=numpy.int32);
        b = corepy.identity_intA_value(a)
        assert (a == b).all()

    def test_argument_intA_value_2D(self):
        a = numpy.array([[1, 2, 3], [4, 5, 6]], dtype=numpy.int32);
        b = corepy.identity_intA_value(a)
        assert (a == b).all()

    def test_argument_intA_reference_1D(self):
        a = numpy.array([1, 2, 3], dtype=numpy.int32);
        b = corepy.identity_intA_reference(a)
        assert (a == b).all()

    def test_argument_intA_reference_2D(self):
        a = numpy.array([[1, 2, 3], [4, 5, 6]], dtype=numpy.int32);
        b = corepy.identity_intA_reference(a)
        assert (a == b).all()

    def test_argument_intA_pointer_1D(self):
        a = numpy.array([1, 2, 3], dtype=numpy.int32);
        b = corepy.identity_intA_pointer(a)
        assert (a == b).all()

    def test_argument_intA_pointer_2D(self):
        a = numpy.array([[1, 2, 3], [4, 5, 6]], dtype=numpy.int32);
        b = corepy.identity_intA_pointer(a)
        assert (a == b).all()

    def test_argument_uintA_value_1D(self):
        a = numpy.array([1, 2, 3], dtype=numpy.uint32);
        b = corepy.identity_uintA_value(a)
        assert (a == b).all()

    def test_argument_uintA_value_2D(self):
        a = numpy.array([[1, 2, 3], [4, 5, 6]], dtype=numpy.uint32);
        b = corepy.identity_uintA_value(a)
        assert (a == b).all()

    def test_argument_uintA_reference_1D(self):
        a = numpy.array([1, 2, 3], dtype=numpy.uint32);
        b = corepy.identity_uintA_reference(a)
        assert (a == b).all()

    def test_argument_uintA_reference_2D(self):
        a = numpy.array([[1, 2, 3], [4, 5, 6]], dtype=numpy.uint32);
        b = corepy.identity_uintA_reference(a)
        assert (a == b).all()

    def test_argument_uintA_pointer_1D(self):
        a = numpy.array([1, 2, 3], dtype=numpy.uint32);
        b = corepy.identity_uintA_pointer(a)
        assert (a == b).all()

    def test_argument_uintA_pointer_2D(self):
        a = numpy.array([[1, 2, 3], [4, 5, 6]], dtype=numpy.uint32);
        b = corepy.identity_uintA_pointer(a)
        assert (a == b).all()

class TestArray_MatlabFunctions():
    """
    Note: this testclass also tests the magic method __getitem__.
    """

    def test_creation_of_matrix_of_ones(self):
        a = corepy.ones(2, 2)
        # check dimensions
        assert a.size == 4
        assert a.shape == (2, 2)
        # check each value
        assert (a[0, 0] - 1.) < 0.01
        assert (a[0, 1] - 1.) < 0.01
        assert (a[1, 0] - 1.) < 0.01
        assert (a[1, 1] - 1.) < 0.01

    def test_creation_of_matrix_of_zeros(self):
        a = corepy.zeros(2, 3)
        # check dimensions
        assert a.size == 6
        assert a.shape == (2, 3)
        # check each value
        assert (a[0, 0] - 0.) < 0.01
        assert (a[0, 1] - 0.) < 0.01
        assert (a[0, 2] - 0.) < 0.01
        assert (a[1, 0] - 0.) < 0.01
        assert (a[1, 1] - 0.) < 0.01
        assert (a[1, 2] - 0.) < 0.01

    def test_creation_of_identity_matrix(self):
        a = corepy.eye(2, 2)
        assert a.shape == (2, 2)
        # check dimensions
        assert a.size == 4
        # check each value
        assert (a[0, 0] - 1.) < 0.01
        assert (a[0, 1] - 0.) < 0.01
        assert (a[1, 0] - 0.) < 0.01
        assert (a[1, 1] - 1.) < 0.01


class TestArray_setFromPythonList():
    """
    Note: this testclass also tests the magic method __getitem__.
    """
    def test_setWithNumpyArray1D_one_element(self):
        a = numpy.array([1]);
        # check dimensions
        assert a.size == 1
        assert a.shape == (1,)
        # check each value
        assert (a[0] - 1.) < 0.01

    def test_setWithPythonList1D_multiple_elements(self):
        a = numpy.array([1.] * 10)
        # check dimensions
        assert a.size == 10
        assert a.shape == (10,)
        # check each value
        assert (a[0] - 1.) < 0.01
        assert (a[4] - 1.) < 0.01
        assert (a[9] - 1.) < 0.01

    def test_setWithNumpyArray2D(self):
        a = numpy.array([[1., 1.], [2., 2.]])
        assert a.shape == (2, 2)
        # check dimensions
        # check each value
        assert (a[0, 0] - 1.) < 0.01
        assert (a[0, 1] - 1.) < 0.01
        assert (a[1, 0] - 2.) < 0.01
        assert (a[1, 1] - 2.) < 0.01


class TestArray_setitem():
    def test_setitem_with_multiple_dimensions(self):
        a = corepy.zeros(2, 2)
        a[0, 0] = 1.
        a[0, 1] = 2.
        a[1, 0] = 3.
        a[1, 1] = 4.

        assert (a[0, 0] - 1.) < 0.01
        assert (a[0, 1] - 2.) < 0.01
        assert (a[1, 0] - 3.) < 0.01
        assert (a[1, 1] - 4.) < 0.01

    def test_setitem_with_single_dimensions(self):
        a = numpy.array([0.] * 10)

        a[0] = 1.
        assert (a[0] - 1.) < 0.01

        a[7] = 1.
        assert (a[7] - 1.) < 0.01


class TestArray_slicing():
    """
    eye() allows to easily identify if the correct columns/rows were selected.
    """

    def test_get_full_sub_matrix(self):
        n = 5
        a = corepy.eye(n, n)
        tmp = a[0:n, 0:n]
        print tmp
        assert tmp.size == n*n
        assert tmp.shape == (n, n)

    def test_get_full_sub_matrix_with_index_out_of_bounds(self):
        """python it pretty forgiving when indices are out of bounds"""
        n = 5
        a = corepy.eye(n, n)
        tmp = a[0:1000, 0:1000]
        print tmp
        assert tmp.size == n*n
        assert tmp.shape == (n, n)

    def test_get_full_sub_matrix_with_slice_without_first_indices(self):
        n = 5
        a = corepy.eye(n, n)
        tmp = a[:n, :n]
        print tmp
        assert tmp.size == n*n
        assert tmp.shape == (n, n)

    def test_get_full_sub_matrix_with_slice_without_last_indices(self):
        n = 5
        a = corepy.eye(n, n)
        tmp = a[0:, 0:]
        print tmp
        assert tmp.size == n*n
        assert tmp.shape == (n, n)

    def test_get_full_sub_matrix_with_slice_without_any_indices(self):
        n = 5
        a = corepy.eye(n, n)
        tmp = a[:, :]
        print tmp
        assert tmp.size == n*n
        assert tmp.shape == (n, n)
        assert (tmp[0, 0] - 1.) < 0.01
        assert (tmp[2, 2] - 1.) < 0.01
        assert (tmp[4, 4] - 1.) < 0.01

    def test_get_one_row(self):
        n = 5
        a = corepy.eye(n, n)
        tmp = a[0:1, 0:n]
        print tmp
        assert tmp.size == n
        assert tmp.shape == (1, n)
        assert (tmp[0, 0] - 1.) < 0.01

    def test_get_one_row_with_explicet_index(self):
        n = 5
        a = corepy.eye(n, n)
        tmp = a[0, :]
        print tmp
        assert tmp.size == n
        assert tmp.shape == (n, )
        assert (tmp[0] - 1.) < 0.01

        tmp = a[4, :]
        assert (tmp[4] - 1.) < 0.01

    def test_get_multiple_rows(self):
        n = 5
        a = corepy.eye(n, n)
        tmp = a[0:3, :]
        print tmp
        assert tmp.size == 3 * n
        assert tmp.shape == (3, n)
        assert (tmp[0, 0] - 1.) < 0.01
        assert (tmp[1, 1] - 1.) < 0.01
        assert (tmp[2, 2] - 1.) < 0.01

    def test_get_with_one_column(self):
        n = 5
        a = corepy.eye(n, n)
        tmp = a[:, 0:1]
        print tmp
        assert tmp.size == n
        assert tmp.shape == (n, 1)
        assert (tmp[0] - 1.) < 0.01

    def test_get_with_one_column_with_explicit_index(self):
        n = 5
        a = corepy.eye(n, n)
        tmp = a[:, 3]
        print tmp
        assert tmp.size == n
        assert tmp.shape == (n,)
        assert (tmp[0] - 1.) < 0.01

    def test_get_with_multiple_column(self):
        n = 5
        a = corepy.eye(n, n)
        tmp = a[:, 0:3]
        print tmp
        assert tmp.size == 3 * n
        assert tmp.shape == (n, 3)
        assert (tmp[0, 0] - 1.) < 0.01
        assert (tmp[2, 2] - 1.) < 0.01
