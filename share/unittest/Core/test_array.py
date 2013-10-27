import core_testpy
import corepy
import numpy

class TestArray_SWIGTypemaps():
    """
    Note: this testclass also tests the magic method __getitem__.
    """

    def test_return_arr(self):
        a = core_testpy.return_arr()    # should return [[1. 3]. [5. 7]]

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
        a = core_testpy.return_intA()    # should return [[1. 3]. [5. 7]]

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
        a = core_testpy.return_uintA()    # should return [[1. 3]. [5. 7]]

        # check dimensions
        assert a.size == 4
        assert a.shape == (2, 2)

        assert a.dtype == numpy.uint32

        # check each value
        assert (a[0, 0] - 1) < 0.01
        assert (a[0, 1] - 3) < 0.01
        assert (a[1, 0] - 5) < 0.01
        assert (a[1, 1] - 7) < 0.01

    def test_memory_leakage(self):
        a = corepy.zeros(100)
        for i in range(1, 100000):
            core_testpy.identity_arr_pointer(a)

    def test_argument_arr_value_1D(self):
        a = numpy.array([1, 2, 3]);
        b = core_testpy.identity_arr_value(a)
        assert (a == b).all()

    def test_argument_const_arr_value(self):
        a = numpy.array([1, 2, 3]);
        b = core_testpy.identity_const_arr_value(a)
        assert (a == b).all()

    def test_argument_arr_value_2D(self):
        a = numpy.array([[1, 2, 3], [4, 5, 6]]);
        b = core_testpy.identity_arr_value(a)
        assert (a == b).all()

    def test_argument_arr_reference_1D(self):
        a = numpy.array([1, 2, 3]);
        b = core_testpy.identity_arr_reference(a)
        assert (a == b).all()

    def test_argument_const_arr_reference(self):
        a = numpy.array([1, 2, 3]);
        b = core_testpy.identity_const_arr_reference(a)
        assert (a == b).all()

    def test_argument_arr_reference_2D(self):
        a = numpy.array([[1, 2, 3], [4, 5, 6]]);
        b = core_testpy.identity_arr_reference(a)
        assert (a == b).all()

    def test_argument_arr_pointer_1D(self):
        a = numpy.array([1, 2, 3]);
        b = core_testpy.identity_arr_pointer(a)
        assert (a == b).all()

    def test_argument_const_arr_pointer(self):
        a = numpy.array([1, 2, 3]);
        b = core_testpy.identity_const_arr_pointer(a)
        assert (a == b).all()

    def test_argument_arr_pointer_2D(self):
        a = numpy.array([[1, 2, 3], [4, 5, 6]]);
        b = core_testpy.identity_arr_pointer(a)
        assert (a == b).all()

    def test_argument_intA_value_1D(self):
        a = numpy.array([1, 2, 3], dtype=numpy.int32);
        b = core_testpy.identity_intA_value(a)
        assert (a == b).all()

    def test_argument_intA_value_2D(self):
        a = numpy.array([[1, 2, 3], [4, 5, 6]], dtype=numpy.int32);
        b = core_testpy.identity_intA_value(a)
        assert (a == b).all()

    def test_argument_intA_reference_1D(self):
        a = numpy.array([1, 2, 3], dtype=numpy.int32);
        b = core_testpy.identity_intA_reference(a)
        assert (a == b).all()

    def test_argument_intA_reference_2D(self):
        a = numpy.array([[1, 2, 3], [4, 5, 6]], dtype=numpy.int32);
        b = core_testpy.identity_intA_reference(a)
        assert (a == b).all()

    def test_argument_intA_pointer_1D(self):
        a = numpy.array([1, 2, 3], dtype=numpy.int32);
        b = core_testpy.identity_intA_pointer(a)
        assert (a == b).all()

    def test_argument_intA_pointer_2D(self):
        a = numpy.array([[1, 2, 3], [4, 5, 6]], dtype=numpy.int32);
        b = core_testpy.identity_intA_pointer(a)
        assert (a == b).all()

    def test_argument_uintA_value_1D(self):
        a = numpy.array([1, 2, 3], dtype=numpy.uint32);
        b = core_testpy.identity_uintA_value(a)
        assert (a == b).all()

    def test_argument_uintA_value_2D(self):
        a = numpy.array([[1, 2, 3], [4, 5, 6]], dtype=numpy.uint32);
        b = core_testpy.identity_uintA_value(a)
        assert (a == b).all()

    def test_argument_uintA_reference_1D(self):
        a = numpy.array([1, 2, 3], dtype=numpy.uint32);
        b = core_testpy.identity_uintA_reference(a)
        assert (a == b).all()

    def test_argument_uintA_reference_2D(self):
        a = numpy.array([[1, 2, 3], [4, 5, 6]], dtype=numpy.uint32);
        b = core_testpy.identity_uintA_reference(a)
        assert (a == b).all()

    def test_argument_uintA_pointer_1D(self):
        a = numpy.array([1, 2, 3], dtype=numpy.uint32);
        b = core_testpy.identity_uintA_pointer(a)
        assert (a == b).all()

    def test_argument_uintA_pointer_2D(self):
        a = numpy.array([[1, 2, 3], [4, 5, 6]], dtype=numpy.uint32);
        b = core_testpy.identity_uintA_pointer(a)
        assert (a == b).all()

    def test_overloading(self):
        a = numpy.array([1, 2, 3])
        b = core_testpy.test_overloading(a)
        c = core_testpy.test_overloading(.1)
        assert (a == b).all()
        assert c.shape == (1, )
        assert c.size == 1
        assert c[0] - .1 < 0.01


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


class TestList_SWIGTypemaps:
    def test_return_arrL(self):
        arrl = core_testpy.return_arrL();
        arr = core_testpy.return_arr();
        assert (arrl[0]==arr)
