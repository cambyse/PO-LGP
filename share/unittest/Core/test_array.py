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
        assert abs(a[0, 0] - 1.2) < 0.01
        assert abs(a[0, 1] - 3.4) < 0.01
        assert abs(a[1, 0] - 5.6) < 0.01
        assert abs(a[1, 1] - 7.8) < 0.01

    def test_return_intA(self):
        a = core_testpy.return_intA()    # should return [[1. 3]. [5. 7]]

        # check dimensions
        assert a.size == 4
        assert a.shape == (2, 2)

        assert a.dtype == numpy.int32

        # check each value
        assert abs(a[0, 0] + 1) < 0.01
        assert abs(a[0, 1] - 3) < 0.01
        assert abs(a[1, 0] + 5) < 0.01
        assert abs(a[1, 1] - 7) < 0.01

    def test_return_uintA(self):
        a = core_testpy.return_uintA()    # should return [[1. 3]. [5. 7]]

        # check dimensions
        assert a.size == 4
        assert a.shape == (2, 2)

        assert a.dtype == numpy.uint32

        # check each value
        assert abs(a[0, 0] - 1) < 0.01
        assert abs(a[0, 1] - 3) < 0.01
        assert abs(a[1, 0] - 5) < 0.01
        assert abs(a[1, 1] - 7) < 0.01

    def un_test_memory_leakage(self):
        # if there is a memory leak, you will definitely get out of memory here
        a = corepy.zeros(100)
        for i in range(1, 100000):
            core_testpy.identity_arr_pointer(a)

    def test_argument_arr_value_1D(self):
        a = numpy.array([1, 2, 3])
        b = core_testpy.identity_arr_value(a)
        assert (a == b).all()

    def test_argument_const_arr_value(self):
        a = numpy.array([1, 2, 3])
        b = core_testpy.identity_const_arr_value(a)
        assert (a == b).all()

    def test_argument_arr_value_2D(self):
        a = numpy.array([[1, 2, 3], [4, 5, 6]])
        b = core_testpy.identity_arr_value(a)
        assert (a == b).all()

    def test_argument_arr_reference_1D(self):
        a = numpy.array([1, 2, 3])
        b = core_testpy.identity_arr_reference(a)
        assert (a == b).all()

    def test_argument_const_arr_reference(self):
        a = numpy.array([1, 2, 3])
        b = core_testpy.identity_const_arr_reference(a)
        assert (a == b).all()

    def test_argument_arr_reference_2D(self):
        a = numpy.array([[1, 2, 3], [4, 5, 6]])
        b = core_testpy.identity_arr_reference(a)
        assert (a == b).all()

    def test_argument_arr_pointer_1D(self):
        a = numpy.array([1, 2, 3])
        b = core_testpy.identity_arr_pointer(a)
        assert (a == b).all()

    def test_argument_const_arr_pointer(self):
        a = numpy.array([1, 2, 3])
        b = core_testpy.identity_const_arr_pointer(a)
        assert (a == b).all()

    def test_argument_arr_pointer_2D(self):
        a = numpy.array([[1, 2, 3], [4, 5, 6]])
        b = core_testpy.identity_arr_pointer(a)
        assert (a == b).all()

    def test_argument_intA_value_1D(self):
        a = numpy.array([1, 2, 3], dtype=numpy.int32)
        b = core_testpy.identity_intA_value(a)
        assert (a == b).all()

    def test_argument_intA_value_2D(self):
        a = numpy.array([[1, 2, 3], [4, 5, 6]], dtype=numpy.int32)
        b = core_testpy.identity_intA_value(a)
        assert (a == b).all()

    def test_argument_intA_reference_1D(self):
        a = numpy.array([1, 2, 3], dtype=numpy.int32)
        b = core_testpy.identity_intA_reference(a)
        assert (a == b).all()

    def test_argument_intA_reference_2D(self):
        a = numpy.array([[1, 2, 3], [4, 5, 6]], dtype=numpy.int32)
        b = core_testpy.identity_intA_reference(a)
        assert (a == b).all()

    def test_argument_intA_pointer_1D(self):
        a = numpy.array([1, 2, 3], dtype=numpy.int32)
        b = core_testpy.identity_intA_pointer(a)
        assert (a == b).all()

    def test_argument_intA_pointer_2D(self):
        a = numpy.array([[1, 2, 3], [4, 5, 6]], dtype=numpy.int32)
        b = core_testpy.identity_intA_pointer(a)
        assert (a == b).all()

    def test_argument_uintA_value_1D(self):
        a = numpy.array([1, 2, 3], dtype=numpy.uint32)
        b = core_testpy.identity_uintA_value(a)
        assert (a == b).all()

    def test_argument_uintA_value_2D(self):
        a = numpy.array([[1, 2, 3], [4, 5, 6]], dtype=numpy.uint32)
        b = core_testpy.identity_uintA_value(a)
        assert (a == b).all()

    def test_argument_uintA_reference_1D(self):
        a = numpy.array([1, 2, 3], dtype=numpy.uint32)
        b = core_testpy.identity_uintA_reference(a)
        assert (a == b).all()

    def test_argument_uintA_reference_2D(self):
        a = numpy.array([[1, 2, 3], [4, 5, 6]], dtype=numpy.uint32)
        b = core_testpy.identity_uintA_reference(a)
        assert (a == b).all()

    def test_argument_uintA_pointer_1D(self):
        a = numpy.array([1, 2, 3], dtype=numpy.uint32)
        b = core_testpy.identity_uintA_pointer(a)
        assert (a == b).all()

    def test_argument_uintA_pointer_2D(self):
        a = numpy.array([[1, 2, 3], [4, 5, 6]], dtype=numpy.uint32)
        b = core_testpy.identity_uintA_pointer(a)
        assert (a == b).all()

    def test_overloading(self):
        a = numpy.array([1, 2, 3])
        b = core_testpy.test_overloading(a)
        c = core_testpy.test_overloading(.1)
        assert (a == b).all()
        assert c.shape == (1, )
        assert c.size == 1
        assert abs(c[0] - .1) < 0.01

    def test_member_out(self):
        t = core_testpy.TestClass()
        a = t.a_val
        p = t.a_poi
        assert a.shape == (2, )
        assert abs(a[0] - 1.2) < 0.01
        assert abs(a[1] - 3.4) < 0.01
        assert p.shape == (2, )
        assert abs(p[0] - 9.0) < 0.01
        assert abs(p[1] - 1.2) < 0.01

    def test_member_in(self):
        t = core_testpy.TestClass()
        t.a_val = numpy.array([3.4, 5.6])
        t.a_poi = numpy.array([7.8, 9.0])
        v = t.get_value()
        p = t.get_pointer()
        assert abs(v[0] - 3.4) < 0.01
        assert abs(v[1] - 5.6) < 0.01
        assert abs(p[0] - 7.8) < 0.01
        assert abs(p[1] - 9.0) < 0.01

    def test_member_set_value(self):
        t = core_testpy.TestClass()
        t.a_val = numpy.array([[1.2, 3.4], [5.6, 7.8]])

        t.a_val[1, 0] = 4.3  # here we shouldn't work on a copy of the data but
                             # on the original pointer. But then we get
                             # problems on different places. So far this test
                             # fails and I don't know a solution :-(

        assert abs(t.a_val[0, 0] - 1.2) < 0.1
        assert abs(t.a_val[0, 1] - 3.4) < .01
        assert abs(t.a_val[1, 0] - 4.3) < .01
        assert abs(t.a_val[1, 1] - 7.8) < .01

    def test_argout(self):
        a = numpy.array([1])
        a = core_testpy.argout_test(a)
        assert abs(a[0] - 1.2) < 0.01
        assert abs(a[1] - 3.4) < 0.01

    def test_multi_argout(self):
        a1 = numpy.array([1])
        a2 = numpy.array([1])
        (a1, a2) = core_testpy.multi_argout_test(a1, a2)
        assert abs(a1[0] - 1.2) < 0.01
        assert abs(a1[1] - 3.4) < 0.01
        assert abs(a2[0] - 5.6) < 0.01
        assert abs(a2[1] - 7.8) < 0.01

    def test_argout_plus_return(self):
        a = numpy.array([1])
        (r, a) = core_testpy.argout_test_plus_return(a)
        assert abs(a[0] - 1.2) < 0.01
        assert abs(a[1] - 3.4) < 0.01
        assert r == 1

    def test_mixed_argout(self):
        a1 = numpy.array([1])
        a2 = numpy.array([1])
        (r, a1, a2) = core_testpy.argout_test_mixed(1.2, a1, 3, a2)
        assert abs(a1[0] - 1.2) < 0.01
        assert abs(a1[1] - 3.4) < 0.01
        assert abs(a2[0] - 5.6) < 0.01
        assert abs(a2[1] - 7.8) < 0.01
        assert r == 1


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
        assert abs(a[0, 0] - 1.) < 0.01
        assert abs(a[0, 1] - 1.) < 0.01
        assert abs(a[1, 0] - 1.) < 0.01
        assert abs(a[1, 1] - 1.) < 0.01

    def test_creation_of_matrix_of_zeros(self):
        a = corepy.zeros(2, 3)
        # check dimensions
        assert a.size == 6
        assert a.shape == (2, 3)
        # check each value
        assert abs(a[0, 0] - 0.) < 0.01
        assert abs(a[0, 1] - 0.) < 0.01
        assert abs(a[0, 2] - 0.) < 0.01
        assert abs(a[1, 0] - 0.) < 0.01
        assert abs(a[1, 1] - 0.) < 0.01
        assert abs(a[1, 2] - 0.) < 0.01

    def test_creation_of_identity_matrix(self):
        a = corepy.eye(2, 2)
        assert a.shape == (2, 2)
        # check dimensions
        assert a.size == 4
        # check each value
        assert abs(a[0, 0] - 1.) < 0.01
        assert abs(a[0, 1] - 0.) < 0.01
        assert abs(a[1, 0] - 0.) < 0.01
        assert abs(a[1, 1] - 1.) < 0.01


class TestList_SWIGTypemaps:
    def test_return_arrL(self):
        arrl = core_testpy.return_arrL()
        arr = core_testpy.return_arr()
        arr = arr.reshape(4)
        assert (arrl[0] == arr).all()

    def test_argument_arrL_value(self):
        arrl = core_testpy.return_arrL()
        cp = core_testpy.identity_arrL_value(arrl)
        assert (cp[0] == arrl[0]).all()

    def test_list_of_pointer(self):
        t1 = core_testpy.ListTest()
        t1.d = 10
        t2 = core_testpy.ListTest()
        t2.d = 20

        test_list = core_testpy.id_list_test([t1, t2])

        assert test_list[0].d == 10
        assert test_list[1].d == 20

    def helper_new_object_copy(self):
        t1 = core_testpy.ListTest()
        t1.thisown = False  # we want to keep the object after python
                            # destruction. But now we must handle the
                            # destruction.
        t1.d = 10
        t2 = core_testpy.ListTest()
        t2.thisown = False
        t2.d = 20

        c = core_testpy.TestClass()
        c.a_list = [t1, t2]
        return c

    def test_new_object_copy(self):
        c = self.helper_new_object_copy()
        assert c.a_list[0].d == 10
        assert c.a_list[1].d == 20

    def test_pointerlist_member(self):
        c = core_testpy.TestClass()

        t1 = core_testpy.ListTest()
        t1.d = 10
        t2 = core_testpy.ListTest()
        t2.d = 20

        c.a_list = [t1, t2]

        assert c.a_list[0].d == 10
        assert c.a_list[1].d == 20

    def test_member_set_value(self):
        c = core_testpy.TestClass()
        t = core_testpy.ListTest()
        t.d = 10
        c.a_list.append(t)

        assert c.a_list[0].d == 10
