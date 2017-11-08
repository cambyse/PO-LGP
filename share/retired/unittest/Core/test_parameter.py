from corepy import (getDoubleParameter, getIntParameter, getArrayParameter,
                    getStringParameter)

import numpy as np


class TestParameterGetters():
    """
    Note: this testclass also tests the magic method __getitem__.
    """

    def test_double_getter(self):
        d = getDoubleParameter("d")
        assert d == 1.2

    def test_int_getter(self):
        i = getIntParameter("i")
        assert i == 3

    def test_arr_getter(self):
        a = getArrayParameter("a")
        assert a[0, 0] == 4.5
        assert a[0, 1] == 6.7
        assert a[1, 0] == 8.9
        assert a[1, 1] == 0.1

    def test_string_getter(self):
        s = getStringParameter("s")
        assert s == "unittest"

    def test_double_default(self):
        p = getDoubleParameter("notReal", .1)
        assert p == .1

    def test_int_default(self):
        p = getIntParameter("notreal", 42)
        assert p == 42

    def test_arr_default(self):
        p = getArrayParameter("notreal", np.array([1.2, 3.4]))
        assert p[0] == 1.2
        assert p[1] == 3.4

    def test_string_default(self):
        p = getStringParameter("notreal", "notSet")
        assert p == "notSet"

    def test_double_ignore_default(self):
        d = getDoubleParameter("d", 3.4)
        assert d == 1.2

    def test_int_ignore_default(self):
        i = getIntParameter("i", 42)
        assert i == 3

    def test_arr_ignore_default(self):
        a = getArrayParameter("a", np.array([1.2, 3.4]))
        assert a[0, 0] == 4.5
        assert a[0, 1] == 6.7
        assert a[1, 0] == 8.9
        assert a[1, 1] == 0.1

    def test_string_ignore_default(self):
        s = getStringParameter("s", "notSet")
        assert s == "unittest"
