import orspy


class TestShapes(object):

    def test_size(self):
        shape = orspy.Shape()

        shape.set_size(0, 0, 0, 0)
        assert shape.get_size(0) == 0.
        assert shape.get_size(1) == 0.
        assert shape.get_size(2) == 0.
        assert shape.get_size(3) == 0.

        shape.set_size(1., 2., 3., 4.)
        assert shape.get_size(0) == 1.
        assert shape.get_size(1) == 2.
        assert shape.get_size(2) == 3.
        assert shape.get_size(3) == 4.

    def test_color(self):
        shape = orspy.Shape()
        shape.set_color(.1, .2, .3)
        assert shape.get_color(0) == .1
        assert shape.get_color(1) == .2
        assert shape.get_color(2) == .3
