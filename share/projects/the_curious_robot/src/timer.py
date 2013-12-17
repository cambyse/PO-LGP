import time


class Timer(object):
    """
    Measure the time of any code with this nice context manager.

    Do it like this
    >>> with Timer('foo_stuff'):
    >>>     # do expensive stuff
    >>>     x = 5 * 9
    >>>     x = x**x

    """
    def __init__(self, name=None):
        self.name = name

    def __enter__(self):
        self.tstart = time.time()

    def __exit__(self, type, value, traceback):
        if self.name:
            print '[%s]' % self.name,
        print 'Elapsed: %s' % (time.time() - self.tstart)
