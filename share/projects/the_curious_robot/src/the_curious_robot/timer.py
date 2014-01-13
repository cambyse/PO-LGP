from __future__ import print_function
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
    def __init__(self, msg, print_function=None):
        self.msg = msg

        if print_function is None:
            self.print_function = print
        else:
            self.print_function = print_function

    def __enter__(self):
        self.tstart = time.time()

    def __exit__(self, type, value, traceback):
        text = "[TIMER] %s -- elapsed %s" % (
            self.msg, time.time() - self.tstart
        )

        self.print_function(text)
