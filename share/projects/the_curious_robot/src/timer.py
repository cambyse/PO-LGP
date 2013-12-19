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
        self.print_function = print_function

    def __enter__(self):
        self.tstart = time.time()

    def __exit__(self, type, value, traceback):
        text = "%s -- elapsed %s" % (self.msg, time.time() - self.tstart)

        if self.print_function:
            self.print_function(text)
        else:
            print text
