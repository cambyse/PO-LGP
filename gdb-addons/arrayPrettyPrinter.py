import gdb
import itertools


# Try to use the new-style pretty-printing if available.
_use_gdb_pp = True
try:
    import gdb.printing
except ImportError:
    _use_gdb_pp = False

class ArrayPrinter:
    def __init__(self, val):
        self.val = val

    def to_string(self):
        output = '[ '
        nd = self.val['nd']
        if nd == 0:
          output += 'empty array ]'
        if nd > 0:
          output += str(self.val['d0'])
        if nd > 1:
          output += ' x '
          output += str(self.val['d1'])
        if nd > 2:
          output += ' x '
          output += str(self.val['d2'])
        if nd == 1:
            output += ' vector ]'
        if nd == 2:
            output += ' matrix ]'
        if nd == 3:
            output += ' tensor ]'

        return output
    def children(self):
        return ;

    class _iterator:
        def __init__(self, p, nd, d0, d1, d2):
            self.p = p
            self.nd = nd
            self.d0 = d0
            self.d1 = d1
            self.d2 = d2
            self.i = 0
            self.j = 0
            self.k = 0

        def __iter__(self):
            return self

        def next(self):
            if self.i == self.d0 or self.d0 * self.d1 * self.d2 > 200: 
                raise StopIteration
            if self.nd == 1:
                position = self.i
                output = '[%d]' % self.i
                self.i += 1
            elif self.nd == 2:
                position = self.i*self.d1 + self.j
                output = '[%(i)d,%(j)d]' % {"i": self.i, "j": self.j}
                if self.j == 0:
                    output = '\n' + output
                self.j += 1
                if self.j == self.d1:
                    self.j = 0
                    self.i += 1
            elif self.nd == 3:
                position = self.i*self.d1*self.d2 + self.j*self.d2 + self.k
                output = '[%(i)d,%(j)d, %(k)d]' % {"i": self.i, "j": self.j, "k": self.k}
                if self.j == 0 and self.k == 0 and self.i > 0:
                    output = ' ]\n[ ' + output
                elif self.k == 0:
                    output = '\n' + output
                self.k += 1
                if self.k == self.d2:
                    self.k = 0
                    self.j += 1
                    if self.j == self.d1:
                        self.j = 0
                        self.i += 1
            value = float((self.p + position).dereference())
            value = format(value, '.4f')
            return (output, value) 



    def children(self):
        return self._iterator(self.val['p'], self.val['nd'], self.val['d0'], self.val['d1'], self.val['d2'])

        

def build_mlr_pretty_printers():
    if _use_gdb_pp:
        pp = gdb.printing.RegexpCollectionPrettyPrinter("mlr")
        pp.add_printer('MT::Array', '^MT::Array<.*>$', ArrayPrinter)
    return pp

if _use_gdb_pp:
    gdb.printing.register_pretty_printer(gdb.current_objfile(), build_mlr_pretty_printers())
else:
    gdb.pretty_printers.append(ArrayPrinter)
