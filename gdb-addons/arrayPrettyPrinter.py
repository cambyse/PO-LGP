import gdb
class ArrayPrinter:
    def __init__(self, val):
        self.val = val

    def to_string(self):
        output = '\n'
        output += 'N  = ' + str(self.val['N']) + '\n'
        output += 'nd = ' + str(self.val['nd']) + '\n'
        output += 'd0 = ' + str(self.val['d0']) + '\n'
        output += 'd1 = ' + str(self.val['d1']) + '\n'
        output += 'd2 = ' + str(self.val['d2']) + '\n'
				output += 'data: \n'
				output += ' [ '
				if self.val['nd'] == 1:
				    for i in range(self.val['d0']):
				         output += ' ' + str(self.val['p'])
				for i in range(self.val['d2']):
						output += ' [ '
            for j in range(self.val['d1']):
                for k in range(self.val['d0']):
                      

        return output

def build_mlr_pretty_printers():
    pp = gdb.printing.RegexpCollectionPrettyPrinter("mlr")
    pp.add_printer('MT::Array', '^MT::Array<.*>$', ArrayPrinter)
    return pp

gdb.printing.register_pretty_printer(gdb.current_objfile(), build_mlr_pretty_printers())
