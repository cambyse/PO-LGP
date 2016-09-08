# This file was automatically generated by SWIG (http://www.swig.org).
# Version 2.0.11
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.





from sys import version_info
if version_info >= (2,6,0):
    def swig_import_helper():
        from os.path import dirname
        import imp
        fp = None
        try:
            fp, pathname, description = imp.find_module('_swig', [dirname(__file__)])
        except ImportError:
            import _swig
            return _swig
        if fp is not None:
            try:
                _mod = imp.load_module('_swig', fp, pathname, description)
            finally:
                fp.close()
            return _mod
    _swig = swig_import_helper()
    del swig_import_helper
else:
    import _swig
del version_info
try:
    _swig_property = property
except NameError:
    pass # Python < 2.2 doesn't have 'property'.
def _swig_setattr_nondynamic(self,class_type,name,value,static=1):
    if (name == "thisown"): return self.this.own(value)
    if (name == "this"):
        if type(value).__name__ == 'SwigPyObject':
            self.__dict__[name] = value
            return
    method = class_type.__swig_setmethods__.get(name,None)
    if method: return method(self,value)
    if (not static):
        self.__dict__[name] = value
    else:
        raise AttributeError("You cannot add attributes to %s" % self)

def _swig_setattr(self,class_type,name,value):
    return _swig_setattr_nondynamic(self,class_type,name,value,0)

def _swig_getattr(self,class_type,name):
    if (name == "thisown"): return self.this.own()
    method = class_type.__swig_getmethods__.get(name,None)
    if method: return method(self)
    raise AttributeError(name)

def _swig_repr(self):
    try: strthis = "proxy of " + self.this.__repr__()
    except: strthis = ""
    return "<%s.%s; %s >" % (self.__class__.__module__, self.__class__.__name__, strthis,)

try:
    _object = object
    _newclass = 1
except AttributeError:
    class _object : pass
    _newclass = 0


class SwigPyIterator(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, SwigPyIterator, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, SwigPyIterator, name)
    def __init__(self, *args, **kwargs): raise AttributeError("No constructor defined - class is abstract")
    __repr__ = _swig_repr
    __swig_destroy__ = _swig.delete_SwigPyIterator
    __del__ = lambda self : None;
    def value(self): return _swig.SwigPyIterator_value(self)
    def incr(self, n=1): return _swig.SwigPyIterator_incr(self, n)
    def decr(self, n=1): return _swig.SwigPyIterator_decr(self, n)
    def distance(self, *args): return _swig.SwigPyIterator_distance(self, *args)
    def equal(self, *args): return _swig.SwigPyIterator_equal(self, *args)
    def copy(self): return _swig.SwigPyIterator_copy(self)
    def next(self): return _swig.SwigPyIterator_next(self)
    def __next__(self): return _swig.SwigPyIterator___next__(self)
    def previous(self): return _swig.SwigPyIterator_previous(self)
    def advance(self, *args): return _swig.SwigPyIterator_advance(self, *args)
    def __eq__(self, *args): return _swig.SwigPyIterator___eq__(self, *args)
    def __ne__(self, *args): return _swig.SwigPyIterator___ne__(self, *args)
    def __iadd__(self, *args): return _swig.SwigPyIterator___iadd__(self, *args)
    def __isub__(self, *args): return _swig.SwigPyIterator___isub__(self, *args)
    def __add__(self, *args): return _swig.SwigPyIterator___add__(self, *args)
    def __sub__(self, *args): return _swig.SwigPyIterator___sub__(self, *args)
    def __iter__(self): return self
SwigPyIterator_swigregister = _swig.SwigPyIterator_swigregister
SwigPyIterator_swigregister(SwigPyIterator)

class VecDouble(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, VecDouble, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, VecDouble, name)
    __repr__ = _swig_repr
    def iterator(self): return _swig.VecDouble_iterator(self)
    def __iter__(self): return self.iterator()
    def __nonzero__(self): return _swig.VecDouble___nonzero__(self)
    def __bool__(self): return _swig.VecDouble___bool__(self)
    def __len__(self): return _swig.VecDouble___len__(self)
    def pop(self): return _swig.VecDouble_pop(self)
    def __getslice__(self, *args): return _swig.VecDouble___getslice__(self, *args)
    def __setslice__(self, *args): return _swig.VecDouble___setslice__(self, *args)
    def __delslice__(self, *args): return _swig.VecDouble___delslice__(self, *args)
    def __delitem__(self, *args): return _swig.VecDouble___delitem__(self, *args)
    def __getitem__(self, *args): return _swig.VecDouble___getitem__(self, *args)
    def __setitem__(self, *args): return _swig.VecDouble___setitem__(self, *args)
    def append(self, *args): return _swig.VecDouble_append(self, *args)
    def empty(self): return _swig.VecDouble_empty(self)
    def size(self): return _swig.VecDouble_size(self)
    def clear(self): return _swig.VecDouble_clear(self)
    def swap(self, *args): return _swig.VecDouble_swap(self, *args)
    def get_allocator(self): return _swig.VecDouble_get_allocator(self)
    def begin(self): return _swig.VecDouble_begin(self)
    def end(self): return _swig.VecDouble_end(self)
    def rbegin(self): return _swig.VecDouble_rbegin(self)
    def rend(self): return _swig.VecDouble_rend(self)
    def pop_back(self): return _swig.VecDouble_pop_back(self)
    def erase(self, *args): return _swig.VecDouble_erase(self, *args)
    def __init__(self, *args): 
        this = _swig.new_VecDouble(*args)
        try: self.this.append(this)
        except: self.this = this
    def push_back(self, *args): return _swig.VecDouble_push_back(self, *args)
    def front(self): return _swig.VecDouble_front(self)
    def back(self): return _swig.VecDouble_back(self)
    def assign(self, *args): return _swig.VecDouble_assign(self, *args)
    def resize(self, *args): return _swig.VecDouble_resize(self, *args)
    def insert(self, *args): return _swig.VecDouble_insert(self, *args)
    def reserve(self, *args): return _swig.VecDouble_reserve(self, *args)
    def capacity(self): return _swig.VecDouble_capacity(self)
    __swig_destroy__ = _swig.delete_VecDouble
    __del__ = lambda self : None;
VecDouble_swigregister = _swig.VecDouble_swigregister
VecDouble_swigregister(VecDouble)

class LockboxSwig(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, LockboxSwig, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, LockboxSwig, name)
    __repr__ = _swig_repr
    __swig_setmethods__["s"] = _swig.LockboxSwig_s_set
    __swig_getmethods__["s"] = _swig.LockboxSwig_s_get
    if _newclass:s = _swig_property(_swig.LockboxSwig_s_get, _swig.LockboxSwig_s_set)
    def __init__(self): 
        this = _swig.new_LockboxSwig()
        try: self.this.append(this)
        except: self.this = this
    __swig_destroy__ = _swig.delete_LockboxSwig
    __del__ = lambda self : None;
    def testJoint(self, *args): return _swig.LockboxSwig_testJoint(self, *args)
    def getJointPosition(self, *args): return _swig.LockboxSwig_getJointPosition(self, *args)
    def testArray(self, *args): return _swig.LockboxSwig_testArray(self, *args)
LockboxSwig_swigregister = _swig.LockboxSwig_swigregister
LockboxSwig_swigregister(LockboxSwig)

# This file is compatible with both classic and new-style classes.


