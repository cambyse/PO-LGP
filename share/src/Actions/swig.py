# This file was automatically generated by SWIG (http://www.swig.org).
# Version 2.0.12
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

class doubleV(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, doubleV, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, doubleV, name)
    __repr__ = _swig_repr
    def iterator(self): return _swig.doubleV_iterator(self)
    def __iter__(self): return self.iterator()
    def __nonzero__(self): return _swig.doubleV___nonzero__(self)
    def __bool__(self): return _swig.doubleV___bool__(self)
    def __len__(self): return _swig.doubleV___len__(self)
    def pop(self): return _swig.doubleV_pop(self)
    def __getslice__(self, *args): return _swig.doubleV___getslice__(self, *args)
    def __setslice__(self, *args): return _swig.doubleV___setslice__(self, *args)
    def __delslice__(self, *args): return _swig.doubleV___delslice__(self, *args)
    def __delitem__(self, *args): return _swig.doubleV___delitem__(self, *args)
    def __getitem__(self, *args): return _swig.doubleV___getitem__(self, *args)
    def __setitem__(self, *args): return _swig.doubleV___setitem__(self, *args)
    def append(self, *args): return _swig.doubleV_append(self, *args)
    def empty(self): return _swig.doubleV_empty(self)
    def size(self): return _swig.doubleV_size(self)
    def clear(self): return _swig.doubleV_clear(self)
    def swap(self, *args): return _swig.doubleV_swap(self, *args)
    def get_allocator(self): return _swig.doubleV_get_allocator(self)
    def begin(self): return _swig.doubleV_begin(self)
    def end(self): return _swig.doubleV_end(self)
    def rbegin(self): return _swig.doubleV_rbegin(self)
    def rend(self): return _swig.doubleV_rend(self)
    def pop_back(self): return _swig.doubleV_pop_back(self)
    def erase(self, *args): return _swig.doubleV_erase(self, *args)
    def __init__(self, *args): 
        this = _swig.new_doubleV(*args)
        try: self.this.append(this)
        except: self.this = this
    def push_back(self, *args): return _swig.doubleV_push_back(self, *args)
    def front(self): return _swig.doubleV_front(self)
    def back(self): return _swig.doubleV_back(self)
    def assign(self, *args): return _swig.doubleV_assign(self, *args)
    def resize(self, *args): return _swig.doubleV_resize(self, *args)
    def insert(self, *args): return _swig.doubleV_insert(self, *args)
    def reserve(self, *args): return _swig.doubleV_reserve(self, *args)
    def capacity(self): return _swig.doubleV_capacity(self)
    __swig_destroy__ = _swig.delete_doubleV
    __del__ = lambda self : None;
doubleV_swigregister = _swig.doubleV_swigregister
doubleV_swigregister(doubleV)

class intV(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, intV, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, intV, name)
    __repr__ = _swig_repr
    def iterator(self): return _swig.intV_iterator(self)
    def __iter__(self): return self.iterator()
    def __nonzero__(self): return _swig.intV___nonzero__(self)
    def __bool__(self): return _swig.intV___bool__(self)
    def __len__(self): return _swig.intV___len__(self)
    def pop(self): return _swig.intV_pop(self)
    def __getslice__(self, *args): return _swig.intV___getslice__(self, *args)
    def __setslice__(self, *args): return _swig.intV___setslice__(self, *args)
    def __delslice__(self, *args): return _swig.intV___delslice__(self, *args)
    def __delitem__(self, *args): return _swig.intV___delitem__(self, *args)
    def __getitem__(self, *args): return _swig.intV___getitem__(self, *args)
    def __setitem__(self, *args): return _swig.intV___setitem__(self, *args)
    def append(self, *args): return _swig.intV_append(self, *args)
    def empty(self): return _swig.intV_empty(self)
    def size(self): return _swig.intV_size(self)
    def clear(self): return _swig.intV_clear(self)
    def swap(self, *args): return _swig.intV_swap(self, *args)
    def get_allocator(self): return _swig.intV_get_allocator(self)
    def begin(self): return _swig.intV_begin(self)
    def end(self): return _swig.intV_end(self)
    def rbegin(self): return _swig.intV_rbegin(self)
    def rend(self): return _swig.intV_rend(self)
    def pop_back(self): return _swig.intV_pop_back(self)
    def erase(self, *args): return _swig.intV_erase(self, *args)
    def __init__(self, *args): 
        this = _swig.new_intV(*args)
        try: self.this.append(this)
        except: self.this = this
    def push_back(self, *args): return _swig.intV_push_back(self, *args)
    def front(self): return _swig.intV_front(self)
    def back(self): return _swig.intV_back(self)
    def assign(self, *args): return _swig.intV_assign(self, *args)
    def resize(self, *args): return _swig.intV_resize(self, *args)
    def insert(self, *args): return _swig.intV_insert(self, *args)
    def reserve(self, *args): return _swig.intV_reserve(self, *args)
    def capacity(self): return _swig.intV_capacity(self)
    __swig_destroy__ = _swig.delete_intV
    __del__ = lambda self : None;
intV_swigregister = _swig.intV_swigregister
intV_swigregister(intV)

class stringV(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, stringV, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, stringV, name)
    __repr__ = _swig_repr
    def iterator(self): return _swig.stringV_iterator(self)
    def __iter__(self): return self.iterator()
    def __nonzero__(self): return _swig.stringV___nonzero__(self)
    def __bool__(self): return _swig.stringV___bool__(self)
    def __len__(self): return _swig.stringV___len__(self)
    def pop(self): return _swig.stringV_pop(self)
    def __getslice__(self, *args): return _swig.stringV___getslice__(self, *args)
    def __setslice__(self, *args): return _swig.stringV___setslice__(self, *args)
    def __delslice__(self, *args): return _swig.stringV___delslice__(self, *args)
    def __delitem__(self, *args): return _swig.stringV___delitem__(self, *args)
    def __getitem__(self, *args): return _swig.stringV___getitem__(self, *args)
    def __setitem__(self, *args): return _swig.stringV___setitem__(self, *args)
    def append(self, *args): return _swig.stringV_append(self, *args)
    def empty(self): return _swig.stringV_empty(self)
    def size(self): return _swig.stringV_size(self)
    def clear(self): return _swig.stringV_clear(self)
    def swap(self, *args): return _swig.stringV_swap(self, *args)
    def get_allocator(self): return _swig.stringV_get_allocator(self)
    def begin(self): return _swig.stringV_begin(self)
    def end(self): return _swig.stringV_end(self)
    def rbegin(self): return _swig.stringV_rbegin(self)
    def rend(self): return _swig.stringV_rend(self)
    def pop_back(self): return _swig.stringV_pop_back(self)
    def erase(self, *args): return _swig.stringV_erase(self, *args)
    def __init__(self, *args): 
        this = _swig.new_stringV(*args)
        try: self.this.append(this)
        except: self.this = this
    def push_back(self, *args): return _swig.stringV_push_back(self, *args)
    def front(self): return _swig.stringV_front(self)
    def back(self): return _swig.stringV_back(self)
    def assign(self, *args): return _swig.stringV_assign(self, *args)
    def resize(self, *args): return _swig.stringV_resize(self, *args)
    def insert(self, *args): return _swig.stringV_insert(self, *args)
    def reserve(self, *args): return _swig.stringV_reserve(self, *args)
    def capacity(self): return _swig.stringV_capacity(self)
    __swig_destroy__ = _swig.delete_stringV
    __del__ = lambda self : None;
stringV_swigregister = _swig.stringV_swigregister
stringV_swigregister(stringV)

class dict(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, dict, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, dict, name)
    __repr__ = _swig_repr
    def iterator(self): return _swig.dict_iterator(self)
    def __iter__(self): return self.iterator()
    def __nonzero__(self): return _swig.dict___nonzero__(self)
    def __bool__(self): return _swig.dict___bool__(self)
    def __len__(self): return _swig.dict___len__(self)
    def __iter__(self): return self.key_iterator()
    def iterkeys(self): return self.key_iterator()
    def itervalues(self): return self.value_iterator()
    def iteritems(self): return self.iterator()
    def __getitem__(self, *args): return _swig.dict___getitem__(self, *args)
    def __delitem__(self, *args): return _swig.dict___delitem__(self, *args)
    def has_key(self, *args): return _swig.dict_has_key(self, *args)
    def keys(self): return _swig.dict_keys(self)
    def values(self): return _swig.dict_values(self)
    def items(self): return _swig.dict_items(self)
    def __contains__(self, *args): return _swig.dict___contains__(self, *args)
    def key_iterator(self): return _swig.dict_key_iterator(self)
    def value_iterator(self): return _swig.dict_value_iterator(self)
    def __setitem__(self, *args): return _swig.dict___setitem__(self, *args)
    def asdict(self): return _swig.dict_asdict(self)
    def __init__(self, *args): 
        this = _swig.new_dict(*args)
        try: self.this.append(this)
        except: self.this = this
    def empty(self): return _swig.dict_empty(self)
    def size(self): return _swig.dict_size(self)
    def clear(self): return _swig.dict_clear(self)
    def swap(self, *args): return _swig.dict_swap(self, *args)
    def get_allocator(self): return _swig.dict_get_allocator(self)
    def begin(self): return _swig.dict_begin(self)
    def end(self): return _swig.dict_end(self)
    def rbegin(self): return _swig.dict_rbegin(self)
    def rend(self): return _swig.dict_rend(self)
    def count(self, *args): return _swig.dict_count(self, *args)
    def erase(self, *args): return _swig.dict_erase(self, *args)
    def find(self, *args): return _swig.dict_find(self, *args)
    def lower_bound(self, *args): return _swig.dict_lower_bound(self, *args)
    def upper_bound(self, *args): return _swig.dict_upper_bound(self, *args)
    __swig_destroy__ = _swig.delete_dict
    __del__ = lambda self : None;
dict_swigregister = _swig.dict_swigregister
dict_swigregister(dict)

class ActionSwigInterface(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, ActionSwigInterface, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, ActionSwigInterface, name)
    __repr__ = _swig_repr
    __swig_setmethods__["S"] = _swig.ActionSwigInterface_S_set
    __swig_getmethods__["S"] = _swig.ActionSwigInterface_S_get
    if _newclass:S = _swig_property(_swig.ActionSwigInterface_S_get, _swig.ActionSwigInterface_S_set)
    def __init__(self, setSignalHandler=True): 
        this = _swig.new_ActionSwigInterface(setSignalHandler)
        try: self.this.append(this)
        except: self.this = this
    __swig_destroy__ = _swig.delete_ActionSwigInterface
    __del__ = lambda self : None;
    def Cancel(self): return _swig.ActionSwigInterface_Cancel(self)
    def setVerbose(self, *args): return _swig.ActionSwigInterface_setVerbose(self, *args)
    def setFixBase(self, *args): return _swig.ActionSwigInterface_setFixBase(self, *args)
    def getShapeList(self): return _swig.ActionSwigInterface_getShapeList(self)
    def getBodyList(self): return _swig.ActionSwigInterface_getBodyList(self)
    def getJointList(self): return _swig.ActionSwigInterface_getJointList(self)
    def getQDim(self): return _swig.ActionSwigInterface_getQDim(self)
    def getV(self): return _swig.ActionSwigInterface_getV(self)
    def getBodyByName(self, *args): return _swig.ActionSwigInterface_getBodyByName(self, *args)
    def getShapeByName(self, *args): return _swig.ActionSwigInterface_getShapeByName(self, *args)
    def getJointByName(self, *args): return _swig.ActionSwigInterface_getJointByName(self, *args)
    def getQIndex(self, *args): return _swig.ActionSwigInterface_getQIndex(self, *args)
    def getForceLeft(self): return _swig.ActionSwigInterface_getForceLeft(self)
    def getForceRight(self): return _swig.ActionSwigInterface_getForceRight(self)
    def getSymbols(self): return _swig.ActionSwigInterface_getSymbols(self)
    def getSymbolInteger(self, *args): return _swig.ActionSwigInterface_getSymbolInteger(self, *args)
    def str2lit(self, *args): return _swig.ActionSwigInterface_str2lit(self, *args)
    def lit2str(self, *args): return _swig.ActionSwigInterface_lit2str(self, *args)
    def isTrue(self, *args): return _swig.ActionSwigInterface_isTrue(self, *args)
    def setFact(self, *args): return _swig.ActionSwigInterface_setFact(self, *args)
    def stopFact(self, *args): return _swig.ActionSwigInterface_stopFact(self, *args)
    def getFacts(self): return _swig.ActionSwigInterface_getFacts(self)
    def startActivity(self, *args): return _swig.ActionSwigInterface_startActivity(self, *args)
    def stopActivity(self, *args): return _swig.ActionSwigInterface_stopActivity(self, *args)
    def waitForCondition(self, *args): return _swig.ActionSwigInterface_waitForCondition(self, *args)
    def waitForOrCondition(self, *args): return _swig.ActionSwigInterface_waitForOrCondition(self, *args)
    def waitForAllCondition(self, *args): return _swig.ActionSwigInterface_waitForAllCondition(self, *args)
    def waitForQuitSymbol(self): return _swig.ActionSwigInterface_waitForQuitSymbol(self)
    def createNewSymbol(self, *args): return _swig.ActionSwigInterface_createNewSymbol(self, *args)
    def defineNewTaskSpaceControlAction(self, *args): return _swig.ActionSwigInterface_defineNewTaskSpaceControlAction(self, *args)
    def getRM(self): return _swig.ActionSwigInterface_getRM(self)
    def execScript(self, *args): return _swig.ActionSwigInterface_execScript(self, *args)
    def getFramePose(self, *args): return _swig.ActionSwigInterface_getFramePose(self, *args)
ActionSwigInterface_swigregister = _swig.ActionSwigInterface_swigregister
ActionSwigInterface_swigregister(ActionSwigInterface)

# This file is compatible with both classic and new-style classes.


