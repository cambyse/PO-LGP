%module listpy

%inline %{
#include <Core/array.h>
struct TestClass {
  double d;
  int i;
};
%}

//===========================================================================
// List Wrapper
//===========================================================================
// MT::Array is also used as list datatype. This is kind of a functional
// overload. However, we translate the list-like types to normal python Lists,
// whereas we translate arr, intA and uintA to their corresponding numpy array.
// We could translate everything to Lists, but the we would loose the shape
// information.

%fragment("asMTArrayList", "header") {
  template<class T>
  int asMTArrayList(MT::Array<T>& list, PyObject* pylist, swig_type_info* swigtype) {
    list.resize(PyList_Size(pylist));
    for(uint i=0; i<PyList_Size(pylist); ++i) {
      T obj;
      int res = SWIG_ConvertPtr(PyList_GetItem(pylist, i), (void**) &obj, swigtype, 0);
      if(SWIG_IsOK(res)) {
        list(i) = obj;
      }
      else {
        PyErr_SetString(PyExc_TypeError, "Not a list of compatible objects.");
        return 0;
      }
    } 
    return 1;
  }
}

%define %List_Typemap(Type)

%typemap(in, fragment="asMTArrayList") MT::Array<Type> {
  if(PyList_Check($input)) {
    if(!asMTArrayList($1, $input, $descriptor(Type))) return NULL;
  }
  else {
    // the typecheck typemap should ensure, that this never happens
    PyErr_SetString(PyExc_TypeError, "Not a list");
    return NULL;
  }
}

%typemap(in, fragment="asMTArrayList") MT::Array<Type> & {
  if(PyList_Check($input)) {
    $1 = new MT::Array<Type>;
    if(!asMTArrayList(*$1, $input, $descriptor(Type))) return NULL;
  }
  else {
    // the typecheck typemap should ensure, that this never happens
    PyErr_SetString(PyExc_TypeError, "Not a list");
    return NULL;
  }
}

%typemap(in, fragment="asMTArrayList") MT::Array<Type> * {
  if(PyList_Check($input)) {
    $1 = new MT::Array<Type>;
    if(!asMTArrayList(*$1, $input, $descriptor(Type))) return NULL;
  }
  else {
    // the typecheck typemap should ensure, that this never happens
    PyErr_SetString(PyExc_TypeError, "Not a list");
    return NULL;
  }
}

%typemap(out) MT::Array<Type> {
  $result = PyList_New($1.N);
  for(uint i=0; i<$1.N; ++i) {
    PyObject* obj = SWIG_NewPointerObj($1(i), $descriptor(Type), 0);
    PyList_SetItem($result, i, obj);
  }
}

%typemap(freearg) MT::Array<Type> * {
  delete $input;
}

%typemap(freearg) MT::Array<Type> & {
  delete $input;
}

%typemap(typecheck) MT::Array<Type> {
  if (PyList_Check($input)) {
    Type obj;
    int res = SWIG_ConvertPtr(PyList_GetItem($input, 0), (void**) &obj, $descriptor(Type), 0);
    if (SWIG_IsOK(res)) $1 = 1;
    else $1 = 0;
  }
  else $1 = 0;
}

%enddef

%List_Typemap(TestClass*);

%inline %{
void test_bodylist(MT::Array<TestClass*> list) {
  cout << list(0)->d << endl;
}
void test_bodylist(double d) {
  cout << d << "d" << endl;
}

MT::Array<TestClass*> test_return() {
  TestClass *t = new TestClass();
  t->d = 8.123;
  MT::Array<TestClass*> a;
  a.append(t);
  return a;
}

%}
