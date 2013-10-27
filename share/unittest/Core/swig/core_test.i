%module core_testpy

%include "Core/array_typemaps.i"
%import "Core/core.i"
//===========================================================================
// Some test functions. TODO: move to some test-lib?
//===========================================================================

%inline %{

struct TestClass {
  MT::Array<double> a_val;
  MT::Array<double>* a_poi;
  TestClass() : a_val({1.2, 3.4}), a_poi(new arr({9.0, 1.2})) {};

  MT::Array<double> get_value() { return a_val; }
  MT::Array<double>* get_pointer() { return a_poi; }
};


MT::Array<double> identity_arr_value(MT::Array<double> INPUT) {
  return INPUT;
}

const MT::Array<double> identity_const_arr_value(const MT::Array<double> INPUT) {
  return INPUT;
}

MT::Array<double> identity_arr_reference(MT::Array<double> &INPUT) {
  return INPUT;
}

const MT::Array<double> identity_const_arr_reference(const MT::Array<double> &INPUT) {
  return INPUT;
}

MT::Array<double> identity_arr_pointer(MT::Array<double> *INPUT) {
  return *INPUT;
}

const MT::Array<double> identity_const_arr_pointer(const MT::Array<double> *INPUT) {
  return *INPUT;
}

MT::Array<int> identity_intA_value(MT::Array<int> INPUT) {
  return INPUT;
}

MT::Array<int> identity_intA_reference(MT::Array<int> &INPUT) {
  return INPUT;
}

MT::Array<int> identity_intA_pointer(MT::Array<int> *INPUT) {
  return *INPUT;
}

MT::Array<uint> identity_uintA_value(MT::Array<uint> INPUT) {
  return INPUT;
}

MT::Array<uint> identity_uintA_reference(MT::Array<uint> &INPUT) {
  return INPUT;
}

MT::Array<uint> identity_uintA_pointer(MT::Array<uint> *INPUT) {
  return *INPUT;
}

typedef MT::Array<double> arr;

const arr test_typedefs(const arr& a) {
  return a;
}

const arr test_overloading(double x) {
  return {x};
}

const arr test_overloading(arr& x) {
  return x;
}

const arr test_overloading(arr& x, double y) {
  return y*x;
}

MT::Array<double> return_arr() {
  arr t = { 1.2, 3.4, 5.6, 7.8};
  t.reshape(2,2);
  return t;
}

MT::Array<int> return_intA() {
  intA t = { -1, 3, -5, 7};
  t.reshape(2,2);
  return t;
}

MT::Array<uint> return_uintA() {
  uintA t = { 1, 3, 5, 7};
  t.reshape(2,2);
  return t;
}

arrL return_arrL() {
  arr *a = new arr({1.2, 3.4, 5.6, 7.8});
  arrL l;
  l.append(a);
  return l;
}

arrL identity_arrL_value(arrL in) {
  return in;
}

arrL identity_arrL_reference(arrL& in) {
  return in;
}

arrL identity_arrL_pointer(arrL* in) {
  return *in;
}
%}  

//===========================================================================
