/*

  We need:

  -- a clean interface for user's (method developer's) for coding methods/algorithms

  -- this should be independent of the ``middleware''/biros/ROS/whatever

 */

#include <MT/array.h>
#include <MT/registry.h>

struct Module;
struct VariableAccess;
typedef MT::Array<VariableAccess*> VariableAccessL;

struct VariableAccess{
};


struct Module{
  VariableAccessL vars;

  virtual void open(){};
  virtual void step() = 0;
  virtual void close(){};
};


template<class T>
struct VariableAccess_typed:VariableAccess{
  Module *m;
  T *var;
  VariableAccess_typed(Module *_m):m(_m), var(NULL){
    m->vars.append(this);
  }
  T get(){ T x(*var); return x; }
  int set(const T& x){ *var=x; return 0; }
};

template<class M>
struct ModuleRegistration{
  ModuleRegistration(){
  }
};

#define VAR(type, name) \
  VariableAccess_typed<type> name##_access; \
  inline type get_##name(){ return name##_access.get(); } \
  inline int  set_##name(const type& _x){ return name##_access.set(_x); }

//#define PARAM (type, name, default) \
//  ParameterAccess_typed<type, default> name##_access; \
//  inline type get_##name(){ return name##_access.get(); }

#define REGISTER_MODULE (M) \
  REGISTER_DERIVED_TYPE(M, Module) \
  //REGISTER_ITEM_2KEYS(M, typeid(M).name(), ModuleRegistration<M>())

void dumpAllRegisteredModules(){
  listWrite(reg_findDerived<Module>(), cout);
}
