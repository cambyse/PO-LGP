/*

  We need:

  -- a clean interface for user's (method developer's) for coding methods/algorithms

  -- this should be independent of the ``middleware''/biros/ROS/whatever

 */

#include <MT/array.h>
#include <MT/registry.h>
#include <stddef.h>

template<class T, class P>
struct UnitRegistry_typed:Item{

};

struct Module:TypeBase{
  Module(){}
  virtual ~Module(){};
  virtual void step() = 0;
};


template<class T>
struct VariableAccess{
  T *var;
  VariableAccess(): var(NULL){}
  T& get(Module *m){ CHECK(var,"access to variable "<<typeid(T).name()<<" uninitialized"); return *var; }
  int set(const T& x, Module *m){ CHECK(var,"access to variable "<<typeid(T).name()<<" uninitialized"); *var=x; return 0; }
};


struct VariableRegistration{
  const char* name;
  uint offset;
  const std::type_info* typeinfo;
  VariableRegistration(const char* _name, uint _offset, const std::type_info* _typeinfo)
    :name(_name), offset(_offset), typeinfo(_typeinfo) {}
  void write(ostream& os) const{ os <<name <<' ' <<offset <<' ' <<typeinfo->name(); }
};
stdOutPipe(VariableRegistration);


#define VAR(type, name) \
  struct VariableAccess< type > name##_access; \
  inline void name##_forceRegistration(){ staticRegistrator.forceSub<type>(); } \
  inline type& get_##name(){ return name##_access.get(this); } \
  inline int  set_##name(const type& _x){ return name##_access.set(_x,this); }


//#define PARAM (type, name, default) \
//  ParameterAccess_typed<type, default> name##_access; \
//  inline type get_##name(){ return name##_access.get(); }

#define MODULE_BASE(name) \
  Registrator<name, void>, Module



