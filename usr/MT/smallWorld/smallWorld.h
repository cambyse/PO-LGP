/*

  We need:

  -- a clean interface for user's (method developer's) for coding methods/algorithms

  -- this should be independent of the ``middleware''/biros/ROS/whatever

 */

#include <MT/array.h>
#include <MT/registry.h>
#include <stddef.h>

#define STRINGS(s0, s1) ARRAY<MT::String>(MT::String(s0), MT::String(s1))

struct Module;
struct VariableAccess;
typedef MT::Array<VariableAccess*> VariableAccessL;

struct VariableAccess{
  const char *name;
  virtual const std::type_info& typeinfo() const = 0;
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
  VariableAccess_typed(Module *_m, const char *_name):m(_m), var(NULL){
    name=_name;
    m->vars.append(this);
  }
  virtual const std::type_info& typeinfo() const { return typeid(T); }
  T get(){ T x(*var); return x; }
  int set(const T& x){ *var=x; return 0; }
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


template<class M>
struct ModuleRegistration{
  ModuleRegistration(const char* name, MapGraph& registry){
    Item *it = registry.append(STRINGS("module", name), *this);
    //briefly generate a module to query vars, etc
    M m;
    //cout <<"Registering Modul '" <<typeid(M).name() <<"' Variables:" <<endl;
    for_list_(VariableAccess, v, m.vars){
      //cout <<"  VAR " <<v->name <<' ' <<v->typeinfo().name() <<endl;
      registry.append(STRINGS("module_var", v->name), ARRAY(it), VariableRegistration(v->name, (uint)v - (uint)&m, &v->typeinfo()));
    }
  }
  void write(ostream& os) const{ cout <<"TODO"; }
};
template<class M> stdOutPipe(ModuleRegistration<M>);

#define VAR(type, name) \
  VariableAccess_typed<type> name##_access; \
  inline type get_##name(){ return name##_access.get(); } \
  inline int  set_##name(const type& _x){ return name##_access.set(_x); }

#define VARc(name) \
  name##_access(this, #name)

//#define PARAM (type, name, default) \
//  ParameterAccess_typed<type, default> name##_access; \
//  inline type get_##name(){ return name##_access.get(); }

#define REGISTER_MODULE(M) \
  ModuleRegistration<M> M##_RegistryEntry(#M, registry());

void dumpAllRegisteredModules(){
  listWrite(reg_findDerived<Module>(), cout);
}
