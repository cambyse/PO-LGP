/*

  We need:

  -- a clean interface for user's (method developer's) for coding methods/algorithms

  -- this should be independent of the ``middleware''/biros/ROS/whatever

 */

#include <MT/array.h>
#include <MT/registry.h>

struct Module;

//===========================================================================
//
// entries in a global pre-main instantiated registry
//

enum UnitKind { none=0, module, varaccess };

template<class T, class P>
struct UnitRegistry:Item{
  UnitKind kind;
  UnitRegistry(const StringA& _keys, const ItemL& _parents, UnitKind _kind, MapGraph *container=NULL):kind(_kind){
    keys=_keys;
    parents=_parents;
    if(container) container->append(this);
  }
  virtual void writeValue(std::ostream &os) const { os <<"REGISTRY " <<typeid(T).name() <<' ' <<typeid(P).name(); }
  virtual const std::type_info& valueType() const { return typeid(UnitRegistry<T,P>); }
  virtual bool is_derived_from_TypeBase() const { return is_base_of<TypeBase, T>::value; }
  virtual void *newInstance() const { return new T; }
};


//===========================================================================
//
// objects to handle r/w access to variables -- may implement locking and versioning later
//

struct VariableAccess{
  const char* name;
  const std::type_info* typeinfo;
  VariableAccess(const char* _name, const std::type_info* _typeinfo):name(_name), typeinfo(_typeinfo) {}
  void write(ostream& os) const{ os <<name <<' ' <<typeinfo->name(); }
  virtual void createOwnVariable() = 0;
};
stdOutPipe(VariableAccess);

typedef MT::Array<VariableAccess*> VariableAccessL;

template<class T>
struct VariableAccess_typed:VariableAccess{
  T *var;
  VariableAccess_typed(const char* name):VariableAccess(name,&typeid(T)), var(NULL){ }
  T& get(Module *m){ CHECK(var,"access to variable "<<typeid(T).name()<<" uninitialized"); return *var; }
  int set(const T& x, Module *m){ CHECK(var,"access to variable "<<typeid(T).name()<<" uninitialized"); *var=x; return 0; }
  virtual void createOwnVariable(){ CHECK(!var,""); var = new T; }
};


//===========================================================================
//
// Module base class
//

struct Module{
  static VariableAccessL accesses;
  Module(){}
  virtual ~Module(){};
  virtual void step() = 0;
};

VariableAccessL Module::accesses;


//===========================================================================
//
// A static registator class; different for any module or field type; triggers global registration
//

/* This class is a registration tool: For any type T (and parent type P, default void)
   it instantiates a static object. The constructor of this object can contain arbitrary
   code which is called during __static_init... before main. Classes can derive from
   Registrator<classType, void> and somewhere (doesn't matter where) call force(); members
   can register by somewhere calling forceSub<memberType>();

   Here, we append a type registration to the global registry() */
template<class T, class P> struct Registrator{
  struct StaticRegistrator{
    StaticRegistrator(){
      //** this is the code executed during registration (__static_init...)
      ItemL parents;
      Item *parent = NULL;
      UnitKind kind=module;
      if(typeid(P)!=typeid(void)){ //dependence registry
        parent = registry().getItem("module", typeid(P).name());
        CHECK(parent,"");
        parents.append(parent);
        kind=varaccess;
      }
      Item *it = new UnitRegistry<T,P>(
            STRINGS((kind==module?"module":"var"), typeid(T).name()),
            parents,
            kind,
            &registry());
      if(parent) parent->parentOf.append(it);
    }
    void* force(){ return &staticRegistrator; }
    template<class S> void* forceSub(){ return &Registrator<S,T>::staticRegistrator; }
  };
  static StaticRegistrator staticRegistrator;
  void write(std::ostream& os) const{};
  void read(std::istream& os){};
};
template<class T,class P> typename Registrator<T,P>::StaticRegistrator Registrator<T,P>::staticRegistrator;
template<class T,class P> stdInPipe(Registrator<T KO P>);
template<class T,class P> stdOutPipe(Registrator<T KO P>);


//===========================================================================
//
// macros to declare modules or fields
//

// the force...() forces the staticRegistrator to be created in pre-main
// and its constructor executed, which does the registration

#define VAR(type, name) \
  struct name##_Access:VariableAccess_typed<type>{ \
    name##_Access():VariableAccess_typed<type>(#name){ accesses.append(this); } \
  } name##_access; \
  inline void name##_forceRegistration(){ staticRegistrator.forceSub<type>(); } \
  inline type& get_##name(){ return name##_access.get(this); } \
  inline int  set_##name(const type& _x){ return name##_access.set(_x,this); }


//#define PARAM (type, name, default) \
//  ParameterAccess_typed<type, default> name##_access; \
//  inline type get_##name(){ return name##_access.get(); }

#define MODULE_BASE(name)  Registrator<name, void>, Module
