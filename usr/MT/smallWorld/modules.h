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
  T *instantiatedObject;
  UnitRegistry(const StringA& _keys, const ItemL& _parents, UnitKind _kind, T *_obj, MapGraph *container):
    kind(_kind),
    instantiatedObject(_obj){
    keys=_keys;
    parents=_parents;
    if(container) container->append(this);
  }
  virtual void writeValue(std::ostream &os) const { os <<"REGISTRY " <<typeid(T).name() <<' ' <<typeid(P).name(); }
  virtual const std::type_info& valueType() const { return typeid(UnitRegistry<T,P>); }
  virtual bool is_derived_from_TypeBase() const { return is_base_of<TypeBase, T>::value; }
  virtual void *newInstance() const { return new T; }
};

template<class T, class P>
Item* registerUnit(T *instance, const char *name, UnitKind kind, Item *parent1=NULL, Item *parent2=NULL){
  ItemL parents;  if(parent1) parents.append(parent1);  if(parent2) parents.append(parent2);
  if(!name) name=typeid(T).name();
  if(!instance){
    if(kind==module)
      return new UnitRegistry<T, void>(
            STRINGS("moduledecl", name), parents, kind, instance, &registry());
    if(kind==varaccess)
      return new UnitRegistry<T, void>(
            STRINGS("accdecl", name), parents, kind, instance, &registry());
  }else{
    if(kind==module)
      return new UnitRegistry<T, void>(
            STRINGS("module", name), parents, kind, instance, &registry());
    if(kind==varaccess)
      return new UnitRegistry<T, void>(
            STRINGS("acc", name), parents, kind, instance, &registry());
  }
}


//===========================================================================
//
// objects to handle r/w access to variables -- may implement locking and versioning later
//

struct VariableAccess{
  Module *module;
  const char* name;
  const std::type_info* typeinfo;
  VariableAccess(const char* _name, const std::type_info* _typeinfo):module(NULL), name(_name), typeinfo(_typeinfo) {}
  void write(ostream& os) const{ os <<name; }
  virtual void createOwnVariable() = 0;
};
stdOutPipe(VariableAccess);

typedef MT::Array<VariableAccess*> VariableAccessL;

template<class T>
struct VariableAccess_typed:VariableAccess{
  T *var;
  VariableAccess_typed(const char* name=NULL):VariableAccess(name,&typeid(T)), var(NULL){ //called at INSTANTIATION of a module
//    Item *it = new UnitRegistry<T,void>(
//          STRINGS("acc", name),
//          ARRAY<Item*>(), //staticRegistrator.regItem),
//          varaccess,
//          &registry());
  }
  T& get(Module *m){ CHECK(var,"access to variable "<<typeid(T).name()<<" uninitialized"); return *var; }
  int set(const T& x, Module *m){ CHECK(var,"access to variable "<<typeid(T).name()<<" uninitialized"); *var=x; return 0; }
  virtual void createOwnVariable(){ CHECK(!var,""); var = new T; }
};


//===========================================================================
//
// Module base class
//

struct Module{
  const char *name;
  VariableAccessL accesses;
  Module(const char *_name):name(_name){}
  virtual ~Module(){};
  virtual void step() = 0;
};


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
    Item *regItem;
    UnitKind kind;
    StaticRegistrator(){ //called whenever a module is DECLARED
      //** this is the code executed during registration (__static_init...)
      regItem = NULL;
      kind = module;
      Item *parent = NULL;
      if(typeid(P)!=typeid(void)){ //dependence registry
        parent = registry().getItem("moduledecl", typeid(P).name());
        CHECK(parent,"");
        kind=varaccess;
      }
      regItem = registerUnit<T,P>(NULL, NULL, kind, parent);
      if(parent) parent->parentOf.append(regItem);
    }
    void* force(){ return &staticRegistrator; }
  };
  static StaticRegistrator staticRegistrator;
  Item *regItem;
  T *instantiatedObject;
  Registrator(T *obj):regItem(NULL), instantiatedObject(obj){ //called whenever a module is INSTANTIATED
  }
  void regis(const char *name, Item *parentItem){
    regItem = registerUnit<T,P>(instantiatedObject,
                                name,
                                staticRegistrator.kind,
                                parentItem,
                                staticRegistrator.regItem);
  }
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
  struct name##_Access:VariableAccess_typed<type>, Registrator<VariableAccess_typed<type>, __MODULE_TYPE__>{ \
  name##_Access():VariableAccess_typed<type>(#name),  Registrator<VariableAccess_typed<type>, __MODULE_TYPE__>(this){ \
      uint offset = (uint)&((__MODULE_TYPE__*)NULL)->name##_access; \
      __MODULE_TYPE__ *thisModule = (__MODULE_TYPE__*)((char*)this - (char*)offset); \
      thisModule->accesses.append(this); \
      Registrator<VariableAccess_typed<type>, __MODULE_TYPE__>::regis(#name, thisModule->regItem); \
      module = thisModule; \
    } \
  } name##_access; \
  inline type& get_##name(){ CHECK(name##_access.module==this,"OUCH!"); return name##_access.get(this); } \
  inline int  set_##name(const type& _x){ CHECK(name##_access.module==this,"OUCH!"); return name##_access.set(_x,this); }

//inline void name##_Access_forceRegistration(){ staticRegistrator.force(); } \
//inline void name##_forceRegistration(){ staticRegistrator.forceSub<type>(); } \

//#define PARAM (type, name, default) \
//  ParameterAccess_typed<type, default> name##_access; \
//  inline type get_##name(){ return name##_access.get(); }


#define DECLARE_MODULE(name) \
  struct name; \
  struct name##_Base: Module, Registrator<name, void> { \
    typedef name __MODULE_TYPE__; \
    inline void name##_forceModuleReg(){ staticRegistrator.force(); } \
    name##_Base(): Module(#name), Registrator<name, void>((name*)this) { \
      Registrator<name, void>::regis(#name, NULL); \
    } \
  };

