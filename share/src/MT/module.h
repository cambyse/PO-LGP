/*

  We need:

  -- a clean interface for user's (method developer's) for coding methods/algorithms

  -- this should be independent of the ``middleware''/biros/ROS/whatever

 */
#ifndef MT_module_h
#define MT_module_h

#include <MT/array.h>
#include <MT/registry.h>
#include <stddef.h>

struct Module;

//===========================================================================
//
// entries in a global pre-main instantiated registry
//

enum UnitKind { none=0, module, varaccess };

template<class T, class P>
Item* registerUnit(T *instance, const char *name, UnitKind kind, Item *parent1=NULL, Item *parent2=NULL){
  ItemL parents;  if(parent1) parents.append(parent1);  if(parent2) parents.append(parent2);
  StringA keys;
  if(!name) name=typeid(T).name();
  if(!instance && kind==module) keys=STRINGS("moduledecl", name);
  if(!instance && kind==varaccess) keys=STRINGS("accdecl", name);
  if(instance && kind==module) keys=STRINGS("module", name);
  if(instance && kind==varaccess) keys=STRINGS("acc", name);
  KeyValueGraph *ats = new KeyValueGraph();
  ats->append<MT::String>("name", new MT::String(name));
  ats->append<TypeInfo>("type", new TypeInfo_typed<T,P>(name, name, NULL, NULL));
  return  new Item_typed<KeyValueGraph>(keys, parents, ats, &registry());
//  return new UnitRegistry<T, void>(keys, parents, kind, instance, &registry());
}


//===========================================================================
//
// objects to handle r/w access to variables -- may implement locking and versioning later
//

struct AccessGuard{
  MT::String name;            ///< Variable name
  ConditionVariable revision; ///< revision (= number of write accesses) number
  AccessGuard(const char* _name):name(_name), revision(0){}
  virtual ~AccessGuard(){}
  virtual int readAccess(struct Process*){ cout <<"R " <<name <<endl; return 0; }
  virtual int writeAccess(struct Process*){ cout <<"W " <<name <<endl; return 0; }
  virtual int deAccess(struct Process*){ return 0; }
};

struct Access{
  Module *module;
  struct Process *process;
  AccessGuard *guard;
  const char* name;
  const std::type_info* typeinfo;
  Access(const char* _name, const std::type_info* _typeinfo):module(NULL), guard(NULL), name(_name), typeinfo(_typeinfo) {}
  void write(ostream& os) const{ os <<name; }
  void read(istream&) { NIY; }
  virtual void* createOwnData() = 0;
  virtual void setData(void*) = 0;
};
stdPipes(Access);

typedef MT::Array<Access*> AccessL;

template<class T>
struct Access_typed:Access{
  T *var;
  Access_typed(const char* name=NULL):Access(name,&typeid(T)), var(NULL){ //called at INSTANTIATION of a module
//    Item *it = new UnitRegistry<T,void>(
//          STRINGS("acc", name),
//          ARRAY<Item*>(), //staticRegistrator.regItem),
//          varaccess,
//          &registry());
  }
  T& get(Module *m){
    CHECK(var,"access to variable "<<typeid(T).name()<<" uninitialized");
    guard->readAccess(process);
    guard->deAccess(process);
    return *var;
  }
  int set(const T& x, Module *m){
    CHECK(var,"access to variable "<<typeid(T).name()<<" uninitialized");
    guard->writeAccess(process);
    *var=x;
    guard->deAccess(process);
    return 0;
  }
  virtual void* createOwnData(){ if(!guard) guard=new AccessGuard(name); if(!var) var=new T; return var; }
  virtual void setData(void* data){ var = (T*)data; }
};


//===========================================================================
//
// Module base class
//

struct Module{
  const char *name;
  AccessL accesses;
  Module(const char *_name):name(_name){}
  virtual ~Module(){};
  virtual void step() = 0;
  virtual bool test() = 0;
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
        //cout <<registry() <<endl;
        parent = registry().getItem("moduledecl", typeid(P).name());
        //CHECK(parent,"");
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

#define VARI(type, name) \
  struct name##_Access:Access_typed<type>, Registrator<Access_typed<type>, __MODULE_TYPE__>{ \
  name##_Access():Access_typed<type>(#name),  Registrator<Access_typed<type>, __MODULE_TYPE__>(this){ \
      uint offset = offsetof(__MODULE_TYPE__, name); \
      __MODULE_TYPE__ *thisModule = (__MODULE_TYPE__*)((char*)this - (char*)offset); \
      thisModule->accesses.append(this); \
      Registrator<Access_typed<type>, __MODULE_TYPE__>::regis(#name, thisModule->regItem); \
      module = thisModule; \
    } \
  } name; \
  inline type& get_##name(){ CHECK(name.module==this,"OUCH!"); return name.get(this); } \
  inline int  set_##name(const type& _x){ CHECK(name.module==this,"OUCH!"); return name.set(_x,this); }


#define DECLARE_MODULE(name) \
  struct name; \
  struct name##_Base: Module, Registrator<name, void> { \
    typedef name __MODULE_TYPE__; \
    inline void name##_forceModuleReg(){ staticRegistrator.force(); } \
    name##_Base(): Module(#name), Registrator<name, void>((name*)this) { \
      Registrator<name, void>::regis(#name, NULL); \
    } \
  };

#define MODULE(name) \
  struct name: Module, Registrator<name, void> { \
    typedef name __MODULE_TYPE__; \
    inline void name##_forceModuleReg(){ staticRegistrator.force(); } \
    name(): Module(#name), Registrator<name, void>((name*)this) { \
      Registrator<name, void>::regis(#name, NULL); \
    } \
    virtual ~name(){}; \
    virtual void step(); \
    virtual bool test();

#endif
