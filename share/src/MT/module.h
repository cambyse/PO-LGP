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


struct Access;
struct Module;
typedef MT::Array<Access*> AccessL;


//===========================================================================
//
// Module base class
//

struct Module{
  const char *name;
  AccessL accesses;
  Item *reg;
  Module(const char *_name):name(_name), reg(NULL){}
  virtual ~Module(){};
  virtual void step() = 0;
  virtual bool test(){ return true; }
  void write(std::ostream& os) const{};
  void read(std::istream& os){};
};
stdPipes(Module);


//===========================================================================
//
// a variable (container) to hold data, potentially handling r/w access
//

struct AccessGuard{
  MT::String name;            ///< Variable name
  ConditionVariable revision; ///< revision (= number of write accesses) number
  //AccessL accesses;
  void *data;
  Item *reg;
  AccessGuard(const char* _name):name(_name), revision(0), data(NULL), reg(NULL){}
  virtual ~AccessGuard(){}
  virtual int readAccess(struct Process*){ cout <<"R " <<name <<endl; return 0; }
  virtual int writeAccess(struct Process*){ cout <<"W " <<name <<endl; return 0; }
  virtual int deAccess(struct Process*){ return 0; }
};


//===========================================================================
//
// modules communicate to variables via an Access
//

struct Access{
  Module *module;
  AccessGuard *guard;
  struct Process *process; //TODO: this is very very ugly!
  const char* name;
  Item *reg;
  Access(const char* _name):module(NULL), guard(NULL), name(_name), reg(NULL) {}
  void write(ostream& os) const{ os <<name; }
  void read(istream&) { NIY; }
  virtual void* createOwnData() = 0;
  virtual void setData(void*) = 0;
  void readAccess(){ guard->readAccess(process); }
  void writeAccess(){ guard->writeAccess(process); }
  void deAccess(){ guard->deAccess(process); }
};
stdPipes(Access);


template<class T>
struct Access_typed:Access{
  struct ReadToken{
    Access_typed<T> *a;
    ReadToken(Access_typed<T> *_a):a(_a){ a->readAccess(); }
    ~ReadToken(){ a->deAccess(); }
    const T& operator()(){ return *a->data; }
  };
  struct WriteToken{
    Access_typed<T> *a;
    WriteToken(Access_typed<T> *_a):a(_a){ a->writeAccess(); }
    ~WriteToken(){ a->deAccess(); }
    T& operator()(){ return *a->data; }
  };

  T *data;

  Access_typed(const char* name=NULL):Access(name), data(NULL){}
  const T& get(){ return ReadToken(this)(); }
  T& set(){ return WriteToken(this)(); }
  virtual void* createOwnData();
  virtual void setData(void* _data);
};


//===========================================================================
//
// some helpers
//

template<class T, class P>
Item* registerItem(T *instance, const char *key1, const char* key2, Item *parent1=NULL, Item *parent2=NULL){
  ItemL parents;  if(parent1) parents.append(parent1);  if(parent2) parents.append(parent2);
  StringA keys; if(key1) keys.append(MT::String(key1)); if(key2) keys.append(MT::String(key2));
  TypeInfo *ti = new TypeInfo_typed<T,P>(NULL, NULL);
  return  new Item_typed<TypeInfo>(keys, parents, ti, &registry());
}


template<class T> void* Access_typed<T>::createOwnData(){
  if(!guard){
    guard=new AccessGuard(name);
    guard->reg = registerItem<T, Access_typed<T> >(data,
                                                   "Variable", name,
                                                   reg, NULL);
  }
  if(!data) guard->data = data = new T;
  return data;
}

template<class T> void Access_typed<T>::setData(void* _data){
  if(!guard){
    guard=new AccessGuard(name);
    guard->reg = registerItem<T, Access_typed<T> >(data,
                                                   "Variable", name,
                                                   reg, NULL);
  }
  guard->data = data = (T*)_data;
}

//===========================================================================
//
// A static registator struct
//

/* This class is a registration tool: For any type T (and parent type P, default void)
   it instantiates a static object. The constructor of this object can contain arbitrary
   code which is called during __static_init... before main. Classes can derive from
   Registrator<classType, void> and somewhere (doesn't matter where) call force();
   In our case we call registerItem */
template<class T, class P> struct Registrator{
  struct StaticRegistrator{
    Item *reg;
    StaticRegistrator():reg(NULL){ //called whenever a module/access is DECLARED
      Item *parent = NULL;
      const char *decltype="Decl_Module";
      if(typeid(P)!=typeid(void)){ //dependence registry
        parent = registry().getItem("Decl_Module", typeid(P).name());
        decltype="Decl_Access";
      }
      reg = registerItem<T,P>(NULL, decltype, typeid(T).name(), parent, NULL);
      if(parent) parent->parentOf.append(reg);
    }
    void* force(){ return &staticRegistrator; }
  };
  static StaticRegistrator staticRegistrator;
};
template<class T,class P> typename Registrator<T,P>::StaticRegistrator Registrator<T,P>::staticRegistrator;


//===========================================================================
//
// macros to declare modules or fields
//

// the force...() forces the staticRegistrator to be created in pre-main
// and its constructor executed, which does the registration

#define ACCESS(type, name) \
  struct name##_Access:Access_typed<type>, Registrator<Access_typed<type>, __MODULE_TYPE__>{ \
  name##_Access():Access_typed<type>(#name){ \
      uint offset = offsetof(__MODULE_TYPE__, name); \
      __MODULE_TYPE__ *thisModule = (__MODULE_TYPE__*)((char*)this - (char*)offset); \
      thisModule->accesses.append(this); \
      reg = registerItem<Access_typed<type>, __MODULE_TYPE__>(this, "Access", #name, thisModule->reg, staticRegistrator.reg); \
      module = thisModule; \
    } \
  } name; \
  inline const type& get_##name(){ return name.get(); } \
  inline void  set_##name(const type& _x){ name.set()=_x; }

#define DECLARE_MODULE(name) \
  struct name; \
  struct name##_Base: Module, Registrator<name, void> { \
    typedef name __MODULE_TYPE__; \
    inline void name##_forceModuleReg(){ staticRegistrator.force(); } \
    name##_Base(): Module(#name) { \
    reg = registerItem<name, void>((name*)this, "Module", #name, NULL, staticRegistrator.reg); \
    } \
  };

#define MODULE(name) \
  struct name: Module, Registrator<name, void> { \
    typedef name __MODULE_TYPE__; \
    inline void name##_forceModuleReg(){ staticRegistrator.force(); } \
    name(): Module(#name) { \
      reg = registerItem<name, void>(this, "Module", #name, NULL, staticRegistrator.reg); \
    } \
    virtual ~name(){}; \
    virtual void step(); \
    virtual bool test();

#endif
