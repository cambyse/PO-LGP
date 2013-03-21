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
struct Variable;
typedef MT::Array<Access*> AccessL;
typedef MT::Array<Module*> ModuleL;
typedef MT::Array<Variable*> VariableL;


//===========================================================================
//
// Module base class
//

struct Module{
  const char *name;
  struct Process *proc;
  AccessL accesses;
  Item *reg;
  uint step_count;         ///< step count
  Module(const char *_name):name(_name), proc(NULL), reg(NULL), step_count(0){}
  virtual ~Module(){};
  virtual void step() = 0;
  virtual bool test(){ return true; }
  void write(std::ostream& os) const{};
  void read(std::istream& os){};
};
stdPipes(Module);


//===========================================================================
/**
 * A Variable is a container to hold data, potentially handling concurrent r/w
 * access, which is used to exchange information between processes.
 */
struct Variable {
  struct sVariable *s;        ///< private
  void *data;                 ///< Variable data
  MT::String name;            ///< Variable name
  ConditionVariable revision; ///< revision (= number of write accesses) number
  RWLock rwlock;              ///< rwLock (usually handled via read/writeAccess -- but views may access directly...)
  Item *reg;
  //AccessL accesses;

  /// @name c'tor/d'tor
  Variable(const char* name);
  virtual ~Variable();

  /// @name access control
  /// to be called by a processes before access, returns the revision
  int readAccess(Module*);  //might set the caller to sleep
  int writeAccess(Module*); //might set the caller to sleep
  int deAccess(Module*);

  /// @name syncing via a variable
  /// the caller is set to sleep
  void waitForNextWriteAccess();
  int  waitForRevisionGreaterThan(int rev); //returns the revision

  /// @name info
  struct FieldRegistration& get_field(uint i) const;
};

//TODO: hide?
struct sVariable {
  MT::Array<struct FieldRegistration*> fields; //? make static? not recreating for each variable?
  ModuleL listeners;
  struct LoggerVariableData *loggerData; //data that the logger may associate with a variable

  virtual void serializeToString(MT::String &string) const;
  virtual void deSerializeFromString(const MT::String &string);

  sVariable():loggerData(NULL){}
};


//===========================================================================
//
// modules communicate to variables via an Access
//

struct Access{
  Module *module;
  Variable *variable;
  const char* name;
  Item *reg;
  Access(const char* _name):module(NULL), variable(NULL), name(_name), reg(NULL) {}
  void write(ostream& os) const{ os <<name; }
  void read(istream&) { NIY; }
  virtual void* createOwnData() = 0;
  virtual void setData(void*) = 0;
  void readAccess(){ variable->readAccess(module); }
  void writeAccess(){ variable->writeAccess(module); }
  void deAccess(){ variable->deAccess(module); }
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
  T& operator()(){ return *data; } //TODO ensure that it is locked
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
  if(!variable){
    variable=new Variable(name);
    variable->reg = registerItem<T, Access_typed<T> >(data,
                                                   "Variable", name,
                                                   reg, NULL);
  }
  if(!data) variable->data = data = new T;
  return data;
}

template<class T> void Access_typed<T>::setData(void* _data){
  if(!variable){
    variable=new Variable(name);
    variable->reg = registerItem<T, Access_typed<T> >(data,
                                                   "Variable", name,
                                                   reg, NULL);
  }
  variable->data = data = (T*)_data;
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
      const char *declkey="Decl_Module";
      if(typeid(P)!=typeid(void)){ //dependence registry
        parent = registry().getItem("Decl_Module", typeid(P).name());
        declkey="Decl_Access";
      }
      reg = registerItem<T,P>(NULL, declkey, typeid(T).name(), parent, NULL);
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
  struct name##_Access:Access_typed<type>, Registrator<name##_Access, __MODULE_TYPE__>{ \
  name##_Access():Access_typed<type>(#name){ \
      uint offset = offsetof(__MODULE_TYPE__, name); \
      __MODULE_TYPE__ *thisModule = (__MODULE_TYPE__*)((char*)this - (char*)offset); \
      thisModule->accesses.append(this); \
      reg = registerItem<name##_Access, __MODULE_TYPE__>(this, "Access", #name, thisModule->reg, staticRegistrator.reg); \
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
