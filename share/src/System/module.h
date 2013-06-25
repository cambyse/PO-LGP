/*

  We need:

  -- a clean interface for user's (method developer's) for coding methods/algorithms

  -- this should be independent of the ``middleware''/biros/ROS/whatever

 */
#ifndef MT_module_h
#define MT_module_h

#include <Core/array.h>
#include <Core/registry.h>
#include <stddef.h>

#ifndef FIELD
#define FIELD(type, name) \
  type name; \
  inline int set_##name(const type& _x, Module *p){ \
    writeAccess(p);  name=(type&)_x;  return deAccess(p); } \
  inline int get_##name(type& _x, Module *p){ \
    readAccess(p);   _x=name;  return deAccess(p); } \
  inline type get_##name(Module *p){ \
    type _x; readAccess(p); _x=name; deAccess(p);  return _x;  }

//  inline void reg_##name(){
//    registerField(this, new FieldRegistration_typed<type>(&name,this,#name,#type)); }
#endif

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
  T& operator()(){ CHECK(variable->rwlock.state==-1,"");  return *data; } //TODO ensure that it is locked
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
  Type *ti = new Type_typed<T,P>(NULL, NULL);
  return new Item_typed<Type>(keys, parents, ti, &registry());
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
template<class T, class N, class P> struct Registrator{
  struct StaticRegistrator{
    Item *reg;
    StaticRegistrator():reg(NULL){ //called whenever a module/access is DECLARED
      Item *parent = NULL;
      const char *declkey="Decl_Module";
      MT::String name;
      if(typeid(P)!=typeid(void)){ //dependence registry
        parent = registry().getItem("Decl_Module", typeid(P).name());
        declkey="Decl_Access";
      }
      if(typeid(N)!=typeid(void)){ //extract a name from the type
        const char *nam=typeid(N).name();
        uint i;
        for(i=0;;i++) if(nam[i]=='_' && nam[i+1]=='_') break;
        nam = &nam[i+2];
        for(i=strlen(nam);;i--) if(nam[i]=='_' && nam[i+1]=='_') break;
        name.set(nam,i);
        MT_MSG("StaticRegistrator of type " <<typeid(N).name() <<" named " <<name);
      }else{
        name = typeid(T).name();
      }
      reg = registerItem<T,P>(NULL, declkey, name, parent, NULL);
      if(parent) parent->parentOf.append(reg);
    }
    void* force(){ return &staticRegistrator; }
  };
  static StaticRegistrator staticRegistrator;
};
template<class T,class N,class P> typename Registrator<T,N,P>::StaticRegistrator Registrator<T,N,P>::staticRegistrator;


//===========================================================================
//
// macros to declare modules or fields
//

// the force...() forces the staticRegistrator to be created in pre-main
// and its constructor executed, which does the registration
#define OFFSETOF(T,F) ((unsigned int)((char *)&((T*)1L)->F - (char*)1L))

#define BEGIN_MODULE(name) \
  struct name; \
  struct name##_Base: Module, Registrator<name, void, void> { \
    typedef name __MODULE_TYPE__; \
    typedef name##_Base __MODULE_BASE_TYPE__; \
    inline void name##_forceModuleReg(){ staticRegistrator.force(); } \
    name##_Base(): Module(#name) { \
      reg = registerItem<name, void>((name*)this, "Module", #name, NULL, staticRegistrator.reg); \
    } \

#define ACCESS(type, name) \
  struct __##name##__Access:Access_typed<type>, Registrator<type, __##name##__Access, __MODULE_TYPE__>{ \
    __##name##__Access():Access_typed<type>(#name){ \
      uint offset = OFFSETOF(__MODULE_BASE_TYPE__, name); \
      __MODULE_BASE_TYPE__ *thisModule = (__MODULE_BASE_TYPE__*)((char*)this - (char*)offset); \
      CHECK(&(thisModule->name)==this,"offset didn't work!"); \
      thisModule->accesses.append(this); \
      reg = registerItem<__##name##__Access, __MODULE_TYPE__>(this, "Access", #name, thisModule->reg, staticRegistrator.reg); \
      module = thisModule; \
    } \
  } name; \
  inline const type& get_##name(){ return name.get(); } \
  inline void  set_##name(const type& _x){ name.set()=_x; }

#define END_MODULE() };

#define MODULE(name) \
  struct name: Module, Registrator<name, void> { \
    typedef name __MODULE_TYPE__; \
    typedef name __MODULE_BASE_TYPE__; \
    inline void name##_forceModuleReg(){ staticRegistrator.force(); } \
    name(): Module(#name) { \
      reg = registerItem<name, void>(this, "Module", #name, NULL, staticRegistrator.reg); \
    } \
    virtual ~name(){}; \
    virtual void step(); \
    virtual bool test();

#endif

//uint offset = offsetof(__MODULE_BASE_TYPE__, name);

