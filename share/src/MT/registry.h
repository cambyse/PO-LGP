#ifndef MT_registry_h
#define MT_registry_h

#include "util.h"
#include "array.h"
#include "mapGraph.h"


//===========================================================================
//
// global registry of anything
//

MapGraph& registry();

#define REGISTER_ITEM(Type, key, value) \
  Item_typed<Type > key##_RegistryEntry(ARRAY<MT::String>(MT::String(#key)), ItemL(), value, &registry());

#define REGISTER_ITEM2(Type, key1, key2, value) \
  Item_typed<Type > key1##_##key2##_RegistryEntry(ARRAY<MT::String>(MT::String(#key1),MT::String(#key2)), ItemL(), value, &registry());


//===========================================================================
//
// a generic singleton registry (list) of things (usually Base_typed deriving from a common Base)
//

template<class T>
struct Singleton{
  struct SingletonFields{ //class cannot have own members: everything in the singleton which is created on first demand
    T obj;
    RWLock lock;
  };

  static SingletonFields *singleton;

  SingletonFields& getSingleton() const{
    static bool currentlyCreating=false;
    if(currentlyCreating) return *((SingletonFields*) NULL);
    if(!singleton) {
      static Mutex m;
      m.lock();
      if(!singleton) {
        currentlyCreating=true;
        singleton = new SingletonFields();
        currentlyCreating=false;
      }
      m.unlock();
    }
    return *singleton;
  }

  T& obj(){ return getSingleton().obj; }
};


//===========================================================================
//
// define a type registry
//

struct TypeInfo:TypeBase{
  StringA keys;
  MT::Array<TypeInfo*> parents;
  virtual const std::type_info& type_info() const {NIY};
  virtual struct Item* readItem(istream&) const {NIY};
  virtual void* newInstance() const {NIY}
  void write(std::ostream& os) const{
    os <<"Type '" <<keys <<"' [" <<type_info().name() <<"] ";
    if(parents.N){
      cout <<"parents=[";
      for_list_(TypeInfo, p, parents) cout <<' ' <<p->type_info().name();
      cout <<" ]";
    }
  }
  void read(std::istream& is) const{ NIY; }
};
stdPipes(TypeInfo);

typedef MT::Array<TypeInfo*> TypeInfoL;

// user interface

//-- query existing types
template <class T> TypeInfoL reg_findDerived();
inline TypeInfo *reg_findType(const char* key){
  TypeInfoL types = registry().getDerivedValues<TypeInfo>();
  for_list_(TypeInfo, ti, types){
    for(uint i=0;i<ti->keys.N;i++) if(ti->keys(i)==key) return ti;
  }
  return NULL;
}
template<class T>
TypeInfo *reg_findType(){
  TypeInfoL types = registry().getDerivedValues<TypeInfo>();
  for_list_(TypeInfo, ti, types){
    if(ti->type_info()==typeid(T)) return ti;
  }
  return NULL;
}

inline Item* readTypeIntoItem(const char* key, std::istream& is){
  TypeInfoL types = registry().getDerivedValues<TypeInfo>();
  TypeInfo *ti = reg_findType(key);
  if(ti) return ti->readItem(is);
  return NULL;
}

//-- use these macros to register types in cpp files

#define KO ,
#define REGISTER_TYPE(Type) \
  REGISTER_ITEM2(TypeInfo_typed<Type KO void>, type, Type, new TypeInfo_typed<Type KO void>(#Type,NULL,NULL,NULL));
//  REGISTER_ITEM2(Type*, type, Type, NULL)

#define REGISTER_TYPE_Key(Key, Type) \
  REGISTER_ITEM2(TypeInfo_typed<Type KO void>, type, Key, new TypeInfo_typed<Type KO void>(#Type,#Key,NULL,NULL));
//  REGISTER_ITEM2(Type*, type, Key, NULL);

#define REGISTER_TYPE_DERIVED(Type, Base) \
  REGISTER_ITEM2(TypeInfo_typed<Type KO Base>, type, Type, new TypeInfo_typed<Type KO Base>(#Type,NULL,#Base,NULL));

#include "registry_t.cxx"


//===========================================================================
//
// registrator -- helper to call registration code for classes & members
//


// stuff that needs to be included by the header to enable macros and templates

#endif
