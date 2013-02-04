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

struct TypeRegistration:TypeBase{
  StringA keys;
  MT::Array<TypeRegistration*> parents;
  virtual const std::type_info& typeinfo() const = 0;
  virtual struct Item* read(istream&) const = 0;
  virtual TypeBase* newInstance() const = 0;
  void write(std::ostream& os) const{
    os <<"Type '" <<keys <<"' [" <<typeinfo().name() <<"] ";
    if(parents.N){
      cout <<"parents=[";
      for_list_(TypeRegistration, p, parents) cout <<' ' <<p->typeinfo().name();
      cout <<" ]";
    }
  }
};
stdOutPipe(TypeRegistration);

typedef MT::Array<TypeRegistration*> TypeRegistrationL;

// user interface

//-- query existing types
TypeRegistration* reg_findType(const char* type);
template <class T> TypeRegistrationL reg_findDerived();
inline Item* readTypeIntoItem(const char* type, std::istream& is){
  TypeRegistration *it = reg_findType(type);
  if(it) return it->read(is);
  return NULL;
}

//-- use these macros to register types in cpp files

#define KO ,
#define REGISTER_TYPE(Type) \
  REGISTER_ITEM2(TypeRegistration_typed<Type KO void>, type, Type, TypeRegistration_typed<Type KO void>(#Type,NULL,NULL,NULL));
//  REGISTER_ITEM2(Type*, type, Type, NULL)

#define REGISTER_TYPE_Key(Key, Type) \
  REGISTER_ITEM2(TypeRegistration_typed<Type KO void>, type, Key, TypeRegistration_typed<Type KO void>(#Type,NULL,NULL,NULL));
//  REGISTER_ITEM2(Type*, type, Key, NULL);

#define REGISTER_TYPE_DERIVED(Type, Base) \
  REGISTER_ITEM2(TypeRegistration_typed<Type KO Base>, type, Type, TypeRegistration_typed<Type KO Base>(#Type,NULL,#Base,NULL));

#include "registry_t.cxx"


//===========================================================================
//
// registrator -- helper to call registration code for classes & members
//

/* This class is a registration tool: For any type T (and parent type P, default void)
   it instantiates a static object. The constructor of this object can contain arbitrary
   code which is called during __static_init... before main. Classes should derive from Registrator<classType, void> and somewhere
   (doesn't matter where) call force(); members can register by somewhere calling forceSub<memberType>();

   Here, we append a type registration to the global registry() */
template<class T, class P> struct Registrator{
  struct StaticRegistrator{
    StaticRegistrator(){
      //** this is the code executed during registration (__static_init...)
      ItemL parents;
      Item *parent = NULL;
      if(typeid(P)!=typeid(void)){ //dependence registry
        parent = registry().getTypedItem<TypeRegistration_typed<P, void> >("unit");
        CHECK(parent,"");
        parents.append(parent);
      }
      Item *it = new Item_typed<TypeRegistration_typed<T,P> >(
            STRINGS("unit",typeid(T).name()),
            parents,
            TypeRegistration_typed<T,P>("noUserName",NULL,NULL,NULL),
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

// stuff that needs to be included by the header to enable macros and templates

#endif
