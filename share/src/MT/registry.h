#ifndef MT_registry_h
#define MT_registry_h

#include "util.h"
#include "array.h"
#include "keyValueGraph.h"


//===========================================================================
//
// global registry of anything
//

KeyValueGraph& registry();

#define REGISTER_ITEM(Type, key, value) \
  Item_typed<Type > key##_RegistryEntry(ARRAY<MT::String>(MT::String(#key)), ItemL(), value, &registry());

#define REGISTER_ITEM2(Type, key1, key2, value) \
  Item_typed<Type > key1##_##key2##_RegistryEntry(ARRAY<MT::String>(MT::String(#key1),MT::String(#key2)), ItemL(), value, &registry());


//===========================================================================
//
// define a type registry
//

struct Type:TypeBase{
  MT::Array<Type*> parents; //TODO -> remove; replace functionality from registry
  virtual const std::type_info& typeId() const {NIY}; //TODO -> typeid()
  virtual struct Item* readItem(istream&) const {NIY}; //TODO -> readIntoNewItem
  virtual void* newInstance() const {NIY}
  void write(std::ostream& os) const{
    os <<"Type '" <<typeId().name() <<"' ";
    if(parents.N){
      cout <<"parents=[";
      for_list_(Type, p, parents) cout <<' ' <<p->typeId().name();
      cout <<" ]";
    }
  }
  void read(std::istream& is) const{ NIY; }
};
stdPipes(Type);

typedef MT::Array<Type*> TypeInfoL;

// user interface

//-- query existing types
template <class T> TypeInfoL reg_findDerived();
inline Item *reg_findType(const char* key){
  ItemL types = registry().getDerivedItems<Type>();
  for_list_(Item, ti, types){
    for(uint i=0;i<ti->keys.N;i++) if(ti->keys(i)==key) return ti;
  }
  return NULL;
}
template<class T>
Item *reg_findType(){
  ItemL types = registry().getDerivedItems<Type>();
  for_list_(Item, ti, types){
    if(ti->value<Type>()->typeId()==typeid(T)) return ti;
  }
  return NULL;
}

inline Item* readTypeIntoItem(const char* key, std::istream& is){
  TypeInfoL types = registry().getDerivedValues<Type>();
  Item *ti = reg_findType(key);
  if(ti) return ti->value<Type>()->readItem(is);
  return NULL;
}

template<class T, class Base>
struct Type_typed:Type{
  Type_typed(){}
  Type_typed(const char *userBase, TypeInfoL *container ){
    if(userBase){
      Item *it=reg_findType<Base>();
      if(it) parents.append(it->value<Type>());
    }
    if(container){
      container->append(this);
    }
  }
  virtual const std::type_info& typeId() const { return typeid(T); }
  virtual Item* readItem(istream& is) const{ T *x=new T(); is >>*x; return new Item_typed<T>(x); }
  virtual void* newInstance() const { return new T(); }
};


//-- use these macros to register types in cpp files

#define KO ,
#define REGISTER_TYPE(Type) \
  REGISTER_ITEM2(Type_typed<Type KO void>, Decl_Type, Type, new Type_typed<Type KO void>(NULL,NULL));
//  REGISTER_ITEM2(Type*, type, Type, NULL)

#define REGISTER_TYPE_Key(Key, Type) \
  REGISTER_ITEM2(Type_typed<Type KO void>, Decl_Type, Key, new Type_typed<Type KO void>(NULL,NULL));
//  REGISTER_ITEM2(Type*, type, Key, NULL);

#define REGISTER_TYPE_DERIVED(Type, Base) \
  REGISTER_ITEM2(Type_typed<Type KO Base>, Decl_Type, Type, new Type_typed<Type KO Base>(#Base,NULL));

#endif
