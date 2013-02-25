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

struct TypeInfo:TypeBase{
  MT::Array<TypeInfo*> parents;
  virtual const std::type_info& type_info() const {NIY};
  virtual struct Item* readItem(istream&) const {NIY};
  virtual void* newInstance() const {NIY}
  void write(std::ostream& os) const{
    os <<"Type '" <<type_info().name() <<"'] ";
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
inline Item *reg_findType(const char* key){
  ItemL types = registry().getDerivedItems<TypeInfo>();
  for_list_(Item, ti, types){
    for(uint i=0;i<ti->keys.N;i++) if(ti->keys(i)==key) return ti;
  }
  return NULL;
}
template<class T>
Item *reg_findType(){
  ItemL types = registry().getDerivedItems<TypeInfo>();
  for_list_(Item, ti, types){
    if(ti->value<TypeInfo>()->type_info()==typeid(T)) return ti;
  }
  return NULL;
}

inline Item* readTypeIntoItem(const char* key, std::istream& is){
  TypeInfoL types = registry().getDerivedValues<TypeInfo>();
  Item *ti = reg_findType(key);
  if(ti) return ti->value<TypeInfo>()->readItem(is);
  return NULL;
}

template<class Type, class Base>
struct TypeInfo_typed:TypeInfo{
  TypeInfo_typed(){}
  TypeInfo_typed(const char *userBase, TypeInfoL *container ){
    if(userBase){
      Item *it=reg_findType<Base>();
      if(it) parents.append(it->value<TypeInfo>());
    }
    if(container){
      container->append(this);
    }
  }
  virtual const std::type_info& type_info() const { return typeid(Type); }
  virtual Item* readItem(istream& is) const{ Type *x=new Type(); is >>*x; return new Item_typed<Type>(x); }
  virtual void* newInstance() const { return new Type(); }
};


//-- use these macros to register types in cpp files

#define KO ,
#define REGISTER_TYPE(Type) \
  REGISTER_ITEM2(TypeInfo_typed<Type KO void>, Decl_Type, Type, new TypeInfo_typed<Type KO void>(NULL,NULL));
//  REGISTER_ITEM2(Type*, type, Type, NULL)

#define REGISTER_TYPE_Key(Key, Type) \
  REGISTER_ITEM2(TypeInfo_typed<Type KO void>, Decl_Type, Key, new TypeInfo_typed<Type KO void>(NULL,NULL));
//  REGISTER_ITEM2(Type*, type, Key, NULL);

#define REGISTER_TYPE_DERIVED(Type, Base) \
  REGISTER_ITEM2(TypeInfo_typed<Type KO Base>, Decl_Type, Type, new TypeInfo_typed<Type KO Base>(#Base,NULL));

#endif
