#include "mapGraph.h"

//===========================================================================
//
// stuff that should be hidden away - not for the user
//

template<class T> typename Singleton<T>::SingletonFields *Singleton<T>::singleton=NULL;

template<class Type, class Base>
struct TypeInfo_typed:TypeInfo{
  TypeInfo_typed(){}
  TypeInfo_typed(const char *key1, const char *key2, const char *userBase, TypeInfoL *container ){
    if(key1) keys.append(MT::String(key1));
    if(key2) keys.append(MT::String(key2));
    keys.append(MT::String(typeid(Type).name()));
    if(userBase){
      TypeInfo *t=reg_findType<Base>();
      if(t) parents.append(t);
    }
    if(container){
      container->append(this);
    }
  }
  virtual const std::type_info& type_info() const { return typeid(Type); }
  virtual Item* readItem(istream& is) const{ Type *x=new Type(); is >>*x; return new Item_typed<Type>(x); }
  virtual void* newInstance() const { return new Type(); }
};

/*
template <class T>
TypeInfoL reg_findDerived(){
  TypeInfoL results;
  TypeInfo *t, *p;
  uint i,j;
  for_list(i, t, typeRegistry.get()){
    for_list(j, p, t->parents){
      if(p->typeinfo()==typeid(T)){ results.append(t); }
    }
  }
  return results;
}
*/
