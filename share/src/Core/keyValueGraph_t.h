#include "keyValueGraph.h"

//////////// taken from http://stackoverflow.com/questions/4532281/how-to-test-whether-class-b-is-derived-from-class-a
typedef char(&yes)[1];
typedef char(&no)[2];
template <typename B, typename D> struct Host {
  operator B*() const;
  operator D*();
};
template <typename B, typename D> struct is_base_of {
  template <typename T>
  static yes check(D*, T);
  static no check(B*, int);
  static const bool value = sizeof(check(Host<B,D>(), int())) == sizeof(yes);
};
///////////////STOP

//===========================================================================
//
//  typed Item
//

template<class T>
struct Item_typed:Item {
  T *value;
  
  Item_typed():value(NULL) {}
  Item_typed(T *_value):value(_value) {}
  //copy value
  Item_typed(const StringA& _keys, const ItemL& _parents, const T& _value, KeyValueGraph *container=NULL):value(NULL) {
    value = new T(_value);
    keys=_keys;
    parents=_parents;
    if(container) container->append(this);
  }
  
  //directly store pointer to value
  Item_typed(const StringA& _keys, const ItemL& _parents, T *_value=NULL, KeyValueGraph *container=NULL):value(_value) {
    keys=_keys;
    parents=_parents;
    if(container) container->append(this);
  }
  
  virtual void writeValue(std::ostream &os) const {
    if(typeid(T)==typeid(ItemL)) listWrite(*(ItemL*)(value), os, " ");
    else os <<*value;
  }
  
  virtual const std::type_info& valueType() const {
    return typeid(T);
  }
  
  virtual bool is_derived_from_RootType() const {
    return is_base_of<RootType, T>::value;
  }
  
  virtual Item *newClone() const { return new Item_typed<T>(keys, parents, value); }
};

template<class T> T *Item::value() {
  Item_typed<T>* typed = dynamic_cast<Item_typed<T>*>(this);
  if(!typed) {
    MT_MSG("can't cast type '" <<valueType().name() <<"' to type '" <<typeid(T).name() <<"' -- returning NULL");
    return NULL;
  }
  return typed->value;
}

template<class T> const T *Item::value() const {
  const Item_typed<T>* typed = dynamic_cast<const Item_typed<T>*>(this);
  if(!typed) {
    MT_MSG("can't cast type '" <<valueType().name() <<"' to type '" <<typeid(T).name() <<"' -- returning reference-to-NULL");
    return NULL;
  }
  return typed->value;
}


template<class T> T* KeyValueGraph::getValue(const char *key) {
  Item *it = getItem(key);
  if(!it) return NULL;
  return it->value<T>();
}

template<class T> MT::Array<T*> KeyValueGraph::getTypedValues(const char* key) {
  MT::Array<T*> ret;
  for_list_(Item, it, (*this)) if(it->valueType()==typeid(T)) {
    if(!key) ret.append(it->value<T>());
    else for(uint i=0; i<it->keys.N; i++) if(it->keys(i)==key) {
          ret.append(it->value<T>());
          break;
        }
  }
  return ret;
}

template<class T> KeyValueGraph KeyValueGraph::getTypedItems(const char* key) {
  KeyValueGraph ret;
  for_list_(Item, it, (*this)) if(it->valueType()==typeid(T)) {
    if(!key) ret.append(it);
    else for(uint i=0; i<it->keys.N; i++) if(it->keys(i)==key) {
          ret.append(it);
          break;
        }
  }
  return ret;
}

template<class T> Item *KeyValueGraph::append(const StringA& keys, const ItemL& parents, T *x) {
  Item *it= ItemL::append(new Item_typed<T>(keys, parents, x, NULL));
  for_list_(Item, par, parents) par->parentOf.append(it);
  return it;
}

template <class T> MT::Array<T*> KeyValueGraph::getDerivedValues() {
  MT::Array<T*> ret;
  for_list_(Item, it, (*this)) {
    if(it->is_derived_from_RootType()) {
      T *val= dynamic_cast<T*>(((Item_typed<RootType>*)it)->value);
      if(val) ret.append(val);
    }
  }
  return ret;
}

template <class T> ItemL KeyValueGraph::getDerivedItems() {
  ItemL ret;
  for_list_(Item, it, (*this)) {
    if(it->is_derived_from_RootType()) {
      T *val= dynamic_cast<T*>(((Item_typed<RootType>*)it)->value);
      if(val) ret.append(it);
    }
  }
  return ret;
}