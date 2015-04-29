/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */

#include "graph.h"

//////////// taken from http://stackoverflow.com/questions/4532281/how-to-test-whether-class-b-is-derived-from-class-a
typedef char(&yes)[1];
typedef char(&no)[2];
template <typename B, typename D> struct Host {
  operator B*() const;
  operator D*();
};
template <typename B, typename D> struct MLR_is_base_of {
  template <typename T>
  static yes check(D*, T);
  static no check(B*, int);
  static const bool value = sizeof(check(Host<B,D>(), int())) == sizeof(yes);
};
///////////////STOP

struct RootType { virtual ~RootType() {}; }; ///< if types derive from RootType, more tricks are possible

//===========================================================================
//
//  typed Item
//

template<class T>
struct Item_typed:Item {
  T *value;
  bool ownsValue;

  Item_typed():value(NULL), ownsValue(false) { HALT("shouldn't be called, right? Always want to append to container"); }

  /// directly store pointer to value
  Item_typed(Graph& container, T *value, bool ownsValue):Item(container), value(value), ownsValue(ownsValue) {
    CHECK(value || !ownsValue,"you cannot own a NULL value pointer!");
    if(typeid(T)==typeid(Graph)) kvg().isItemOfParentKvg = this;
  }

  /// directly store pointer to value
  Item_typed(Graph& container, const StringA& _keys, const ItemL& parents, T *value, bool ownsValue)
    : Item(container, parents), value(value), ownsValue(ownsValue) {
    CHECK(value || !ownsValue,"you cannot own a NULL value pointer!");
    keys=_keys;
    if(typeid(T)==typeid(Graph)) kvg().isItemOfParentKvg = this;
  }

//  /// copy value
//  Item_typed(Graph& container, const StringA& _keys, const ItemL& parents, const T& _value)
//    : Item(container, parents), value(NULL), ownsValue(true) {
//    value = new T(_value);
//    keys=_keys;
//    if(typeid(T)==typeid(Graph)) kvg().isItemOfParentKvg = this;
//  }

  virtual ~Item_typed(){
    if(ownsValue) delete value;
  }

  virtual bool hasValue() const {
    return value!=NULL;
  }

  virtual void *getValueDirectly() const {
    return (void*)value;
  }

  virtual void copyValue(Item *it) {
    Item_typed<T> *itt = dynamic_cast<Item_typed<T>*>(it);
    CHECK(itt,"can't assign to wrong type");
    CHECK(itt->value,"can't assign to nothing");
    if(value) delete value;
    value = new T(*itt->value);
  }

  virtual void takeoverValue(Item *it) {
    Item_typed<T> *itt = dynamic_cast<Item_typed<T>*>(it);
    CHECK(itt,"can't assign to wrong type");
    CHECK(itt->value,"can't assign to nothing");
    if(value) delete value;
    value = itt->value;
    itt->value = NULL;
  }

  virtual bool hasEqualValue(Item *it) {
    Item_typed<T> *itt = dynamic_cast<Item_typed<T>*>(it);
    CHECK(itt,"can't assign to wrong type");
    if(!itt->value || !value) return false;
#ifdef MT_CLANG
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wdynamic-class-memaccess"
#endif
    return memcmp(itt->value, value, sizeof(T))==0;
#ifdef MT_CLANG
#  pragma clang diagnostic pop
#endif
  }

  virtual void writeValue(std::ostream &os) const {
    if(typeid(T)==typeid(ItemL)) listWrite(*(ItemL*)(value), os, " ");
    else os <<*value;
  }
  
  virtual const std::type_info& getValueType() const {
    return typeid(T);
  }
  
  virtual bool is_derived_from_RootType() const {
    return MLR_is_base_of<RootType, T>::value;
  }
  
  virtual Item *newClone(Graph& container) const {
    if(!value) return new Item_typed<T>(container, keys, parents, (T*)NULL, false);
    return new Item_typed<T>(container, keys, parents, new T(*value), true);
  }
};

template<class T> T *Item::getValue() {
  Item_typed<T>* typed = dynamic_cast<Item_typed<T>*>(this);
  if(!typed) {
    if(getValueType() == typeid(Graph)){ //try to get the item from the key value graph
      const Graph *kvg = getValue<Graph>();
      if(kvg->N==1){ //only if it has size 1??
        typed = dynamic_cast<Item_typed<T>*>(kvg->elem(0));
      }
    }
    if(!typed){
      MT_MSG("can't cast type '" <<getValueType().name() <<"' to type '" <<typeid(T).name() <<"' -- returning NULL");
      return NULL;
    }
  }
  return typed->value;
}

template<class T> const T *Item::getValue() const {
  const Item_typed<T>* typed = dynamic_cast<const Item_typed<T>*>(this);
  if(!typed) {
    if(getValueType() == typeid(Graph)){ //try to get the item from the key value graph
      const Graph *kvg = getValue<Graph>();
      if(kvg->N==1){ //only if it has size 1??
        typed = dynamic_cast<const Item_typed<T>*>(kvg->elem(0));
      }
    }
    MT_MSG("can't cast type '" <<getValueType().name() <<"' to type '" <<typeid(T).name() <<"' -- returning reference-to-NULL");
    return NULL;
  }
  return typed->value;
}

template<class T> ItemInitializer::ItemInitializer(const char* key, const T& x){
  it = new Item_typed<T>(NoGraph, new T(x), true);
  it->keys.append(STRING(key));
}

template<class T> ItemInitializer::ItemInitializer(const char* key, const StringA& parents, const T& x)
  : parents(parents){
  it = new Item_typed<T>(NoGraph, new T(x), true);
  it->keys.append(STRING(key));
}

template<class T> T* Graph::getValue(const char *key) {
  Item *it = getItem(key);
  if(!it) return NULL;
  return it->getValue<T>();
}

template<class T> T* Graph::getValue(const StringA &keys) {
  Item *it = getItem(keys);
  if(!it) return NULL;
  return it->getValue<T>();
}

template<class T> MT::Array<T*> Graph::getTypedValues(const char* key) {
  MT::Array<T*> ret;
  for(Item *it: (*this)) if(it->getValueType()==typeid(T)) {
    if(!key) ret.append(it->getValue<T>());
    else for(uint i=0; i<it->keys.N; i++) if(it->keys(i)==key) {
      ret.append(it->getValue<T>());
      break;
    }
  }
  return ret;
}

template<class T> Item *Graph::append(T *x, bool ownsValue) {
  return new Item_typed<T>(*this, x, ownsValue);
}

template<class T> Item *Graph::append(const char* key, T *x, bool ownsValue) {
  return new Item_typed<T>(*this, {MT::String(key)}, {}, x, ownsValue);
}

template<class T> Item *Graph::append(const StringA& keys, const ItemL& parents, T *x, bool ownsValue) {
  return new Item_typed<T>(*this, keys, parents, x, ownsValue);
}

template <class T> MT::Array<T*> Graph::getDerivedValues() {
  MT::Array<T*> ret;
  for(Item *it: (*this)) {
    if(it->is_derived_from_RootType()) {
      T *val= dynamic_cast<T*>(((Item_typed<RootType>*)it)->value);
      if(val) ret.append(val);
    }
  }
  return ret;
}

template <class T> ItemL Graph::getDerivedItems() {
  ItemL ret;
  for(Item *it: (*this)) {
    if(it->is_derived_from_RootType()) {
      T *val= dynamic_cast<T*>(((Item_typed<RootType>*)it)->value);
      if(val) ret.append(it);
    }
  }
  return ret;
}
