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

struct RootType { virtual ~RootType() {} }; ///< if types derive from RootType, more tricks are possible

//===========================================================================
//
//  typed Node
//

template<class T>
struct Node_typed : Node {
  T *value;
  bool ownsValue;

  Node_typed():value(NULL), ownsValue(false) { HALT("shouldn't be called, right? Always want to append to container"); }

  /// directly store pointer to value
  Node_typed(Graph& container, T *value, bool ownsValue)
    : Node(container), value(value), ownsValue(ownsValue) {
    CHECK(value || !ownsValue,"you cannot own a NULL value pointer!");
    if(value && typeid(T)==typeid(Graph)) graph().isNodeOfParentGraph = this;
    if(&container && container.callbacks.N) for(GraphEditCallback *cb:container.callbacks) cb->cb_new(this);
  }

  /// directly store pointer to value
  Node_typed(Graph& container, const StringA& keys, const NodeL& parents, T *value, bool ownsValue)
    : Node(container, keys, parents), value(value), ownsValue(ownsValue) {
    CHECK(value || !ownsValue,"you cannot own a NULL value pointer!");
    if(value && typeid(T)==typeid(Graph)) graph().isNodeOfParentGraph = this;
    if(&container && container.callbacks.N) for(GraphEditCallback *cb:container.callbacks) cb->cb_new(this);
  }

  virtual ~Node_typed(){
    if(&container && container.callbacks.N) for(GraphEditCallback *cb:container.callbacks) cb->cb_delete(this);
    if(ownsValue) delete value;
    value=NULL;
  }

  virtual bool hasValue() const {
    return value!=NULL;
  }

  virtual void *getValueDirectly() const {
    return (void*)value;
  }

  virtual void copyValue(Node *it) {
    Node_typed<T> *itt = dynamic_cast<Node_typed<T>*>(it);
    CHECK(itt,"can't assign to wrong type");
    CHECK(itt->value,"can't assign to nothing");
    if(value) delete value;
    value = new T(*itt->value);
  }

  virtual void takeoverValue(Node *it) {
    Node_typed<T> *itt = dynamic_cast<Node_typed<T>*>(it);
    CHECK(itt,"can't assign to wrong type");
    CHECK(itt->value,"can't assign to nothing");
    if(value) delete value;
    value = itt->value;
    itt->value = NULL;
  }

  virtual bool hasEqualValue(Node *it) {
    Node_typed<T> *itt = dynamic_cast<Node_typed<T>*>(it);
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
    if(value){
      if(typeid(T)==typeid(NodeL)) listWrite(*(NodeL*)(value), os, " ");
      else os <<*value;
    }else{
      os <<"<" <<typeid(T).name() <<">";
    }
  }
  
  virtual const std::type_info& getValueType() const {
    return typeid(T);
  }
  
  virtual bool is_derived_from_RootType() const {
    return MLR_is_base_of<RootType, T>::value;
  }
  
  virtual Node* newClone(Graph& container) const {
    if(!value) return new Node_typed<T>(container, keys, parents, (T*)NULL, false);
    if(getValueType()==typeid(Graph)){
      Graph *g = new Graph();
      g->copy(*getValue<Graph>(), &container);
      return g->isNodeOfParentGraph;
    }
    return new Node_typed<T>(container, keys, parents, new T(*value), true);
  }
};

template<class T> T *Node::getValue() {
  Node_typed<T>* typed = dynamic_cast<Node_typed<T>*>(this);
  if(!typed) {
    if(getValueType() == typeid(Graph)){ //try to get the item from the key value graph
      const Graph *graph = getValue<Graph>();
      if(graph->N==1){ //only if it has size 1??
        typed = dynamic_cast<Node_typed<T>*>(graph->elem(0));
      }
    }
    if(!typed){
      MT_MSG("can't cast type '" <<getValueType().name() <<"' to type '" <<typeid(T).name() <<"' -- returning NULL");
      return NULL;
    }
  }
  return typed->value;
}

template<class T> const T *Node::getValue() const {
  const Node_typed<T>* typed = dynamic_cast<const Node_typed<T>*>(this);
  if(!typed) {
    if(getValueType() == typeid(Graph)){ //try to get the item from the key value graph
      const Graph *graph = getValue<Graph>();
      if(graph->N==1){ //only if it has size 1??
        typed = dynamic_cast<const Node_typed<T>*>(graph->elem(0));
      }
    }
    MT_MSG("can't cast type '" <<getValueType().name() <<"' to type '" <<typeid(T).name() <<"' -- returning reference-to-NULL");
    return NULL;
  }
  return typed->value;
}

template<class T> NodeInitializer::NodeInitializer(const char* key, const T& x){
  it = new Node_typed<T>(G, new T(x), true);
  it->keys.append(STRING(key));
}

template<class T> NodeInitializer::NodeInitializer(const char* key, const StringA& parents, const T& x)
  : parents(parents){
  it = new Node_typed<T>(G, new T(x), true);
  it->keys.append(STRING(key));
}

template<class T> T& Graph::get(const char *key) const {
  Node *it = getNode(key);
  if(!it) HALT("node '"<< key<< "' does not exist (to retrieve type '"<<typeid(T).name() <<"')");
  T* val=it->getValue<T>();
  if(!val) HALT("node " <<*it <<" does not have type '"<<typeid(T).name() <<"'");
  return *val;
}

template<class T> const T& Graph::get(const char *key, const T& defaultValue) const{
  Node *it = getNode(key);
  if(!it) return defaultValue;
  T* val=it->getValue<T>();
  if(!val) return defaultValue;
  return *val;
}

template<class T> T* Graph::getValue(const char *key) const {
  Node *it = getNode(key);
  if(!it) return NULL;
  return it->getValue<T>();
}

template<class T> T* Graph::getValue(const StringA &keys) const {
  Node *it = getNode(keys);
  if(!it) return NULL;
  return it->getValue<T>();
}

template<class T> MT::Array<T*> Graph::getTypedValues(const char* key) {
  MT::Array<T*> ret;
  for(Node *it: (*this)) if(it->getValueType()==typeid(T)) {
    if(!key) ret.append(it->getValue<T>());
    else for(uint i=0; i<it->keys.N; i++) if(it->keys(i)==key) {
      ret.append(it->getValue<T>());
      break;
    }
  }
  return ret;
}

template<class T> Node *Graph::append(T *x, bool ownsValue) {
  return new Node_typed<T>(*this, x, ownsValue);
}

//template<class T> Node *Graph::append(const char* key, T *x, bool ownsValue) {
//  return new Node_typed<T>(*this, {MT::String(key)}, {}, x, ownsValue);
//}

template<class T> Node *Graph::append(const StringA& keys, const NodeL& parents, const T& x){
  return new Node_typed<T>(*this, keys, parents, new T(x), true);
}

template<class T> Node *Graph::append(const StringA& keys, const NodeL& parents, T *x, bool ownsValue) {
  return new Node_typed<T>(*this, keys, parents, x, ownsValue);
}

template <class T> MT::Array<T*> Graph::getDerivedValues() {
  MT::Array<T*> ret;
  for(Node *it: (*this)) {
    if(it->is_derived_from_RootType()) {
      T *val= dynamic_cast<T*>(((Node_typed<RootType>*)it)->value);
      if(val) ret.append(val);
    }
  }
  return ret;
}

template <class T> NodeL Graph::getDerivedNodes() {
  NodeL ret;
  for(Node *it: (*this)) {
    if(it->is_derived_from_RootType()) {
      T *val= dynamic_cast<T*>(((Node_typed<RootType>*)it)->value);
      if(val) ret.append(it);
    }
  }
  return ret;
}
