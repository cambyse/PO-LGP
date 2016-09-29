/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "graph.h"

//===========================================================================
//
//  typed Node
//

template<class T>
struct Node_typed : Node {
  T value;

  Node_typed():value(NULL) { HALT("shouldn't be called, right? You always want to append to a container"); }

  /// directly store pointer to value
  Node_typed(Graph& container, const T& _value)
    : Node(typeid(T), &this->value, container), value(_value) {
    if(isGraph()) graph().isNodeOfGraph = this; //this is the only place where isNodeOfGraph is set
    if(&container && container.callbacks.N) for(GraphEditCallback *cb:container.callbacks) cb->cb_new(this);
  }

  /// directly store pointer to value
  Node_typed(Graph& container, const StringA& keys, const NodeL& parents, const T& _value)
    : Node(typeid(T), &this->value, container, keys, parents), value(_value) {
    if(isGraph()) graph().isNodeOfGraph = this; //this is the only place where isNodeOfGraph is set
    if(&container && container.callbacks.N) for(GraphEditCallback *cb:container.callbacks) cb->cb_new(this);
  }

  virtual ~Node_typed(){
    if(&container && container.callbacks.N) for(GraphEditCallback *cb:container.callbacks) cb->cb_delete(this);
  }

  virtual void copyValue(Node *it) {
    Node_typed<T> *itt = dynamic_cast<Node_typed<T>*>(it);
    CHECK(itt,"can't assign to wrong type");
    value = itt->value;
  }

  virtual bool hasEqualValue(Node *it) {
    Node_typed<T> *itt = dynamic_cast<Node_typed<T>*>(it);
    CHECK(itt,"can't compare to wrong type");
    return value == itt->value;
  }

  virtual void writeValue(std::ostream &os) const {
    if(typeid(T)==typeid(NodeL)) listWrite(*getValue<NodeL>(), os, " ");
    else os <<value;
  }
  
  virtual const std::type_info& getValueType() const {
    return typeid(T);
  }
  
  virtual Node* newClone(Graph& container) const {
    if(isGraph()){
      Node_typed<Graph> *n = container.newSubgraph(keys, parents);
      n->value.copy(graph());
      return n;
    }
    return new Node_typed<T>(container, keys, parents, value);
  }
};

template<class T> T* Node::getValue() {
  Node_typed<T>* typed = dynamic_cast<Node_typed<T>*>(this);
  if(!typed) return NULL;
  return &typed->value;
}

template<class T> const T* Node::getValue() const {
  const Node_typed<T>* typed = dynamic_cast<const Node_typed<T>*>(this);
  if(!typed) return NULL;
  return &typed->value;
}

template<class T> Nod::Nod(const char* key, const T& x){
  n = new Node_typed<T>(G, x);
  n->keys.append(STRING(key));
}

template<class T> Nod::Nod(const char* key, const StringA& parents, const T& x)
  : parents(parents){
  n = new Node_typed<T>(G, x);
  n->keys.append(STRING(key));
}

template<class T> T& Graph::get(const char *key) const {
  Node *n = findNodeOfType(typeid(T), {key});
  if(!n) HALT("no node of type '" <<typeid(T).name() <<"' with key '"<< key<< "' found");
  return n->get<T>();
}

template<class T> T& Graph::get(const StringA& keys) const {
  Node *n = findNodeOfType(typeid(T), keys);
  if(!n) HALT("no node of type '" <<typeid(T).name() <<"' with keys '"<< keys<< "' found");
  return n->get<T>();
}

template<class T> const T& Graph::get(const char *key, const T& defaultValue) const{
  Node *n = findNodeOfType(typeid(T), {key});
  if(!n) return defaultValue;
  return n->get<T>();
}

template<class T> mlr::Array<T*> Graph::getValuesOfType(const char* key) {
  NodeL nodes = findNodesOfType(typeid(T), {key});
  mlr::Array<T*> ret;
  for(Node *n: nodes) ret.append(n->getValue<T>());
  return ret;
}

template<class T> Node_typed<T> *Graph::append(const StringA& keys, const NodeL& parents, const T& x){
  return new Node_typed<T>(*this, keys, parents, x);
}



