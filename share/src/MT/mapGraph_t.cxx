//===========================================================================
//
//  typed Item
//

template<class T>
struct Item_typed:Item {
  T value;

  Item_typed(const T& _value):value(_value){}

  Item_typed(const StringL& _keys, const ItemL& _parents, const T& _value):value(_value){
    keys=_keys;
    parents=_parents;
  }

  virtual void writeValue(std::ostream &os) const {
    if(typeid(T)==typeid(ItemL)) listWrite(*(ItemL*)(&value), os, " ");
    else os <<value;
  }

  virtual const std::type_info& valueType() const {
    return typeid(T);
  }
};

template<class T> T& Item::value(){
  Item_typed<T>* typed = dynamic_cast<Item_typed<T>*>(this);
  if(!typed){
    MT_MSG("can't cast type '" <<valueType().name() <<"' to type '" <<typeid(T).name() <<"' -- returning reference-to-NULL");
    return *((T*)NULL);
  }
  return typed->value;
}

template<class T> const T& Item::value() const{
  const Item_typed<T>* typed = dynamic_cast<const Item_typed<T>*>(this);
  if(!typed){
    MT_MSG("can't cast type '" <<valueType().name() <<"' to type '" <<typeid(T).name() <<"' -- returning reference-to-NULL");
    return *((T*)NULL);
  }
  return typed->value;
}


template<class T> T& MapGraph::value(const char *key){
  Item *it = item(key);
  if(!it) return *((T*)NULL);
  return it->value<T>();
}
