template <class T>
MT::Array<T> G4ID::query(const MT::Array<T> &data, const String &sensor) {
  return data[hsi(sensor)];
}

template <class T>
MT::Array<T> G4ID::query(const MT::Array<T>  &data, const StringA &sensors) {
  MT::Array<T> x;
  for(const String &sensor: sensors) {
    x.append(data[hsi(sensor)]);
  }
  uint nsensors = sensors.N;
  x.reshape(nsensors, x.N/nsensors);
  return x;
}

template<class T>
void G4Rec::set(const char *key, const T &value) {
  Item *i = params.getItem(key);
  if(i)
    *i->getValue<T>() = value;
  else
    params.append(key, new T(value));
}

template<class T>
bool G4Rec::get(const char *key, T &value) {
  return params.getValue(value, key);
}

template<class T>
T* G4Rec::get(const char *key) {
  return params.getValue<T>(key);
}

