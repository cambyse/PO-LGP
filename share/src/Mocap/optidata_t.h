template <class T>
mlr::Array<T> OptiID::query(const mlr::Array<T> &data, const String &sensor) {
  return data[sensor_rn(sensor)];
}

template <class T>
mlr::Array<T> OptiID::query(const mlr::Array<T>  &data, const StringA &sensors) {
  mlr::Array<T> x;
  for(const String &sensor: sensors) {
    x.append(data[sensor_rn(sensor)]);
  }
  uint nsensors = sensors.N;
  x.reshape(nsensors, x.N/nsensors);
  return x;
}
