template <class T>
MT::Array<T> OptiID::query(const MT::Array<T> &data, const String &sensor) {
  return data[sensor_rn(sensor)];
}

template <class T>
MT::Array<T> OptiID::query(const MT::Array<T>  &data, const StringA &sensors) {
  MT::Array<T> x;
  for(const String &sensor: sensors) {
    x.append(data[sensor_rn(sensor)]);
  }
  uint nsensors = sensors.N;
  x.reshape(nsensors, x.N/nsensors);
  return x;
}
