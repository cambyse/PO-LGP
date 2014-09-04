#pragma once

#include <Core/array.h>

struct G4ID {
  struct sG4ID;
  sG4ID *s;

  G4ID();
  ~G4ID();

  void clear();
  void load(const char *meta);

  // TODO change some strings into const char * (mostly the input ones)
  const StringA& sensors();
  const StringA& struct_sensors();
  const StringA& unstruct_sensors();
  const StringA& sensorsof(const char *obj);

  const StringA& subjects();
  const StringA& objects();

  const StringA& agents();
  const StringA& limbs();
  const StringA& digits();

  const StringA& digitsof(const String &limb);
  const StringA& sublimbs(const String &limb);
  const String& suplimb(const String &limb);

  uint hsitoi(uint hsi);
  uint itohsi(uint i);

  int i(const char *sensor);
  int hsi(const char *sensor);
  const char *sensor(uint hsi);

  template <class T>
  MT::Array<T> query(const MT::Array<T> &data, const String &sensor) {
    return data[hsi(sensor)];
  }

  template <class T>
  MT::Array<T> query(const MT::Array<T> &data, const StringA &sensors) {
    MT::Array<T> x;
    for(const String &sensor: sensors)
      x.append(data[hsi(sensor)]);
    uint nsensors = sensors.N;
    x.reshape(nsensors, x.N / nsensors);
    return x;
  }

  /* arr query(const String &sensor) jjjj */
  /*   // TODO */
  /*   // check that you have an instance of G4Data, to get the data.. */
  /* } */

  void write(std::ostream &os = std::cout) const;
};
stdOutPipe(G4ID);
