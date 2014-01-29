#pragma once

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <ostream>

struct StreamCollector: std::ostream {
  FILE *file;
  std::stringstream stream;

  StreamCollector(FILE *f);
  StreamCollector(const StreamCollector &sc);
  ~StreamCollector();

  template<class T>
  StreamCollector& operator<<(const T &t);
};

template<class T>
StreamCollector& StreamCollector::operator<<(const T &t) {
  stream << t;
  return *this;
}

struct Feedgnuplot {
  FILE *f;

  bool lines, points, domain, dataid, autolegend;
  const char *title, *hardcopy;
  double stream;

  Feedgnuplot();
  ~Feedgnuplot();

  void setDefault();
  void setLines(bool l);
  void setPoints(bool p);
  void setDomain(bool d);
  void setDataID(bool d);
  void setAutolegend(bool al);
  void setTitle(const char *s);
  void setHardcopy(const char *s);
  void setStream(double s);
  void open();

  StreamCollector operator()();
};

