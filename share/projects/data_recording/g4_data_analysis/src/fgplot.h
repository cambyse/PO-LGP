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

struct FGPlot {
  FILE *f;

  bool lines, points, domain, dataid, autolegend, ymin_b, ymax_b;
  const char *title, *hardcopy;
  double stream;
  double ymin, ymax;

  FGPlot();
  ~FGPlot();

  void setDefault();
  void setLines(bool l);
  void setPoints(bool p);
  void setDomain(bool d);
  void setDataID(bool d);
  void setAutolegend(bool al);
  void setTitle(const char *s);
  void setHardcopy(const char *s);
  void setStream(double s);
  void setYMin(double min);
  void setYMax(double max);
  void open();

  StreamCollector operator()();
};

#include <Core/keyValueGraph.h>
struct FGPlots {
  struct sFGPlots;

  sFGPlots *s;
  const KeyValueGraph *kvg;
  FGPlots();
  ~FGPlots();

  void open(const KeyValueGraph &kvg);
  void step(uint t);
};

