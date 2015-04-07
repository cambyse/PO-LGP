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

  bool lines, points, domain, dim3d, dataid, autolegend, ymin_b, ymax_b, dump;
  const char *title, *hardcopy;
  double stream;
  double ymin, ymax;

  FGPlot();
  ~FGPlot();

  void setDefault();
  void setLines(bool l);
  void setPoints(bool p);
  void setDomain(bool d);
  void setDim3D(bool d);
  void setDataID(bool d);
  void setAutolegend(bool al);
  void setTitle(const char *s);
  void setHardcopy(const char *s);
  void setStream(double s);
  void setYMin(double min);
  void setYMax(double max);
  void setDump(bool d);

  bool isDim3D();

  void open();

  StreamCollector operator()();
};

#include <Core/graph.h>
struct FGPlots {
  struct sFGPlots;

  sFGPlots *s;
  const Graph *kvg;
  FGPlots();
  ~FGPlots();

  void open(const Graph &kvg);
  void step(uint t);
  void replot();
  void exit();
};

