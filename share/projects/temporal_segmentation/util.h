#pragma once

#include <iomanip>
#include <iostream>
#include <ostream>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <string>
#include <functional>
#include <algorithm>
#include <chrono>

#include <Core/array.h>

// StreamCollector {{{
struct StreamCollector: std::ostream {
  FILE *file;
  std::stringstream stream;

  StreamCollector(FILE *_file);
  StreamCollector(const StreamCollector &sc);
  ~StreamCollector();

  template<class T>
  StreamCollector& operator<<(const T &t);
};
// }}}
// Collector {{{
struct Collector: public std::ostream {
  std::function<void(std::string)> f;
  std::stringstream stream;

  Collector(std::function<void(std::string)> _f);
  Collector(const Collector &c);
  ~Collector();

  template<class T>
  Collector& operator<<(const T &t);
};
// }}}
// ProgressBar {{{
struct ProgressBar {
  private:
  int N, n;
  //const char *wheel[4] = { "◐", "◑", "◒", "◓" };
  const char *wheel[4] = { "◜ ", " ◝", " ◞", "◟ " };

  public:
  uint wa, wb;
  String prefix;

   ProgressBar(): wa(15), wb(40) { reset(0); };
  ~ProgressBar() {};

  void reset(int _N);
  void width(uint a, uint b);
  Collector step(bool done = false);
  Collector stop();

  private:
  void bar(String &str, uint n, uint N, bool done = false);
  void slide(String &str, uint n, bool done = false);
};
// }}}
template<class T>
void split(MT::Array<MT::Array<T*> > &trainlist, MT::Array<MT::Array<T*> > &testlist, const MT::Array<T*> &list, uint nsplits);

// namespace watch {{{
namespace watch {
  std::chrono::time_point<std::chrono::steady_clock> now();
}
// }}}
#include "util_t.h"
