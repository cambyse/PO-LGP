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
#include <stack>
#include <map>

#include <Core/array.h>

// gen purpose enum defines
#define ENUM_ELEM(elem) elem,
#define ENUM_CASE(elem) case elem: return #elem;

// use into .h file
#define ENUM_MACRO_H(enum_name)                     \
  enum enum_name {                                  \
    enum_name ## _list(ENUM_ELEM)                   \
  };                                                \
const char *enum_name ## _to_str(enum_name elem);   \
enum_name str_to_ ## enum_name(const char *str);

// use into .cpp file
#define ENUM_MACRO_CPP(enum_name)                               \
const char *enum_name ## _to_str(enum_name elem) {              \
  switch(elem) {                                                \
    enum_name ## _list(ENUM_CASE)                               \
    default:                                                    \
      HALT(#enum_name " not defined?");                         \
  }                                                             \
};                                                              \
Graph enum_name ## _kvg;                                \
enum_name str_to_ ## enum_name(const char *str) {               \
  if(enum_name ## _kvg.N == 0) {                                \
    enum_name ## _list(ENUM_KVG)                                \
  }                                                             \
  enum_name *elem = enum_name ## _kvg.find<enum_name>(str); \
  return *elem;                                                 \
}

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
void split(mlr::Array<mlr::Array<T*> > &trainlist, mlr::Array<mlr::Array<T*> > &testlist, const mlr::Array<T*> &list, uint nsplits);

// namespace watch {{{
namespace watch {
  extern std::stack<std::chrono::time_point<std::chrono::steady_clock> > stack;

  // typedef watch_t std::chrono::time_cpoint<std::chrono::steady_clock>
  std::chrono::time_point<std::chrono::steady_clock> now();
  void push();
}
// }}}
// namespace hash {{{
namespace hash {
  extern std::hash<std::string> hash;
  extern std::map<std::string, size_t> map;

  void clear();
  void clear(const std::string &key);
  void append(const std::string &key, const std::string &value);
  std::string get(const std::string &key);
};
// }}}
#include "utilAB_t.h"
