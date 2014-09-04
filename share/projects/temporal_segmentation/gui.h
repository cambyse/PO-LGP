#pragma once
#include <Core/keyValueGraph.h>
#include "util.h"

// FGP {{{
struct FGP {
  FILE *f;
  Params defParams, params;

  public:
    FGP();
    void setDefaultParams();

    template<class T>
    void set(const char *key, const T &value);
    void reset();

    void open();
    void close();

    void plot();
    void exit();

    Collector operator()();
};
// }}}

#include "gui_t.h"
