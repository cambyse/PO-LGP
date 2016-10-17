#pragma once

#include <Core/array.h>
#include <Ors/ors.h>
#include "mocapdata.h"

typedef std::function<void(ors::KinematicWorld &kw)> kw_init_function;
typedef std::function<void(ors::KinematicWorld &kw, uint f)> kw_update_function;

struct MocapGui {
  struct sMocapGui *s;

  MocapGui(MocapRec &rec);
  ~MocapGui();

  ors::KinematicWorld &kw();

  kw_init_function &init_custom();
  kw_update_function &update_custom();

  void play(Target target = NO_TARGET);
  void annotate(const String &name, uint cardinality);
  void show();
};

