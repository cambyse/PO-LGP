#pragma once

#include <Core/module.h>

struct UEyePoller: Module {
  struct sUEyeInterface *s;

  ACCESS(int, ueye_num);
  ACCESS(byteA, ueye_rgb); // TODO a list of
  ACCESS(floatA, ueye_fps); // TODO a list of

  UEyePoller();
  virtual ~UEyePoller();

  void open();
  void step();
  void close();
};

