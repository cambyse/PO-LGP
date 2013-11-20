#pragma once

#include <Core/module.h>

struct UEyePoller: Module {
  struct sUEyeInterface *s;

  ACCESS(uint, cid);
  ACCESS(int, ueye_num);
  //ACCESS(byteA, ueye_rgb);
  //ACCESS(floatA, ueye_fps);

  UEyePoller();
  virtual ~UEyePoller();

  void open(uint _cid);
  void step();
  void close();
};

