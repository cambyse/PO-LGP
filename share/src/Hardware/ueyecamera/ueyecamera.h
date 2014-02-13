#pragma once

#include <Core/module.h>

struct UEyePoller: Module {
  struct sUEyeInterface *s;

  ACCESS(byteA, ueye_rgb);
  ACCESS(int, ueye_num);
  ACCESS(floatA, ueye_fps);
  ACCESS(uint, cid);

  UEyePoller();
  virtual ~UEyePoller();

  //void open(uint _cid);
  void open();
  void step();
  void close();
};

