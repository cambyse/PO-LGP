#pragma once

#include<sys/time.h>
#include <Core/module.h>

struct UEyePoller: Module {
  struct sUEyeInterface *s;

  ACCESS(byteA, ueye_rgb);
  //ACCESS(double, ueye_fps);

  UEyePoller();
  virtual ~UEyePoller();

  void open();
  void step();
  void close();
};

