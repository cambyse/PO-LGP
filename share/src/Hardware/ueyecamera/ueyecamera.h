#pragma once

#include<sys/time.h>
#include <Core/module.h>

namespace MLR {
class UEyeInterface {
private:
	struct sUEyeInterface *s;
public:
	UEyeInterface(const uint cameraID);
};
}

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

