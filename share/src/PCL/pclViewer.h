#pragma once

#include <Core/thread.h>

#include "conv.h"

struct PclViewer : Thread {
  struct sPclViewer *s;
  Access_typed<Pcl> cloud;
  PclViewer(const char* cloud_name);
  ~PclViewer();
  void open();
  void step();
  void close();
};
