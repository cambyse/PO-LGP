#ifndef NJ_UrgModule_h
#define NJ_UrgModule_h
//
// C++ Interface: LaserWrapper
//
// Description: 
//
//
// Author: Nikolay Jetchev,,,, <nikolay@nikolay>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//

#include <MT/threads.h>
#include <MT/array.h>
#include <MT/util.h>

struct UrgWorkspace;

struct UrgModule:public StepThread{
  UrgWorkspace *WS;

  arr scanline;
  
  UrgModule();
  ~UrgModule();
  
  void open();
  void scanLine(arr& line);
  void step(){ scanLine(scanline); }
  void close();
};

#ifdef MT_IMPLEMENTATION
#  include "UrgModule.cpp"
#endif

#endif
 
