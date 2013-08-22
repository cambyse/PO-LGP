#ifndef NJ_UrgInterface_h
#define NJ_UrgInterface_h
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

#include <System/biros.h>
#include <Core/array.h>
#include <Core/util.h>

struct UrgWorkspace;

struct UrgInterface:public Process{
  UrgWorkspace *s;

  arr scanline;
  
  UrgInterface();
  ~UrgInterface();
  
  void open();
  void scanLine(arr& line);
  void step(){ scanLine(scanline); }
  void close();
};

#endif
 
