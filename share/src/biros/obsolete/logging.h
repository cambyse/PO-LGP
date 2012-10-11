#ifndef MT_logging_h
#define MT_logging_h

#include "biros.h"
#include "biros_internal.h"


//===========================================================================
//
// the logger (and blocker)
//

struct AccessController {
  struct sAccessController *s;

  //all accesses
  ofstream* eventsFile;
  
  AccessController();
  ~AccessController();
  
  void setReplay(const bool replay = true);
  bool getReplay() const;
  
  
  //writing into a file
  void dumpEventList();
};

extern AccessController accessController; //TODO:! singleton!!...

#endif
