#ifndef MT_logging_h
#define MT_logging_h

#include "biros.h"

//===========================================================================
//
// information about a single access event
//

struct AccessEvent{
  const Variable *var;
  const Process *proc;
  enum AccessType{ read, write } type;
  uint revision;
  uint procStep;
  AccessEvent(const Variable *v, const Process *p, AccessType _type, uint _revision, uint _procStep):
    var(v), proc(p), type(_type), revision(_revision), procStep(_procStep){}
};

typedef MT::Array<AccessEvent*> AccessEventL;

//===========================================================================
//
// the logger (and blocker)
//

struct AccessController {
  struct sAccessController *s;

  //all accesses
  AccessEventL events;
  ofstream* eventsFile;
  
  //blocking accesses
  AccessEventL blockedAccesses;
  
  AccessController();
  ~AccessController();
  
  //methods called by the user
  void blockAllAccesses();
  void unblockAllAccesses();
  void stepToNextAccess();
  void stepToNextWriteAccess();
  void setReplay(const bool replay = true);
  bool getReplay() const;
  
  //methods called during write/read access from WITHIN biros
  void queryReadAccess(Variable *v, const Process *p);
  void queryWriteAccess(Variable *v, const Process *p);
  void logReadAccess(const Variable *v, const Process *p);
  void logReadDeAccess(const Variable *v, const Process *p);
  void logWriteAccess(const Variable *v, const Process *p);
  void logWriteDeAccess(const Variable *v, const Process *p);
  
  //writing into a file
  void dumpEventList();
};

extern AccessController accessController;

#endif