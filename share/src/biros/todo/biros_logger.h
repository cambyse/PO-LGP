#ifndef LOGGER_V2_H
#define LOGGER_V2_H

#include <biros/biros.h>

struct Logger {
  struct sLogger *s;
  Logger();
  ~Logger();
  //methods called by the user
  void logRevision(const Variable *i_var);
  void setReplay(const bool replay = true);
  bool getReplay() const;
  
  //methods called during write/read access from WITHIN biros
  void logReadAccess(const Variable *i_var, const Process *i_p);
  void logWriteAccess(const Variable *i_var, const Process *i_p);
  void queryReadAccess(Variable *i_var, const Process *i_p);
  void queryWriteAccess(Variable *i_var, const Process *i_p);
  void freeWriteAccess(Variable *i_var);
  void freeReadAccess(Variable *i_var);
  void setValueIfDbDriven(Variable *i_var);
  
//  TODO never called
//  void setActiveProcesses(ProcessL *list);
};

extern Logger logService; // TODO no global stuff

#endif //LOGGER_V2_H

