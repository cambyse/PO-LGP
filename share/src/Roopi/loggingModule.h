#ifndef LOGGINGMODULE_H
#define LOGGINGMODULE_H

#include <Core/array.h>
#include <Core/thread.h>
#include <Control/ctrlMsg.h>
#include <Control/taskControl.h>

/// Struct for logging data
struct SetOfDataFiles {
  mlr::String folderName;
  std::map<mlr::String, ofstream*> logMap;

  void write(const mlr::String& name, const arr& data);
  SetOfDataFiles(const char* logFolderName);
  ~SetOfDataFiles();
};

struct LoggingModule : Thread {

  ACCESSname(CtrlMsg, ctrl_ref) //< the message send to the RTController
  ACCESSname(CtrlMsg, ctrl_obs) //< the message received from the RTController
  ACCESSname(mlr::Array<CtrlTask*>, ctrlTasks)
  ACCESSname(arr, qSign)

  SetOfDataFiles logFiles;

  LoggingModule();

  void open();
  void step();
  void close() {}

};

#endif // LOGGINGMODULE_H
