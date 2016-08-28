#ifndef LOGGINGMODULE_H
#define LOGGINGMODULE_H

#include <Core/array.h>
#include <Core/module.h>
#include <Control/ctrlMsg.h>
#include <Control/taskController.h>

/// Struct for logging data
struct SetOfDataFiles {
  mlr::String folderName;
  std::map<mlr::String, ofstream*> logMap;

  void write(const mlr::String& name, const arr& data);
  SetOfDataFiles(const char* logFolderName);
  ~SetOfDataFiles();
};

struct LoggingModule : Module {

  ACCESSname(CtrlMsg, ctrl_ref) //< the message send to the RTController
  ACCESSname(CtrlMsg, ctrl_obs) //< the message received from the RTController
  ACCESSname(mlr::Array<CtrlTask*>, ctrlTasks)

  SetOfDataFiles logFiles;

  LoggingModule();

  void open();
  void step();
  void close() {}

};

#endif // LOGGINGMODULE_H
