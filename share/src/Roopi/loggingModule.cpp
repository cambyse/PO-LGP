#include "loggingModule.h"

void SetOfDataFiles::write(const mlr::String& name, const arr& data) {
  std::map<mlr::String, ofstream*>::iterator i = logMap.find(name);
  if(i == logMap.end()) {
    logMap.insert(std::map<mlr::String, ofstream*>::value_type(name, new ofstream(STRING("logData/" << folderName << "/" << name))));
  } else {
    *i->second << data << endl;
  }
}

SetOfDataFiles::SetOfDataFiles(const char* logFolderName) : folderName(logFolderName) {
  system(STRING("mkdir -p " << "logData/" << folderName)); //linux specific :-) TODO
}

SetOfDataFiles::~SetOfDataFiles() {
  for(auto file : logMap) {
    file.second->close();
    delete file.second;
  }
}


LoggingModule::LoggingModule()
  : Module("LoggingModule", 0.01)
  , logFiles("specifyMe") {}

void LoggingModule::open() {

}

void LoggingModule::step() {
  ctrl_ref.waitForNextRevision();
  CtrlMsg ref = ctrl_ref.get();
  logFiles.write("qRef", ref.q);
  logFiles.write("qDotRef", ref.qdot);
  logFiles.write("compliance", ARR(trace(ref.Kp)));
  logFiles.write("dynCompliance", ARR(trace(ref.Kd)));
  logFiles.write("uBias", ref.u_bias);

  logFiles.write("t", ARR(mlr::timerRead()));

  CtrlMsg obs = ctrl_obs.get();
  logFiles.write("q", obs.q);
  logFiles.write("qDot", obs.qdot);
  logFiles.write("fL", obs.fL);
  logFiles.write("fR", obs.fR);
  logFiles.write("u", obs.u_bias);

  //TODO this produces many files. Maybe separate folder for each task? And a timestamp when the task was created or active or whatever?
  mlr::Array<CtrlTask*> tasks = ctrlTasks.get();
  for(CtrlTask* c : tasks) {
    logFiles.write(STRING(c->name << "YRef"), c->y_ref);
    logFiles.write(STRING(c->name << "YDotRef"), c->v_ref);
    logFiles.write(STRING(c->name << "Y"), c->y); //TODO is that safe, or better call phi again?
    logFiles.write(STRING(c->name << "YDot"), c->v); //TODO is that safe, or better call phi again?
  }

}
