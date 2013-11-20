#include "module_FloatA_Recorder.h"

REGISTER_MODULE(FloatA_Recorder)

FloatA_Recorder::FloatA_Recorder():Module("FloatA_Recorder"){
}

void FloatA_Recorder::open(){
  MT::String nowStr;
  MT::getNowString(nowStr);
  file.open(STRING("z." << nowStr << ".something.dat"));
}


void FloatA_Recorder::close(){
  file.close();
}

void FloatA_Recorder::step(){
  uint rev = x.readAccess();
  floatA X = x();
  double time = x.var->revisionTime();
  x.deAccess();
  MT::String tag;
  tag.resize(30, false);
  sprintf(tag.p, "%6i %13.6f", rev, time);

  file <<tag <<' ' <<X <<endl;
}
