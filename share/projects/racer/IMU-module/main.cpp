#include <Core/util.h>
#include <System/engine.h>

#include "modules.h"


int main() {

  struct MySystem:System{
    ACCESS(arr, imuData)
    MySystem(){
      addModule<IMU_Poller>("IMU_Poller", ModuleThread::loopFull);
      addModule<KalmanFilter>("KalmanFilter", ModuleThread::listenFirst);
      addModule<RacerDisplay>("RacerDisplay", ModuleThread::loopWithBeat, 0.1);
      connect();
    }
  } S;

  cout <<S <<endl;

  //    engine().enableAccessLog();
  engine().open(S);


  for(;;){
    S.imuData.var->waitForNextWriteAccess();
    if(S.imuData.get()()(0)>10.) break;
  }

  //    engine().shutdown.waitForSignal();

  engine().close(S);

  cout <<"bye bye" <<endl;


  return 0;
}


