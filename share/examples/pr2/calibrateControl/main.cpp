#include <Core/module.h>
#include <Control/TaskControllerModule.h>
#include <Roopi/roopi.h>
#include <Roopi/loggingModule.h>

#include "poseGenerator.h"


void sampleData() {
  Roopi R;
  SetOfDataFiles logging("gravityCompensation_1");
  rnd.clockSeed();
  PoseGenerator poser(R.tcm()->modelWorld.get()());

  for(uint i = 0; i < 1000; i++) {
    //try to find both a suitable random pose and a trajectory that transfers from the current configuration to the random pose
    while(true) {
      arr qPose = poser.getRandomPose();
      if(R.gotToJointConfiguration(qPose,5.0)) break;
    }
    R.holdPosition();

    //wait a few seconds to ensure that the controller has converged
    mlr::wait(4.0);

    //mean over 1 second with 10 samples
    arr q, qSign, fL, fR, u;
    for(uint j = 0; j < 10; j++) {
      q.append(~R.getJointState());
      qSign.append(~R.getJointSign());
      fL.append(~R.getFTLeft());
      fR.append(~R.getFTRight());
      u.append(~R.getTorques());
      mlr::wait(0.1);
    }
    //save everything
    logging.write("q", sum(q,0)/(double)q.d0);
    logging.write("qSign", sum(qSign,0)/(double)qSign.d0); //TODO does it make sense to mean over them? NO!!
    logging.write("u", sum(u,0)/(double)u.d0);
    logging.write("fL", sum(fL,0)/(double)fL.d0);
    logging.write("fR", sum(fR,0)/(double)fR.d0);
  }
}


// =================================================================================================
int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  sampleData();
  return 0;
}
