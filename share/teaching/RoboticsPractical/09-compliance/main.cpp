#include <Core/util.h>
#include "../interface/myBaxter.h"
#include <Control/TaskControllerModule.h>

//#include <Media/audio.h>

// =================================================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  {
    MyBaxter baxter;

    auto posR   = baxter.task(GRAPH("map=pos ref1=endeffR ref2=base_footprint target=[0.7 -0.2 1.3] PD=[2., .8, 1., 1.]"));
//    auto alignR = baxter.task(GRAPH("map=vecAlign ref1=endeffR ref2=base_footprint vec1=[1 0 0] vec2=[0 1 0] target=[1] PD=[1., .8, 1., 1.]"));

    baxter.waitConv({posR/*, alignR*/});

    baxter.disablePosControl();

    mlr::wait(1.);

    //-- compute torques
    arr y,J;
    posR->map.phi(y, J, baxter.getKinematicWorld());



    // Send it 0 torques for 1 second
  //  sound().addNote(12, .5, 0.);
//    TaskControllerModule& TCM = baxter.getTaskControllerModule();
//    TCM.oldfashioned = false;
//    CtrlMsg refs;
    arr q0, q, qdot, u, qdot_filtered;
//    TCM.ctrl_ref.waitForNextRevision();
    double kp = mlr::getParameter<double>("kp");
    double kd = mlr::getParameter<double>("kd");
    baxter.getState(q0, qdot, u);
    qdot_filtered = zeros(q0.N);
    for (uint i = 0; i < 1000; i++) {
#if 1
//      refs = TCM.ctrl_ref.get();
      baxter.getState(q, qdot, u);

//      qdot_filtered = .9*qdot_filtered + .1*qdot;

//      arr a = refs.Kp * (refs.q-q) + refs.Kd * qdot;
      arr a = kp * (q0-q) - kd * qdot;

      //-- translate to motor torques
      arr M, F;
      baxter.getEquationOfMotion(M, F);
//      TCM->world.equationOfMotion(M, F, false);
      arr u = M*a; // + F;
      cout <<u <<endl;

      baxter.publishTorque(u, "right_");
#else
      baxter.publishTorque(~J * ARR(0.,0.,0.), "right_");
#endif
      mlr::wait(0.005);
    }
//    TCM.oldfashioned = true;
    //sound().reset();
    baxter.enablePosControl();

    auto home = baxter.task(GRAPH(" map=qItself PD=[.5, 1., .2, 10.]"));
    baxter.modifyTarget(home, baxter.q0());
    baxter.stop({posR/*, alignR*/});
    baxter.waitConv({home});

  }

  cout <<"bye bye" <<endl;
  return 0;
}

