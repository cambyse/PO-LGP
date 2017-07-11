#include <Core/array.h>
#include <Kin/kin.h>
#include <KOMO/komo.h>
#include <Kin/taskMaps.h>
#include <Algo/gaussianProcess.h>
#include <Geo/mesh.h>
#include <Gui/opengl.h>
#include <Plot/plot.h>
#include <Roopi/roopi.h>
#include <Control/TaskControlThread.h>
#include <Kin/kinViewer.h>

void testForceControl() {
  Roopi R;
  arr jointState = FILE("preForceControlJointState");
  jointState.reshapeFlat();
  if(!R.gotToJointConfiguration(jointState, 5.0,true)) return;
  R.holdPosition();
  CtrlTask* holdLeftArm = R.createCtrlTask("holdLeftArm", new TaskMap_Default(posTMT, R.tcm()->modelWorld.get()(), "endeffL"));
  R.modifyCtrlTaskGains(holdLeftArm, 10.0,5.0);
  CtrlTask* ho = R.createCtrlTask("ho", new TaskMap_Default(posTMT, R.tcm()->modelWorld.get()(), "endeffR"));
  R.modifyCtrlTaskGains(ho, diag(ARR(30.0,30.0,0.0)), diag(ARR(5.0,5.0,0.0)));
  R.releasePosition();
  R.activateCtrlTask(holdLeftArm);
  R.activateCtrlTask(ho);
  mlr::wait(1.0);
  CtrlTask* orientation = R.createCtrlTask("orientation", new TaskMap_Default(vecTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(1.0,0.0,0.0)));
  R.modifyCtrlTaskGains(orientation, 10.0, 5.0);
  R.modifyCtrlTaskReference(orientation, ARR(0.0,0.0,-1.0));
  R.activateCtrlTask(orientation);
  CtrlTask* move1D = R.createCtrlTask("move1D", new TaskMap_Default(pos1DTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(0.0,0.0,-1.0)));
  R.modifyCtrlTaskGains(move1D, ARR(0.0), ARR(15.0));
  R.modifyCtrlTaskReference(move1D, ARR(0.0), ARR(0.1));
  R.modifyForce(move1D, ARR(-4.0), 0.005, 0.999);
  R.activateCtrlTask(move1D);
  mlr::wait(3.0);
  //R.modifyCtrlTaskGains(ho, diag(ARR(20.0,20.0,0.0)), diag(ARR(5.0,5.0,0.0)));
  arr pos = R.getTaskValue(ho);
  arr old = pos;
  pos(1) += 0.15;
  R.interpolateToReference(ho, 5.0, pos, old);
  mlr::wait(0.5);
  //pos = R.getTaskValue(ho);
  old = pos;
  pos(0) -= 0.15;
  R.interpolateToReference(ho, 5.0, pos, old);
  mlr::wait(0.5);
  //pos = R.getTaskValue(ho);
  old = pos;
  pos(1) -= 0.15;
  R.interpolateToReference(ho, 5.0, pos, old);
  mlr::wait(0.5);
  //pos = R.getTaskValue(ho);
  old = pos;
  pos(0) += 0.15;
  R.interpolateToReference(ho, 5.0, pos, old);
  mlr::wait(0.5);
  old = pos;
  pos(1) += 0.3;
  R.interpolateToReference(ho, 5.0, pos, old);
  return;
  /*pos(1) += 0.1;
  R.modifyCtrlTaskGains(ho, diag(ARR(10.0,10.0,0.0)), diag(ARR(5.0,5.0,0.0)),0.05);
  R.modifyCtrlTaskReference(ho, pos);
  R.waitForConv(ho, -1, 0.05);
  pos = R.getTaskValue(ho);
  pos(0) -= 0.1;
  R.modifyCtrlTaskReference(ho, pos);
  R.waitForConv(ho, -1, 0.05);
  pos = R.getTaskValue(ho);
  pos(1) -= 0.1;
  R.modifyCtrlTaskReference(ho, pos);
  R.waitForConv(ho, -1, 0.05);
  pos = R.getTaskValue(ho);
  pos(0) += 0.1;
  R.modifyCtrlTaskReference(ho, pos);
  R.waitForConv(ho, -1, 0.05);
  */

  /*while(true) {
    cout << R.getFTRight() << endl;
    mlr::wait(0.2);
  }*/
  mlr::wait(1.0);
  cout << R.getFTRight() << endl;
  mlr::wait(100.0);
}

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  testForceControl();
  return 0;
}


