#include <Roopi/roopi.h>


//===============================================================================

void testHRI() {
  Roopi R(true);

//  R.taskController().verbose(1);
  R.getTaskController().lockJointGroupControl("torso");
  R.hold(false);

  Script_graspBox(R,"obj1", LR_right);
//  auto grasp = R.graspBox("obj1", LR_right);
//  mlr::wait(1.);
//  auto grasp2 = R.graspBox("obj2", LR_left);
//  R.wait({&grasp, &grasp2});

  mlr::wait();

  Script_place(R, "obj1", "objTarget");


}


//===============================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  testHRI();

  return 0;
}
