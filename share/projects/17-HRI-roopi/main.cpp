#include <Roopi/roopi.h>


//===============================================================================

void buildTower(Roopi& R, const char* objName1, const char* objName2, const char* objName3, const char* objName4, const char* ontoName){
  //obj1, obj2, obj3, and obj4 in a tower
  {
    auto graspR = R.graspBox(objName1, LR_right);
    R.wait({&graspR});
  }
  {
    auto graspL = R.graspBox(objName2, LR_left);
    R.wait({&graspL});
  }
  {
    auto placeR = R.place(objName1,ontoName);
    R.wait({&placeR});
  }
  {
    auto placeL = R.place(objName2,objName1);
    R.wait({&placeL});
  }
  {
    auto graspR2 = R.graspBox(objName3, LR_right);
    R.wait({&graspR2});
  }
  {
    auto graspL2 = R.graspBox(objName4, LR_left);
    R.wait({&graspL2});
  }
  {
    auto placeR2 = R.place(objName3,objName2);
    R.wait({&placeR2});
  }
  {
    auto placeL2 = R.place(objName4,objName3);
    R.wait({&placeL2});
  }
  mlr::wait();
}

void buildLine(Roopi& R, const char* objName1, const char* objName2, const char* objName3, const char* objName4, const char* ontoName1, const char* ontoName2, const char* ontoName3, const char* ontoName4){
  //obj1, obj2, obj3, and obj4 in a line
  auto graspR = R.graspBox(objName1, LR_right);
  R.wait({&graspR});
  auto graspL = R.graspBox(objName2, LR_left);
  R.wait({&graspL});
  auto placeR = R.place(objName1,ontoName1);
  R.wait({&placeR});
  auto placeL = R.place(objName2,ontoName2);
  R.wait({&placeL});
  auto graspR2 = R.graspBox(objName3, LR_right);
  R.wait({&graspR2});
  auto graspL2 = R.graspBox(objName4, LR_left);
  R.wait({&graspL2});
  auto placeR2 = R.place(objName3,ontoName3);
  R.wait({&placeR2});
  auto placeL2 = R.place(objName4,ontoName4);
  R.wait({&placeL2});
  mlr::wait();
}

void buildBridge(Roopi& R, const char* objName1, const char* objName2, const char* objName3, const char* ontoName1, const char* ontoName2){
  //obj1, obj2, and obj3 in a bridge
  auto graspR = R.graspBox(objName1, LR_right);
  R.wait({&graspR});
  auto graspL = R.graspBox(objName2, LR_left);
  R.wait({&graspL});
  auto placeR = R.place(objName1,ontoName1);
  R.wait({&placeR});
  auto placeL = R.place(objName2,ontoName2);
  R.wait({&placeL});
  auto graspR3 = R.graspBox(objName3, LR_right);
  R.wait({&graspR3});
  //auto placeR3 = R.place(objName3,objName1);
  auto placeR3 = R.placeDistDir(objName3,objName1,0.1,0,0);
  R.wait({&placeR3});
  mlr::wait();
}

void buildHouse(Roopi& R, const char* objName1, const char* objName2, const char* objName3, const char* objName4, const char* ontoName1){
  //obj1, obj2, obj3, and obj4 in a house
  auto graspR = R.graspBox(objName1, LR_right);
  R.wait({&graspR});
  auto placeR = R.placeDistDir(objName1,ontoName1,0,0,0);
  R.wait({&placeR});
  auto graspR1 = R.graspBox(objName2, LR_right);
  R.wait({&graspR1});
  auto graspL1 = R.graspBox(objName3, LR_left);
  R.wait({&graspL1});
  auto placeR1 = R.placeDistDir(objName2,objName1,0.1,0,0);
  R.wait({&placeR1});
  auto placeL1 = R.placeDistDir(objName3,objName1,-0.1,0,0);
  R.wait({&placeL1});
  auto graspR3 = R.graspBox(objName4, LR_right);
  R.wait({&graspR3});
  auto placeR3 = R.placeDistDir(objName4,objName2,-0.1,0,0);
  R.wait({&placeR3});
  mlr::wait();
}

void buildBox(Roopi& R, const char* objName1, const char* objName2, const char* objName3, const char* objName4,
              const char* objName5, const char* objName6, const char* objName7, const char* objName8,
              const char* objName9, const char* objName10, const char* ontoName1, const char* ontoName2, const char* ontoName3){
  //obj1 to obj14 in a box (1 to 8 are rectangles, 9 and 10 are cubes
  auto graspR = R.graspBox(objName1, LR_right);
  R.wait({&graspR});
  auto graspL = R.graspBox(objName2, LR_left);
  R.wait({&graspL});
  auto placeR = R.placeDistDir(objName1,ontoName1,0,0,0);
  R.wait({&placeR});
  auto placeL = R.placeDistDir(objName2,ontoName2,0,0,0);
  R.wait({&placeL});
  auto graspR1 = R.graspBox(objName3, LR_right);
  R.wait({&graspR1});
  auto graspL1 = R.graspBox(objName4, LR_left);
  R.wait({&graspL1});
  auto placeR1 = R.placeDistDir(objName3,ontoName3,0,0,0);
  R.wait({&placeR1});
  auto placeL1 = R.placeDistDir(objName4,objName1,0,0,1);
  R.wait({&placeL1});
  auto graspR2 = R.graspBox(objName9, LR_right);
  R.wait({&graspR2});
  auto graspL2 = R.graspBox(objName10, LR_left);
  R.wait({&graspL2});
  auto placeR2 = R.placeDistDir(objName9,objName2,0.1,0,0);
  R.wait({&placeR2});
  auto placeL2 = R.placeDistDir(objName10,objName2,-0.1,0,0);
  R.wait({&placeL2});
  auto graspR3 = R.graspBox(objName5, LR_right);
  R.wait({&graspR3});
  auto graspL3 = R.graspBox(objName6, LR_left);
  R.wait({&graspL3});
  auto placeR3 = R.placeDistDir(objName5,objName3,0,0,1);
  R.wait({&placeR3});
  auto placeL3 = R.placeDistDir(objName6,objName4,0,0,1);
  R.wait({&placeL3});
  auto graspR4 = R.graspBox(objName7, LR_right);
  R.wait({&graspR4});
  auto graspL4 = R.graspBox(objName8, LR_left);
  R.wait({&graspL4});
  auto placeR4 = R.placeDistDir(objName7,objName9,-0.1,0,0);
  R.wait({&placeR4});
  auto placeL4 = R.placeDistDir(objName8,objName5,0,0,1);
  R.wait({&placeL4});
  mlr::wait();
}

void testHRI() {
  Roopi R(true);

//  R.getTaskController().verbose(1);
  //R.getTaskController().lockJointGroupControl("torso");
  //R.hold(false);
  R.collisions(true);

  buildTower(R,"cube1","cube2","cube3","cube4","objTarget");
  //buildLine(R,"cube1","cube2","cube3","cube4","objTarget","objTarget2","objTarget3","objTarget4");
//  buildBridge(R,"cube1","cube2","rect1","objTarget","objTarget3");
  //buildHouse(R,"rect1","cube1","cube2","rect2","objTarget");
  //buildBox(R,"rect1","rect2","rect3","rect4","rect5","rect6","rect7","rect8","cube1","cube2","objTarget","objTarget5","objTarget6");


  //auto graspR = R.graspBox("rect1", LR_right);
  // R.wait({&graspR});
  //auto graspL = R.graspBox("rect2", LR_left);
  //R.wait({&graspL});
  //auto placeR = R.place("rect1","objTarget6");
  //R.wait({&placeR});
  //auto placeL = R.place("rect2","objTarget7");
  //R.wait({&placeL});
  //mlr::wait();

  //Script_graspBox(R,"rect1", LR_right);
  //Script_place(R,"rect1","objTarget");
  //Script_placeDistDir(R,"rect1","objTarget",0.2,0,1);
  //mlr::wait();
  //Script_graspBox(R,"obj1", LR_right);
  //  auto grasp = R.graspBox("obj1", LR_right);
  //  mlr::wait(1.);
  //  auto grasp2 = R.graspBox("obj2", LR_left);
  //graspR = R.place("obj1",LR_right);
  //graspL = R.place("obj2", LR_left);
  //  auto grasp = R.graspBox("obj1", LR_right);
  //  mlr::wait(1.);
  //  auto grasp2 = R.graspBox("obj2", LR_left);
  //R.wait({&graspR, &graspL});

  //mlr::wait();
  //Script_graspBox(R,"obj2", LR_left);

  //mlr::wait();

}


//===============================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  testHRI();

  return 0;
}
