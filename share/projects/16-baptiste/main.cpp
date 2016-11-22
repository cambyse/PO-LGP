#include <Motion/komo.h>
#include <string>
#include <map>

using namespace std;

struct HandPositionMap:TaskMap{
  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t=-1){
    ors::Body *arm = G.getBodyByName("/human/left_wrist");
    arr posArm, Jarm;
    G.kinematicsPos(posArm, Jarm, arm);

    // posArm -= ARR(.5,.5,1.3);

    y = posArm;
    J = Jarm;
  }

  virtual uint dim_phi(const ors::KinematicWorld& G){
    return 3;
  }

  virtual mlr::String shortTag(const ors::KinematicWorld& G){ return mlr::String("HandPositionMap"); }

};

//===========================================================================

struct RebaMap:TaskMap{
  map<const string, arr> coeffs_map;
  const int nb_joints = 18;
  const string joint_names[18] = {"neck_0", "neck_1", "neck_2",
                                  "spine_0", "spine_1", "spine_2",
                                  "right_knee", "left_knee",
                                  "right_shoulder_0", "right_shoulder_1",
                                  "left_shoulder_0", "left_shoulder_1",
                                  "right_elbow_0", "left_elbow_0",
                                  "right_wrist_0", "right_wrist_1",
                                  "left_wrist_0", "left_wrist_1"};

  RebaMap(){
    initCoefficientMap(coeffs_map);
  }

  void initCoefficientMap( map<const string, arr>& theMap ){
    theMap["neck_0"] = ARR(0.45594512, -0.07958066,  1.00000056, 0.1);
    theMap["neck_1"] = ARR(0.45594512, -0.07958066,  1.00000056, 0.1);
    theMap["neck_2"] = ARR(0.45594512, -0.07958066,  1.00000056, 0.1);
    theMap["spine_0"] = ARR(1.82377278,  0.        ,  1.        , 0.1);
    theMap["spine_1"] = ARR(1.82377278,  0.        ,  1.        , 0.1);
    theMap["spine_2"] = ARR(1.82377278,  0.        ,  1.        , 0.1);
    theMap["right_knee"] = ARR(1.82377278,  0.        ,  1.        , 0.1);
    theMap["left_knee"] = ARR(1.82377278,  0.        ,  1.        , 0.1);
    theMap["right_shoulder_0"] = ARR(1.21587174, -3.81974618,  4.        , 0.0033);
    theMap["right_shoulder_1"] = ARR(1.21584852,  0.        ,  1.        , 0.0033);
    theMap["left_shoulder_0"] = ARR(1.21587174, -3.81974618,  4.        , 0.0033);
    theMap["left_shoulder_1"] = ARR(1.21584852,  0.        ,  1.        , 0.0033);
    theMap["right_elbow_0"] = ARR(1.36783611, -2.3873254 ,  2.        , 0.0033);
    theMap["left_elbow_0"] = ARR(1.36783611,  2.3873254 ,  2.        , 0.0033);
    theMap["right_wrist_0"] = ARR(1.62113894,  0.        ,  1.        , 0.0033);
    theMap["right_wrist_1"] = ARR(1.62113894,  0.        ,  1.        , 0.0033);
    theMap["left_wrist_0"] = ARR(1.62113894,  0.        ,  1.        , 0.0033);
    theMap["left_wrist_1"] = ARR(1.62113894,  0.        ,  1.        , 0.0033);
  }

  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t=-1){
    y = zeros(nb_joints);
    if(&J){
      J = zeros(nb_joints, G.q.N);
    }

    for (int i=0; i<nb_joints; ++i){
      const string name = joint_names[i];
      arr coeffs = coeffs_map[name];
      ors::Joint *joint = G.getJointByName(name.c_str());
      double joint_value = G.q(joint->qIndex);
      y(i) = coeffs(3) * (coeffs(0) * joint_value * joint_value + coeffs(1) * joint_value + coeffs(2));
      if(&J) J(i, joint->qIndex) = coeffs(3) * (2. * coeffs(0) * joint_value + coeffs(1));
    }
  }


  virtual uint dim_phi(const ors::KinematicWorld& G){
    return nb_joints;
  }

  virtual mlr::String shortTag(const ors::KinematicWorld& G){ return mlr::String("RebaMap"); }

};

//===========================================================================

void moveReba(){
  
  KOMO komo;
  komo.setConfigFromFile();

  ors::Body *b = komo.world.getBodyByName("/human/base");
  b->X.addRelativeTranslation(.3,0,0);



  //  komo.setHoming(-1., -1., 1e-1);
  //  komo.setSquaredQVelocities();
  komo.setSquaredFixJointVelocities();
  komo.setSquaredFixSwitchVelocities();
  komo.setSquaredQAccelerations();
#if 0
  komo.setPosition(1., 1., "endeffL", "target", sumOfSqrTT, NoArr, 1e2);
#else
//  komo.setTask(.3, .5, new HandPositionMap(), sumOfSqrTT, ARR(.5,.5,1.3), 1e2);
//  komo.setTask(.8, 1., new HandPositionMap(), sumOfSqrTT, ARR(.8,0.,1.3), 1e2);
//  komo.setTask(.8, 1., new TaskMap_Default(posDiffTMT, komo.world, "/human/endeffR", NoVector, "target", NoVector), sumOfSqrTT, NoArr, 1e2);

//  komo.setTask(0., 1., new RebaMap(), sumOfSqrTT, NoArr, 1e2);

//  komo.setTask(.3, 1., new TaskMap_Default(gazeAtTMT, komo.world, "eyes", NoVector, "target", NoVector), sumOfSqrTT, NoArr, 1e2);


  komo.setGrasp(1., "baxterR", "screwdriverHandle", true);

  komo.setPlace(1.8, "baxterR", "screwdriverHandle", "tableC", true);
#endif
  komo.setSlowAround(1., .1, 1e3);

  komo.reset();
  komo.run();
//  komo.checkGradients();

  Graph result = komo.getReport(true);

  for(;;) komo.displayTrajectory(.1, true);
}

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  orsDrawAlpha=1.;

  moveReba();

  return 0;
}

/*
 * TODO:
 * Marc: add Baxter
 * Marc: check change joint types to quatBall (DONE; problem: reba task map)
 * Baptiste: add Markers to ShapeO
*/
