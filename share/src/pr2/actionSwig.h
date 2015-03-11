//#include <Motion/taskMaps.h>

typedef double double3[3];
struct miniarray{ uint n; double* p; };

class ActionSwigInterface{

  ActionSwigInterface(bool useRos); //instantiates the ActionMachineSystem, runs it

  //-- data access
  std::vector<std::string> listShapes();
  std::vector<double> getQ();
  std::vector<double> getForceTorque();


  //-- methods to modify the activity state (also sequence actions)
  void activateAction(const char* symbolName); //"(reachAt A)"
  void waitForCondition(const char* condition); //"(reachAt A converged)"
  int waitForConditions(const char* condition1, const char* condition2);

  bool queryActivity(const char* symbolName);

  //-- methods to define tasks/actions
  void defineNewTaskSpaceControlAction(
      const char* symbolName,
      //task map
      const char* taskMapType,
      const char* shape_ref1, miniarray vec_ref1,
      const char* shape_ref2="baseReferenceNotWorld", miniarray vec_ref2,
      //controller parameters
      const miniarray yref, const miniarray vref,
      double refTrajectoryDurationInSeconds=-1., //trajectory reference
      double decayTime=.5, double dampingRatio=.9, double maxVel=.2, double maxAcc=10.,
      //fusion parameters
      double relativePrec=100.,
      //indicator parameters
      double convergenceIndicatorTolerance=1e-2,
      double timeoutIndicator=-1.
                              );

  defineNewTaskSpaceForceControlAction();

};
