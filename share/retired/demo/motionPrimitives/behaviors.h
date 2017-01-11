#include <Core/array.h>
#include <MT/kin.h>

#include <motion/motion.h>

void gamepad();
void wait(double sec=0);
void homing(bool fixFingers = false);
void reach(const char* shapeName, const arr& posGoal, double maxVel=.1);
void reachAndAlign(const char* shapeName, const arr& posGoal, const arr& vecGoal, double maxVel=.1);
void setMesh(const char* shapeName, const mlr::Mesh& mesh);

void waitForPerceivedObjects(uint numObjects, uint foundSteps);
void homing2(const char* objShape, const char* belowToShape);
double reach2(const char* objShape, const char* belowToShape);
void pickOrPlaceObject(Action::ActionPredicate action, const char* objShape, const char* belowToShape);
void closeOrOpenHand(bool closeHand);
void plannedHoming(const char* objShape, const char* belowToShape);
void graspISF();

