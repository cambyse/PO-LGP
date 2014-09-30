#include <Core/array.h>
#include <MT/ors.h>

#include <motion/motion.h>

void gamepad();
void wait(double sec=0);
void homing(bool fixFingers = false);
void reach(const char* shapeName, const arr& posGoal, double maxVel=.1);
void reachAndAlign(const char* shapeName, const arr& posGoal, const arr& vecGoal, double maxVel=.1);
void setMesh(const char* shapeName, const ors::Mesh& mesh);

void waitForPerceivedObjects(uint numObjects, uint foundSteps);
void pickOrPlaceObject(MotionPrimitive::ActionPredicate action, const char* objShape, const char* belowToShape);
void closeOrOpenHand(bool closeHand);
void plannedHoming(const char* objShape, const char* belowToShape);
void graspISF();

