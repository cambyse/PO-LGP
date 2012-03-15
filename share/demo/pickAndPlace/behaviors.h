#include <MT/array.h>
#include <MT/ors.h>

void joystick();
void wait(double sec=0);
void homing();
void reach(const char* shapeName, const arr& posGoal, double maxVel=.1);
void reachAndAlign(const char* shapeName, const arr& posGoal, const arr& vecGoal, double maxVel=.1);
void setMesh(const char* shapeName, const ors::Mesh& mesh);

void waitForPerceivedObjects(uint numObjects, uint foundSteps);
void pickObject(char* objShape);
void placeObject(char* objShape, char* belowFromShape, const char* belowToShape);
void plannedHoming(const char* objShape, const char* belowToShape);
void graspISF();

