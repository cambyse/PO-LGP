#pragma once

#include <Geo/geo.h>

struct Roopi;

enum LeftOrRight { LR_left, LR_right };
enum CubeSide { CS_x, CS_y, CS_z };

int Script_setGripper(Roopi& R, LeftOrRight lr, double gripSize);
int Script_graspBox(Roopi& R, const char* objName, LeftOrRight rl);
int Script_place(Roopi& R, const char* objName, const char* ontoName, const mlr::Quaternion& rot=Quaternion_x);
int Script_placeDistDir(Roopi& R, const char* objName, const char* ontoName, double deltaX, double deltaY, int deltaTheta);
int Script_armsNeutral(Roopi& R, LeftOrRight rl);
int Script_workspaceReady(Roopi& R, const char* objName);



// Marc's scripts: KOMO (planned motion) scripts

int Script_komoGraspBox(Roopi& R, const char* objName, LeftOrRight rl);


//helpers
double getGripSize(Roopi& R, const char* objName, CubeSide cs=CS_y);
