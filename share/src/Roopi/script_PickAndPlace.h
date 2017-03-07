#pragma once

struct Roopi;

enum LeftOrRight { LR_left, LR_right };

int Script_setGripper(Roopi& R, LeftOrRight lr, double gripSize);
int Script_graspBox(Roopi& R, const char* objName, LeftOrRight rl);
int Script_place(Roopi& R, const char* objName, const char* ontoName);
int Script_placeDistDir(Roopi& R, const char* objName, const char* ontoName, double deltaX, double deltaY, int deltaTheta);
