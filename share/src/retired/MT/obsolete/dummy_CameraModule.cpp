#include "robot.h"

CameraModule::CameraModule():Process("BumblebeeProcess"){}
void CameraModule::open(){ MLR_MSG("Warning: opening dummy Camera class"); }
void CameraModule::step(){ }
void CameraModule::close(){ MLR_MSG("Warning: closing dummy Camera class"); }
