#include "robot.h"

CameraModule::CameraModule():Process("BumblebeeProcess"){}
void CameraModule::open(){ MT_MSG("Warning: opening dummy Camera class"); }
void CameraModule::step(){ }
void CameraModule::close(){ MT_MSG("Warning: closing dummy Camera class"); }
