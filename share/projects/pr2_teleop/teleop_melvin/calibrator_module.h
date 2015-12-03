#pragma once
#include <Core/module.h>
#include <Motion/feedbackControl.h>
// #include <pr2/actionMachine.h>
// #include <pr2/actions.h>
#include <Mocap/mocapdata.h>

// ============================================================================


struct G4HutoRoMap:Module{
  ACCESS(bool, initmapper)

  ACCESS(floatA, g4_poses)
  ACCESS(arr, gamepadState)

  ACCESS(floatA, calibrated_pose_rh)
  ACCESS(floatA, calibrated_pose_lh)

  ACCESS(float, calibrated_gripper_rh)
  ACCESS(float, calibrated_gripper_lh)

  ACCESS(arr, drive) //desired drive command

  ACCESS(floatA, ftdata)

  /////////////////////INIT////////////////////
  bool initphase = true;
  floatA  poselhthumbmaxopen  , poselhindexmaxopen, poselhthumbminopen  , poselhindexminopen;
  float distlhmaxopen = 0.1;
  float distlhminopen = 0.04;
  floatA  poserhthumbmaxopen  , poserhindexmaxopen, poserhthumbminopen  , poserhindexminopen;
  float distrhmaxopen = 0.1;
  float distrhminopen = 0.04;

  float calarm_r_r = 0;
  float calarm_r_l = 0;

  uint SN =50;

  floatA shoulderL;
  floatA shoulderLori;
  floatA shoulderR;
  floatA shoulderRori;

  arr dp_i;
  arr rpmean;
  arr lpmean;
  arr rPmean;
  arr lPmean;
  arr rP;
  arr lP;
  arr rp;
  arr lp;
  arr sr;
  arr sl;
  double rpp;
  double lpp;
  bool shoulderinit = true;
  // void doshouldercalc();
  /////////////////////////////////////////////


  G4HutoRoMap();
  MocapID mid;
  floatA centerpos;
  floatA centerORI;
  floatA UnitPosR;
  floatA UnitPosL;

  double x,y,phistand,phi;
  void calcparameters(floatA a);
  bool decayed=true;

  void transform(const floatA& a);
  void open();
  void step();
  void close();
  /////////////////////////////////////////////
  //
  void doinitsendROS(floatA a);

  bool  btnpressed = true;
  floatA turnL;
  floatA turnR;
  floatA driveposX;
  floatA drivenegX;
  floatA driveRY;
  floatA driveLY;
  bool driveready = false;

  void doinitpresaved(int button);
};


