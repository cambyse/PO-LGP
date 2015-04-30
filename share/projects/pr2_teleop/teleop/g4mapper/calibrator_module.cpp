#include "calibrator_module.h"
#include <Hardware/gamepad/gamepad.h>
#include <Ors/ors.h>
#include <Mocap/mocapdata.h>
/////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////G4 HUMAN TO ROBOT MAPPER///////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

G4HuToRoMap::G4HuToRoMap();

void G4HuToRoMap::open()
{
    mid.load("g4mapping.kvg");
    G4HuToRoMap InitThread;

    InitThread.LoopWithBeatAndWaitForClose(0.05);
}
void G4HuToRoMap::step()
{
    transformPosition();
    transformOrientatiob();
}

/// Transform the human hand position into the unit sphere.
floatA transformPosition(const floatA& thumb, const floatA& index, const floatA& center, float radius) {
  // pos
  floatA pos_mean = (thumb.sub(0, 2) + index.sub(0, 2)) / 2.f - center;
  if (length(pos_mean) >= radius)
    pos_mean /= length(pos_mean);
  else
    pos_mean /= radius;

  return {pos_mean(0), pos_mean(1), pos_mean(2)};
}

floatA transformOrientation(const floatA &pose_thumb, const floatA &pose_index) {
  ors::Quaternion quat;
  ors::Vector x_thumb, x_index;
  ors::Vector pos_thumb, pos_index;
  ors::Vector x_pr2, y_pr2, z_pr2;

  pos_thumb.set(pose_thumb(0), pose_thumb(1), pose_thumb(2));
  quat.set(pose_thumb(3), pose_thumb(4), pose_thumb(5), pose_thumb(6));
  x_thumb = quat * Vector_x;

  pos_index.set(pose_index(0), pose_index(1), pose_index(2));
  quat.set(pose_index(3), pose_index(4), pose_index(5), pose_index(6));
  x_index = quat * Vector_x;

  y_pr2 = pos_index - pos_thumb;
  y_pr2.normalize();

  x_pr2 = .5 * (x_index + x_thumb);
  x_pr2.makeNormal(y_pr2);
  x_pr2.normalize();

  z_pr2 = x_pr2 ^ y_pr2;
  z_pr2.normalize();

  double matrix[9];
  matrix[0] = x_pr2.x;
  matrix[1] = y_pr2.x;
  matrix[2] = z_pr2.x;
  matrix[3] = x_pr2.y;
  matrix[4] = y_pr2.y;
  matrix[5] = z_pr2.y;
  matrix[6] = x_pr2.z;
  matrix[7] = y_pr2.z;
  matrix[8] = z_pr2.z;
  quat.setMatrix(matrix);

  return {(float)quat.w, (float)quat.x, (float)quat.y, (float)quat.z};
}

void G4HuToRoMap::transform(const floatA& poses_raw)
{

      floatA cal_pose_rh, cal_pose_lh;

      // Positions
      auto poses_thumb_rh = mid.query(poses_raw, STRING("/human/rh/thumb"));
      auto poses_index_rh = mid.query(poses_raw, STRING("/human/rh/index"));
      cal_pose_rh.append(transformPosition(poses_thumb_rh, poses_index_rh, center, radius));

      auto poses_thumb_lh = mid.query(poses_raw, STRING("/human/lh/thumb"));
      auto poses_index_lh = mid.query(poses_raw, STRING("/human/lh/index"));
      cal_pose_lh.append(transformPosition(poses_thumb_lh, poses_index_lh, center, radius));

      // Orientations
      cal_pose_rh.append(transformOrientation(poses_thumb_rh, poses_index_rh));
      cal_pose_lh.append(transformOrientation(poses_thumb_lh, poses_index_lh));

      // Gripper
      float dummy;
      // calibrated_gripper_rh.set() = clip(
      //     length(poses_thumb_rh.sub(0, 2) - poses_index_rh.sub(0, 2)) * m_rh + q_rh,
      //     0.f, 1.f);
      dummy = length(poses_thumb_rh.sub(0, 2) - poses_index_rh.sub(0, 2)) * m_rh + q_rh;
      clip(dummy, 0.f, 1.f);
      calibrated_gripper_rh.set() = dummy;
      // calibrated_gripper_lh.set() = clip(
      //     length(poses_thumb_lh.sub(0, 2) - poses_index_lh.sub(0, 2)) * m_lh + q_lh,
      //     0.f, 1.f);
      dummy = length(poses_thumb_lh.sub(0, 2) - poses_index_lh.sub(0, 2)) * m_rh + q_rh;
      clip(dummy, 0.f, 1.f);
      calibrated_gripper_rh.set() = dummy;

      // setting access variables
      calibrated_pose_rh.set() = cal_pose_rh;
      calibrated_pose_lh.set() = cal_pose_lh;
}



//////////////////////////////////////////////////////////////////////////////
//////////////////////INIT THREAD/////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////



void initG4Mapper::LoopWithBeatAndWaitForClose(double sec)
{
      if(!metronome)
         metronome=new Metronome("threadTiccer", sec);
      else
         metronome->reset(sec);
      if(isClosed()) threadOpen();
        state.setValue(tsBEATING);


        state.waitForValueEq(tsCLOSE);

}

void initG4Mapper::open()
{
    mid.load("g4mapping.kvg");
}
void initG4Mapper::step()
{
      // starting in calibraton phase:
      //   "Y"    - save the front poses
      //   "B"    - save the right poses
      //   "A"    - save the open gripper poses
      //   "X"    - save the closed gripper poses
      //   "back" - finish calibration phase (only iff all other poses are saved)
      //
      // "start" - start calibration phase

      arr gpstate = gamepadState.get();
      CHECK(gpstate.N, "ERROR: No GamePad found");
      int button = gpstate(0);

      floatA tmpPoses = poses.get();
      // cout << tmpPoses << endl;

      if(tmpPoses.N == 0) {
        return;
      }

      // pusblish raw sensor data

      poses_rh.set() = mid.query(tmpPoses, STRINGS("/human/rh/thumb", "/human/rh/index","/human/rh/wrist"));
      poses_lh.set() = mid.query(tmpPoses, STRINGS("/human/lh/thumb", "/human/lh/index","/human/lh/wrist"));

      poses_rh.set() = mid.query(tmpPoses, {"/human/rh/thumb", "/human/rh/index","/human/lh/wrist"});
      poses_lh.set() = mid.query(tmpPoses, {"/human/lh/thumb", "/human/lh/index","/human/lh/wrist"});


      // discard lost frames
      if (length(tmpPoses.row(0)) == 0 || length(tmpPoses.row(1)) == 0 ||
          length(tmpPoses.row(3)) == 0 || length(tmpPoses.row(4)) == 0)
      return;

      // Calibration logic
if (calibration_phase)
{
   if (button & BTN_Y)
   {
           cout << "calibrating front" << endl;
           posesFront = tmpPoses;
   }
   else if (button & BTN_B)
   {
          cout << "calibrating side" << endl;
          posesSide = tmpPoses;
   }
   else if (button & BTN_A)
   {
          cout << "calibrating open gripper" << endl;
          posesOpen = tmpPoses;
   }
   else if (button & BTN_X)
   {
          cout << "calibrating closed gripper" << endl;
          posesClosed = tmpPoses;
   }
   else if (button & BTN_BACK)
   {
                 && posesSide.N != 0
                 && posesFront.N != 0
                 && posesOpen.N != 0
                 && posesClosed.N != 0
   }
   cout << "calibrating done" << endl;
   calibrate();
   calibration_phase = false;

}

void initG4Mapper::close()
{

}

void initG4Mapper::calibrate()
{
          // arm position
          // assuming posesFront and poses2 only contain 1 position
          // floatA p_front, p_side;
          // floatA p_front_xy, p_side_xy, diff, orth, center_xy;
          // float height, dist;
          // orth.resize(2);
          floatA p_side_rh, p_side_lh;

          floatA p_open, p_closed;
          float dist_open, dist_closed;

          p_side_rh = mid.query(posesSide, STRING("/human/rh/index")).subRange(0, 2);
          p_side_lh = mid.query(posesSide, STRING("/human/lh/index")).subRange(0, 2);
          center = mid.query(posesSide, STRING(" /human/torso/chest")).subRange(0, 2);
          //radius = .5f * length(p_side_rh - p_side_lh);

          // RIGHT GRIPPER
          // ===========================================================================
          p_open = mid.query(posesOpen, {"/human/rh/thumb", "/human/rh/index"}).cols(0, 3);
          p_closed = mid.query(posesClosed, {"/human/rh/thumb", "/human/rh/index"}).cols(0, 3);
          dist_open = length(p_open[0] - p_open[1]);
          dist_closed = length(p_closed[0] - p_closed[1]);
          m_rh = 1 / (dist_open - dist_closed);
          q_rh = - dist_closed * m_rh;

          // LEFT GRIPPER
          // ===========================================================================
          p_open = mid.query(posesOpen, {"/human/lh/thumb", "/human/rh/index"}).cols(0, 3);
          p_closed = mid.query(posesClosed, {"/human/lh/thumb", "/human/lh/index"}).cols(0, 3);
          dist_open = length(p_open[0] - p_open[1]);
          dist_closed = length(p_closed[0] - p_closed[1]);
          m_lh = 1 / (dist_open - dist_closed);
          q_lh = - dist_closed * m_lh;
}



