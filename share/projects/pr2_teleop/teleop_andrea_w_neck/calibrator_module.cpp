#include "calibrator_module.h"
#include <Hardware/gamepad/gamepad.h>
#include <Ors/ors.h>
#include <Mocap/mocapdata.h>

// ############################################################################
// Calibrator
Calibrator::Calibrator()
    : calibration_phase(true)
    // , center(3)
{
  mid.load("g4mapping.kvg");
}

void Calibrator::step() {
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

  arrf tmpPoses = poses.get();

  if(tmpPoses.N == 0) {
    return;
  }

  // changing the coordinate system
  fixCoordinates(tmpPoses);

  // pusblish raw sensor data
  poses_rh.set() = mid.query(tmpPoses, {"/human/rh/thumb", "/human/rh/index"});
  poses_lh.set() = mid.query(tmpPoses, {"/human/lh/thumb", "/human/lh/index"});

  // discard lost frames
  if (length(tmpPoses.row(0)) == 0 || length(tmpPoses.row(1)) == 0 ||
      length(tmpPoses.row(3)) == 0 || length(tmpPoses.row(4)) == 0)
    return;

  // Calibration logic
  if (calibration_phase) {
    // if (button & BTN_Y) {
    //   cout << "calibrating front" << endl;
    //   posesFront = tmpPoses;
    // }
    if (button & BTN_B) {
      cout << "calibrating side" << endl;
      posesSide = tmpPoses;
    }
    else if (button & BTN_A) {
      cout << "calibrating open gripper" << endl;
      posesOpen = tmpPoses;
    }
    else if (button & BTN_X) {
      cout << "calibrating closed gripper" << endl;
      posesClosed = tmpPoses;
    }
    else if (button & BTN_BACK
             && posesSide.N != 0
             // && posesFront.N != 0
             && posesOpen.N != 0
             && posesClosed.N != 0)
    {
      cout << "calibrating done" << endl;
      calibrate();
      calibration_phase = false;
    }
  }
  else {
    transform(tmpPoses);
    // cout << "raw" << tmpPoses << endl;

    if(button & BTN_START) {
      cout << "calibrating start" << endl;
      calibration_phase = true;
      posesSide.resize(0);
      // posesFront.resize(0);
      posesOpen.resize(0);
      posesClosed.resize(0);
    }
  }
}

void compute_neck_params(const arrf &pose_neck) {
  transf_neck.setZero();
  transf_neck.pos.set(pose_neck[0], pose_neck[1], pose_neck[2]);
  transf_neck.rot.set(pose_neck[3], pose_neck[4], pose_neck[5], pose_neck[6]);

  ors::Vector v_x, v_y, v_z;

  v_x = transf_neck.rot * Vector_x;
  v_x.makeNormal(Vector_z);
  v_x.normalize();

  v_z = transf_neck.rot * Vector_z;
  v_z.makeNormal(Vector_z);
  v_z.normalize();

  v_y = v_z ^ v_x;
  v_y.normalize();

  double matrix[9];
  matrix[0] = v_x.x;
  matrix[1] = v_y.x;
  matrix[2] = v_z.x;
  matrix[3] = v_x.y;
  matrix[4] = v_y.y;
  matrix[5] = v_z.y;
  matrix[6] = v_x.z;
  matrix[7] = v_y.z;
  matrix[8] = v_z.z;

  transf_neck_planar.setZero();
  transf_neck_planar.pos = transf_neck.pos;
  transf_neck_planar.rot.setMatrix(matrix);

  quat_neck = transf_neck_planar.rot / transf_neck.rot;
}

void Calibrator::calibrate() {
  // arm position
  // assuming posesFront and poses2 only contain 1 position
  // arrf p_front, p_side;
  // arrf p_front_xy, p_side_xy, diff, orth, center_xy;
  // float height, dist;
  // orth.resize(2);
  arrf pose_side_rh, pose_side_lh, pose_neck;
  ors::Transformation transf_side_rh, transf_side_lh, transf_neck;
  ors::Transformation transf_neck_side_rh, transf_neck_side_lh;

  arrf p_open, p_closed;
  float dist_open, dist_closed;

  pose_side_rh = mid.query(posesSide, STRING("/human/rh/index"));
  pose_side_lh = mid.query(posesSide, STRING("/human/lh/index"));
  pose_neck = mid.query(posesSide, STRING("/human/neck/neck"));

  transf_side_rh.setZero();
  transf_side_rh.pos.set(pose_side_rh[0], pose_side_rh[1], pose_side_rh[2]);
  transf_side_rh.rot.set(pose_side_rh[3], pose_side_rh[4], pose_side_rh[5], pose_side_rh[6]);

  transf_side_lh.setZero();
  transf_side_lh.pos.set(pose_side_lh[0], pose_side_lh[1], pose_side_lh[2]);
  transf_side_lh.rot.set(pose_side_lh[3], pose_side_lh[4], pose_side_lh[5], pose_side_lh[6]);

  compute_neck_params(pose_neck);

  transf_neck_side_rh = transf_side_rh / transf_neck;
  transf_neck_side_lh = transf_side_lh / transf_neck;

  radius = .5f * ( transf_neck_side_rh.pos.length() + transf_neck_side_lh.pos.length() );

  // center = .5f * (p_side_rh + p_side_lh);
  // radius = .5f * length(p_side_rh - p_side_lh);

// Older calibration code
//   // RIGHT HAND
//   // ===========================================================================
//   p_front = mid.query(posesFront, STRING("rh:index")).subRange(0, 2);
//   p_side = mid.query(posesSide, STRING("rh:index")).subRange(0, 2);

//   // assuming the first three coordinates are position
//   diff = p_side - p_front;
//   height = .5 * (p_front(2) + p_side(2));

//   p_front_xy = p_front.subRange(0, 1);
//   p_side_xy = p_side.subRange(0, 1);

//   // assuming the first 2 coordinates are x-y
//   dist = length(diff);
//   radius_rh = sqrt(.5) * dist;

//   orth(0) = -diff(1);
//   orth(1) = diff(0);

//   center_xy = .5f * (p_front_xy + p_side_xy - orth * dist);
//   center_rh.subRange(0, 1) = center_xy;
//   center_rh(2) = height;

//   // LEFT HAND
//   // ===========================================================================
//   p_front = mid.query(posesFront, STRING("lh:index")).subRange(0, 2);
//   p_side = mid.query(posesSide, STRING("lh:index")).subRange(0, 2);

//   // assuming the first three coordinates are position
//   diff = p_side - p_front;
//   height = .5 * (p_front(2) + p_side(2));

//   p_front_xy = p_front.subRange(0, 1);
//   p_side_xy = p_side.subRange(0, 1);

//   // assuming the first 2 coordinates are x-y
//   dist = length(diff);
//   radius_lh = sqrt(.5) * dist;

//   orth(0) = diff(1);
//   orth(1) = -diff(0);

//   center_xy = .5f * (p_front_xy + p_side_xy - orth * dist);
//   center_lh.subRange(0, 1) = center_xy;
//   center_lh(2) = height;

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
  p_open = mid.query(posesOpen, {"/human/lh/thumb", "/human/lh/index"}).cols(0, 3);
  p_closed = mid.query(posesClosed, {"/human/lh/thumb", "/human/lh/index"}).cols(0, 3);
  dist_open = length(p_open[0] - p_open[1]);
  dist_closed = length(p_closed[0] - p_closed[1]);
  m_lh = 1 / (dist_open - dist_closed);
  q_lh = - dist_closed * m_lh;

  // cout << "RIGHT"
  //      << " radius" << radius_rh
  //      << " center" << center_rh
  //      << " q" << q_rh
  //      << " m" << m_rh
  //      << "\nLEFT"
  //      << " radius" << radius_lh
  //      << " center" << center_lh
  //      << " q" << q_lh
  //      << " m" << m_lh
  //      << endl;
}

ors::Quaternion makeHandRotation(const arrf &thumb, const arrf &index) {
  ors::Quaternion quat;
  ors::Vector x_thumb, x_index;
  ors::Vector pos_thumb, pos_index;
  ors::Vector x_pr2, y_pr2, z_pr2;

  pos_thumb.set(thumb(0), thumb(1), thumb(2));
  quat.set(thumb(3), thumb(4), thumb(5), thumb(6));
  x_thumb = quat * Vector_x;

  pos_index.set(index(0), index(1), index(2));
  quat.set(index(3), index(4), index(5), index(6));
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

  return quat;
}

arrf makeHandPose(const arrf &thumb, const arrf& index, const ors::Transformation transf_neck, float radius) {
  ors::Transformation transf_hand, transf_neck_hand;
  transf_hand.setZero();

  // 1) define transf_hand (position and orientation independently)
  transf_hand.pos = ors::Vector(.5f * (thumb.sub(0, 2) + index.sub(0, 2)));
  transf_hand.rot = makeHandRotation(thumb, index);

  // 2) remove neck element
  transf_neck_hand = transf_hand / transf_neck;
  
  // 3) limit radius
  if(transf_neck_hand.pos.length() >= radius)
    transf_neck_hand.pos.setLength(1.);
  else
    transf_neck_hand.pos /= radius;

  // 4) remove one dimension (maybe not?)

  return {
    (float) transf_neck_hand.pos.x,
    (float) transf_neck_hand.pos.y,
    (float) transf_neck_hand.pos.z,
    (float) transf_neck_hand.rot.w,
    (float) transf_neck_hand.rot.x,
    (float) transf_neck_hand.rot.y,
    (float) transf_neck_hand.rot.z
  };
}

/// Transform the human hand position into the unit sphere.
// arrf transformPosition(const arrf& thumb, const arrf& index, const arrf& center, float radius) {
//   // pos
//   arrf pos_mean = (thumb.sub(0, 2) + index.sub(0, 2)) / 2.f - center;
//   if(length(pos_mean) >= radius)
//     pos_mean /= length(pos_mean);
//   else
//     pos_mean /= radius;

//   return {pos_mean(0), pos_mean(1), pos_mean(2)};
// }

// arrf transformOrientation(const arrf &pose_thumb, const arrf &pose_index) {
//   ors::Quaternion quat;
//   ors::Vector x_thumb, x_index;
//   ors::Vector pos_thumb, pos_index;
//   ors::Vector x_pr2, y_pr2, z_pr2;

//   pos_thumb.set(pose_thumb(0), pose_thumb(1), pose_thumb(2));
//   quat.set(pose_thumb(3), pose_thumb(4), pose_thumb(5), pose_thumb(6));
//   x_thumb = quat * Vector_x;

//   pos_index.set(pose_index(0), pose_index(1), pose_index(2));
//   quat.set(pose_index(3), pose_index(4), pose_index(5), pose_index(6));
//   x_index = quat * Vector_x;

//   y_pr2 = pos_index - pos_thumb;
//   y_pr2.normalize();

//   x_pr2 = .5 * (x_index + x_thumb);
//   x_pr2.makeNormal(y_pr2);
//   x_pr2.normalize();

//   z_pr2 = x_pr2 ^ y_pr2;
//   z_pr2.normalize();
  
//   double matrix[9];
//   matrix[0] = x_pr2.x;
//   matrix[1] = y_pr2.x;
//   matrix[2] = z_pr2.x;
//   matrix[3] = x_pr2.y;
//   matrix[4] = y_pr2.y;
//   matrix[5] = z_pr2.y;
//   matrix[6] = x_pr2.z;
//   matrix[7] = y_pr2.z;
//   matrix[8] = z_pr2.z;
//   quat.setMatrix(matrix);

//   return {(float)quat.w, (float)quat.x, (float)quat.y, (float)quat.z};
// }

void Calibrator::transform(const arrf& poses_raw) {
  arrf cal_pose_rh, cal_pose_lh;

  // Positions
  auto poses_thumb_rh = mid.query(poses_raw, STRING("/human/rh/thumb"));
  auto poses_index_rh = mid.query(poses_raw, STRING("/human/rh/index"));
  // cal_pose_rh.append(transformPosition(poses_thumb_rh, poses_index_rh));
  cal_pose_rh = makeHandPose(poses_thumb_rh, poses_index_rh, transf_neck, radius);

  auto poses_thumb_lh = mid.query(poses_raw, STRING("/human/lh/thumb"));
  auto poses_index_lh = mid.query(poses_raw, STRING("/human/lh/index"));
  // cal_pose_lh.append(transformPosition(poses_thumb_lh, poses_index_lh));
  cal_pose_lh = makeHandPose(poses_thumb_lh, poses_index_lh, transf_neck, radius);

  // Orientations
  // cal_pose_rh.append(transformOrientation(poses_thumb_rh, poses_index_rh));
  // cal_pose_lh.append(transformOrientation(poses_thumb_lh, poses_index_lh));

  // Gripper
  float dummy;

  dummy = length(poses_thumb_rh.sub(0, 2) - poses_index_rh.sub(0, 2)) * m_rh + q_rh;
  clip(dummy, 0.f, 1.f);
  calibrated_gripper_rh.set() = dummy;

  dummy = length(poses_thumb_lh.sub(0, 2) - poses_index_lh.sub(0, 2)) * m_lh + q_lh;
  clip(dummy, 0.f, 1.f);
  calibrated_gripper_lh.set() = dummy;

  // setting access variables
  calibrated_pose_rh.set() = cal_pose_rh;
  calibrated_pose_lh.set() = cal_pose_lh;
}

void Calibrator::fixCoordinates(arrf &poses) {
  ors::Transformation T, Tfix;
  Tfix.setZero();
  Tfix.addRelativeRotationDeg(-90, 0, 0, 1);

  for(uint i = 0; i < poses.d0; i++) {
    T.setZero();
    T.pos.set(poses(i, 0), poses(i, 1), poses(i, 2));
    T.rot.set(poses(i, 3), poses(i, 4), poses(i, 5), poses(i, 6));
    T = Tfix * T;

    poses(i, 0) = T.pos.x;
    poses(i, 1) = T.pos.y;
    poses(i, 2) = T.pos.z;
    poses(i, 3) = T.rot.w;
    poses(i, 4) = T.rot.x;
    poses(i, 5) = T.rot.y;
    poses(i, 6) = T.rot.z;
  }
}
