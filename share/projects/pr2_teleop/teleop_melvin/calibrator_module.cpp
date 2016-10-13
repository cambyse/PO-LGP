#include "calibrator_module.h"
#include <Hardware/gamepad/gamepad.h>
#include <Ors/ors.h>
#include <Mocap/mocapdata.h>

//////////////////////////////////////////////////////////////////////////////
//////////////////////INIT THREAD/////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////Andrea CALI method////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
/*void G4HutoRoMap::doinitandrea(tmpPoses,button)
{
  if (calibration_phase) {
    if (button & BTN_B)
    {
      cout << "calibrating side" << endl;
      posesSideR = mid.query(tmpPoses, STRING("/human/rh/index")).refRange(0, 2)-centerpos;
      posesSideL = mid.query(tmpPoses, STRING("/human/lh/index")).refRange(0, 2)-centerpos;

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
    else if (button & BTN_Y)
    {
      cout <<"calibrating front gripper"<<endl;
      posesFrontR = mid.query(tmpPoses, STRING("/human/rh/index")).refRange(0, 2)-centerpos;
      posesFrontL = mid.query(tmpPoses, STRING("/human/lh/index")).refRange(0, 2)-centerpos;
    }
    else if (button & BTN_BACK
             && posesSideR.N != 0
             && posesSideL.N != 0
             && posesFrontR.N != 0
             && posesFrontL.N != 0
             && posesOpen.N != 0
             && posesClosed.N != 0)
    {
      cout << "calibrating done" << endl;
      caliandrea();
      calibration_phase = false;
    }
    else
    {
        cout<<"calibration is not done!"<<endl;
    }
  }
  else {
    // cout << "raw" << tmpPoses << endl;

    if(button & BTN_START) {
      cout << "calibrating start" << endl;
      calibration_phase = true;
      posesSideR.resize(0);
      posesSideL.resize(0);
      posesFrontR.resize(0);
      posesFrontL.resize(0);
      posesOpen.resize(0);
      posesClosed.resize(0);
    }
  }
}
void G4HutoRoMap::caliandrea();
{

  arrf p_side_rh, p_side_lh;
  arrf p_open, p_closed;
  float dist_open, dist_closed;



  shoulderR =
  shoulderL =
  radiusR_andrea = .5f * length(p_side_rh - p_side_lh);
  radiusL_andrea = .5f * length(p_side_rh - p_side_lh);

  // RIGHT GRIPPER
  // ===========================================================================
  p_open = mid.query(posesOpen, {"/human/rh/thumb", "/human/rh/index"}).cols(0, 3);
  p_closed = mid.query(posesClosed, {"/human/rh/thumb", "/human/rh/index"}).cols(0, 3);
  dist_open = length(p_open[0] - p_open[1]);
  dist_closed = length(p_closed[0] - p_closed[1]);
  m_rh_andrea  = 1 / (dist_open - dist_closed);
  q_rh_andrea  = - dist_closed * m_rh;

  // LEFT GRIPPER
  // ===========================================================================
  p_open = mid.query(posesOpen, {"/human/lh/thumb", "/human/lh/index"}).cols(0, 3);
  p_closed = mid.query(posesClosed, {"/human/lh/thumb", "/human/lh/index"}).cols(0, 3);
  dist_open = length(p_open[0] - p_open[1]);
  dist_closed = length(p_closed[0] - p_closed[1]);
  m_lh_andrea  = 1 / (dist_open - dist_closed);
  q_lh_andrea  = - dist_closed * m_lh;

}
void G4HutoRoMap::transform_andrea()
{

}
*/
/////////////////////////////////////////////////////////////////////////////////////////
void G4HutoRoMap::doinitpresaved(int button)
{
}

/////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////G4 HUMAN TO ROBOT MAPPER///////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
floatA CORDtranstoRo(const floatA& input,floatA centera)
{
  //-90° from G4 to roboto

  ors::Vector PosToRobot;
  ors::Quaternion OrToRobot;
  ors::Quaternion trans;
  trans.setDeg(-90,0.,0.,1.);

  ors::Vector center;
  center.set(centera(0),centera(1),centera(2));
  PosToRobot.set(input(0),input(1),input(2));
  OrToRobot.set(input(3) , input(4), input(5), input(6));

  PosToRobot = trans* PosToRobot - center;
  OrToRobot = trans * OrToRobot;


  return {(float)PosToRobot.x,(float)PosToRobot.y,(float)PosToRobot.z,(float)OrToRobot.w,
        (float)OrToRobot.x,(float)OrToRobot.y,(float)OrToRobot.z};
}

floatA transcenter(const floatA& tempData,const floatA& ref )
{
  floatA TtempData(tempData);
  ors::Transformation transform;

  ors::Vector refVector;
  ors::Quaternion refOrien;
  refVector.set(ref(0),ref(1),ref(2));
  refOrien.set(ref(3),ref(4),ref(5),ref(6));
  ors::Quaternion flip;
  flip.setDeg(180,0.,0.,1.);

  refOrien.alignWith(Vector_z);
  transform.addRelativeTranslation(refVector);
  //transform.addRelativeRotation(refOrien);
  transform.setInverse(transform);

  for(uint i =0 ;i<tempData.d0;i++)
  {
    ors::Vector tempV;
    tempV.set(tempData[i](0),tempData[i](1),tempData[i](2));
        ors::Quaternion tempQ;
    tempQ.set(tempData[i](3),tempData[i](4),tempData[i](5),tempData[i](6));
        tempV = transform*tempV;
    tempV = refOrien/tempV;
    tempV = flip*tempV;
    tempQ = refOrien/tempQ;
    tempQ = flip* tempQ;
    TtempData[i]={(float)tempV.x,(float)tempV.y,(float)tempV.z,
                  (float)tempQ.w,(float)tempQ.x,(float)tempQ.y,(float)tempQ.z};
  }
  return TtempData;
}
floatA transfshoulder(const floatA& shoulder,const floatA& ref,const floatA& shoulderOR)
{
  floatA TtempData(shoulder);
  ors::Transformation transform;

  ors::Quaternion refOrien;
  refOrien.set(ref(0),ref(1),ref(2),ref(3));

  transform.addRelativeRotation(refOrien);
  refOrien.set(shoulderOR(3),shoulderOR(4),shoulderOR(5),shoulderOR(6));

  transform.addRelativeRotation(refOrien.invert());
  //transform.setInverse(transform);

  ors::Vector shoulderV;
  shoulderV.set(shoulder[0](0),shoulder[0](1),shoulder[0](2));
  shoulderV = transform *shoulderV;


  return {(float)shoulderV.x, (float)shoulderV.y, (float)shoulderV.z};
}
void G4HutoRoMap::calcparameters(floatA tempData)
{

  floatA poses_thumb_rh = mid.query(tempData, STRING("/human/rh/thumb")).refRange(0,2);
  floatA poses_index_rh = mid.query(tempData, STRING("/human/rh/index")).refRange(0,2);

  floatA TI_vec = poses_thumb_rh-poses_index_rh;
  // cout<<TI_vec<<endl;
  floatA quats = mid.query(tempData, STRING("/human/rh/thumb")).refRange(3,6);

  TI_vec = TI_vec/length(TI_vec);
  // cout<<TI_vec<<endl;
  ors::Quaternion orsquats;
  orsquats.set(double(quats(0)),double(quats(1)),double(quats(2)),double(quats(3)));
  x = (double)TI_vec(0);
  y = (double)TI_vec(1);
  orsquats.alignWith(Vector_z);
  phi = orsquats.getRad();


  if(x <= 0.15 && x>=-0.15 && y <= 0.15 && y>=-0.15 &&  decayed)
  {
    phistand = orsquats.getRad();
    decayed = false;
    drive.set()={0.,0.,0.};
  }
  else if(!decayed)
  {
    if(x>.8 || x<-0.8)
    {
      x = 0.005*x;
      y = 0.;
      phi=0.;
    }
    else
    {
      x=0.;

      if(y>.8 || y<-0.8)
      {
        y = 0.005*y;
        phi = 0.;
      }
      else
      {
        y=0.;
        
        if((phi-phistand)>1.3 ||(phi-phistand)<-1.3)
        {
          phi =-0.005*(phi-phistand);
        }
        else{
          phi=0.;
        }
      }
    }
    cout<<"  drive "<<x<<" "<<y<<" "<<phi<<endl;
    drive.set()={x,y,phi};

  }
  else
  {
    cout<<"x "<<x<<" y "<<y<<endl;
  }



}


G4HutoRoMap::G4HutoRoMap()
  : Thread("G4HutoRoMap", .05){}

void G4HutoRoMap::open()
{
  sr.resize(3,1);
  sl.resize(3,1);
  shoulderR.resize(3,1);
  shoulderL.resize(3,1);
  rP.resize(3,3);
  rP = 0;
  lP.resize(3,3);
  lP = 0;
  rp.resize(3,1);
  rp = 0;
  lp.resize(3,1);
  lp = 0;
  rPmean.resize(3,3);
  lPmean.resize(3,3);
  rpmean.resize(3,1);
  rpmean=0;
  lpmean.resize(3,1);
  lpmean=0;
  rpp = 0;
  lpp = 0;
  initmapper.set() = true;
  mid.load("g4mapping.kvg");
  cout<<"----------------------InitThread mapper Start-----------------"<<endl;
  cout<<"----------------Start moving freely,press Y for reset---------"<<endl;
}

void G4HutoRoMap::step(){

  //////////////////////////////////InPut Check////////////////////////////
  arr gpstate = gamepadState.get();

  CHECK(gpstate.N, "ERROR: No GamePad found");
  int button = gpstate(0);

//  floatA tempData = g4_data.get();
  floatA data = g4_data.get();
  floatA tempData = data.resizeCopy(6,7);
  floatA temp;


  if(tempData.N == 0){
    return;
  }
  // discard lost frames
  if (length(tempData.row(0)) == 0 || length(tempData.row(1)) == 0 ||
      length(tempData.row(3)) == 0 || length(tempData.row(4)) == 0 ||
      length(tempData.row(2))==0)
  {
    temp.clear();
    ftdata.set()=temp;
    return;
  }
  ////////////////////////////////////////////////////////////////////////
  temp=mid.query(tempData,STRING("/human/rl/rf"));
  ftdata.set()=temp;
  centerpos =  {0.20f,0.55f,.40f};

  /////////////////////////transform to robot cords//////////////////////

  for(uint i= 0 ; i<tempData.d0;i++)
  {
    tempData[i]=CORDtranstoRo(tempData[i],centerpos);
  }

  ///////////////////////////////////////////////////////////////////////

  if(button & BTN_X && !initialised)
  //if(!teleop && counter>20)
  {
    teleop=true;
    initphase = false;
    cout<<"---------TELEOP IS LIVE--------------"<<endl;
    initmapper.set() = initphase;
    initialised = true;
  }
  decayed =true;

  doinitsendROS(tempData);

}



void G4HutoRoMap::close(){
}


/// Transform the human hand position into the unit sphere.

floatA transformPosition(const floatA& thumb, const floatA& index, const floatA& center, float radius) {
  return (thumb.sub(0, 2) + index.sub(0, 2)) / 2.f;
}

floatA transformOrientation(const floatA &pose_thumb, const floatA &pose_index,bool right) {
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
  if(right)
  {
    y_pr2 = -pos_index + pos_thumb;
    y_pr2.normalize();
  }
  else
  {

    y_pr2 = pos_index - pos_thumb;
    y_pr2.normalize();
  }
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
/*
void G4HutoRoMap::transform(const floatA& poses_raw){

  floatA cal_pose_rh, cal_pose_lh;

  // Positions
  floatA poses_thumb_rh = mid.query(poses_raw, STRING("/human/rh/thumb"));
  floatA poses_index_rh = mid.query(poses_raw, STRING("/human/rh/index"));
  cal_pose_rh.append(transformPosition(poses_thumb_rh, poses_index_rh,
                                       shoulderR+centerpos, calarm_r_r));


  floatA poses_thumb_lh = mid.query(poses_raw, STRING("/human/lh/thumb"));
  floatA poses_index_lh = mid.query(poses_raw, STRING("/human/lh/index"));
  cal_pose_lh.append(transformPosition(poses_thumb_lh, poses_index_lh,
                                       shoulderL+centerpos, calarm_r_l));

  // Orientations
  cal_pose_rh.append(transformOrientation(poses_thumb_rh, poses_index_rh,false));
  cal_pose_lh.append(transformOrientation(poses_thumb_lh, poses_index_lh,true));

  // Gripper
  float dummy = 0;
  dummy = length(poses_thumb_rh.refRange(0, 2) - poses_index_rh.refRange(0, 2)) ;// * 1./(distrhmaxopen) -distrhminopen/distrhmaxopen;
  clip(dummy, 0.f, 0.9f);
  calibrated_gripper_rh.set() = dummy;

  dummy = 0;
  dummy = length(poses_thumb_lh.refRange(0, 2) - poses_index_lh.refRange(0, 2)) ;// *  1./(distlhmaxopen) - distlhminopen/distlhmaxopen ;
  clip(dummy, 0.f, 0.9f);
  calibrated_gripper_lh.set() = dummy;

  // setting access variables
  calibrated_pose_rh.set() = cal_pose_rh;
  calibrated_pose_lh.set() = cal_pose_lh;
}
*/
void G4HutoRoMap::doinitsendROS( floatA poses_raw)
{
  floatA cal_pose_rh, cal_pose_lh;

  // Positions
  floatA poses_thumb_rh = mid.query(poses_raw, STRING("/human/rh/thumb"));
  floatA poses_index_rh = mid.query(poses_raw, STRING("/human/rh/index"));
  cal_pose_rh.append(transformPosition(poses_thumb_rh, poses_index_rh,
                                       centerpos,1.f));


  floatA poses_thumb_lh = mid.query(poses_raw, STRING("/human/lh/thumb"));
  floatA poses_index_lh = mid.query(poses_raw, STRING("/human/lh/index"));
  cal_pose_lh.append(transformPosition(poses_thumb_lh, poses_index_lh,
                                       centerpos,1.f ));

  // Orientations
  cal_pose_rh.append(transformOrientation(poses_thumb_rh, poses_index_rh,false));
  cal_pose_lh.append(transformOrientation(poses_thumb_lh, poses_index_lh,true));

  // Gripper
  float dummy = 0;
  dummy = length(poses_thumb_rh.refRange(0, 2) - poses_index_rh.refRange(0, 2))-0.05;
  clip(dummy, 0.0f, 0.09f);
  calibrated_gripper_rh.set() = dummy;

  dummy = 0;
  dummy = length(poses_thumb_lh.refRange(0, 2) - poses_index_lh.refRange(0, 2))-0.05;
  clip(dummy, 0.f, 0.09f);
  calibrated_gripper_lh.set() = dummy;

  // setting access variables
  calibrated_pose_rh.set() = cal_pose_rh;
  calibrated_pose_lh.set() = cal_pose_lh;
}


