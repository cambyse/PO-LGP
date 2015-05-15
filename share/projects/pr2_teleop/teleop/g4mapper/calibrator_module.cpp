#include "calibrator_module.h"
#include <Hardware/gamepad/gamepad.h>
#include <Ors/ors.h>
#include <Mocap/mocapdata.h>

//////////////////////////////////////////////////////////////////////////////
//////////////////////INIT THREAD/////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
floatA G4HutoRoMap::getshoulderpos(floatA tempshoulderpos,floatA &p_i)
{
    floatA pmean(3,1);
    //float  pdistmean;
    p_i.append(~(-centerpos+tempshoulderpos));
    if(p_i.d0 == SN)
    {
    }
    else if(p_i.d0>SN)
    {
        floatA tempshift = p_i;
        p_i.clear();
        for(uint i = 1;i<=SN;i++)
        {
            p_i.append(tempshift.row(i));
        }

    }
    else
    {

        return  {0.f};
    }
    cout<<"\x1B[2J\x1B[H";

  //  cout<<"Max dist open             Min dist open"<<endl;
  //  cout<<"left hand :"<<distlhmaxopen<<"          "<<endl;
  //  cout<<"right hand: "<<distrhmaxopen<<"          "<<endl<<endl;

       floatA temp(p_i);
       temp =1;
       temp = (~p_i*temp);
       pmean(0,0)= temp(0,0);
       pmean(1,0)= temp(1,1);
       pmean(2,0)= temp(2,2);
       pmean = 1.f/SN*pmean;


     //  pdistmean = 1.f/SN*trace(~p_i*p_i);


       floatA pipitpi(3,1);
       pipitpi = 0 ;
       float pitpi=0;



       floatA P_i(3,3);
       P_i = 0;


       for(uint i = 0; i<SN;i++)
       {
           pitpi = pitpi + (~p_i[i]*p_i[i])(0);
           P_i =P_i+p_i[i]*~p_i[i];
           pipitpi = pipitpi+ p_i[i]*~p_i[i]*p_i[i];
       }
       pitpi = 1.f/SN*(pitpi);
        pipitpi = 1.f/SN*(pipitpi);
        P_i = 1.f/SN*(P_i);


       floatA P =(pmean*~pmean-P_i);
       arr douP(P.d0,P.d1);
       for(uint i = 0;i<P.d0;i++)
           for(uint j = 0 ;j<P.d1;j++)
               douP(i,j)=(double)(P(i,j));
       douP = inverse(douP);
       for(uint i = 0;i<P.d0;i++)
           for(uint j = 0 ;j<P.d1;j++)
               P(i,j)=(float)(douP(i,j));



        return (0.5f*P*(pitpi*pmean-pipitpi));

/*
  //-------------------- ???------------------
       floatA temp(p_i);
       temp =1;
       temp = (~p_i*temp);
       pmean(0,0)= temp(0,0);
       pmean(1,0)= temp(1,1);
       pmean(2,0)= temp(2,2);
       pmean = 1.f/SN*pmean;


       pdistmean = 1.f/SN*trace(~p_i*p_i);
       floatA I;
       I.setDiag(pdistmean,3);
       floatA P =I-pmean*~pmean;
       arr douP(P.d0,P.d1);
       for(uint i = 0;i<P.d0;i++)
           for(uint j = 0 ;j<P.d1;j++)
               douP(i,j)=(double)(P(i,j));
       douP = inverse(douP);
       for(uint i = 0;i<P.d0;i++)
           for(uint j = 0 ;j<P.d1;j++)
               P(i,j)=(float)(douP(i,j));



        return ~(0.5f*pdistmean*(~pmean*P));
//----------------------------------------------------
*/
    // cout<<"----P matrix----"<<endl<<pmean<<endl;
    // cout<<"----p_i   vektor----"<<endl<<p_i<<endl;
    // cout<<"---- pdistmean------"<<endl<<pdistmean<<endl;

    // cout<<"----q ---"<<endl<<q<<endl;



}
void G4HutoRoMap::gripperinit(floatA tempgripperPos)
{
   // floatA tempPoslhIndex = mid.query(tempData,STRING( "/human/lh/index")).subRange(0, 2);
   // floatA tempPoslhThumb =  mid.query(tempData,STRING("/human/lh/thumb")).subRange(0, 2);
    if(length(tempgripperPos[0]-tempgripperPos[1]) > distlhmaxopen)
    {
        poselhthumbmaxopen = tempgripperPos[0];
        poselhindexmaxopen = tempgripperPos[1];
        distlhmaxopen =length(poselhthumbmaxopen-poselhindexmaxopen);
    }
 //   floatA tempPosrhIndex = mid.query(tempData,STRING( "/human/rh/index")).subRange(0,2);
 //   floatA tempPosrhThumb =  mid.query(tempData,STRING("/human/rh/thumb")).subRange(0,2);
    if(length(tempgripperPos[2]-tempgripperPos[3]) > distrhmaxopen)
    {
        poserhthumbmaxopen = tempgripperPos[2];
        poserhindexmaxopen = tempgripperPos[3];
        distrhmaxopen =length(poselhthumbmaxopen-poselhindexmaxopen);
    }



}


void G4HutoRoMap::doinit(floatA tempData,int button)
{
    cout<<cout<<"\x1B[2J\x1B[H";
//cout<<lp_i<<endl<<rp_i<<endl;
    cout<<"Max dist open             Min dist open"<<endl;
    cout<<"left hand :"<<distlhmaxopen<<"          "<<endl;
    cout<<"right hand: "<<distrhmaxopen<<"          "<<endl<<endl;
    cout<<"----RS vektor----"<<endl<<"ABS_RS  "<<length(shoulderR)<<"  "<<shoulderR<<endl<<"-----LS vektor----"<<endl<<"ABS_LS  "<<length(shoulderL)<<"  "<<shoulderL<<endl;
    cout<<"----- CENTERPOS-----"<<endl<<centerpos<<endl;


    //gripperinit(mid.query(tempData,{ "/human/lh/index","/human/lh/thumb","/human/rh/index","/human/rh/thumb"}).cols(0,2));

    shoulderL=getshoulderpos(mid.query(tempData,STRING( "/human/lh/index")).subRange(0,2),lp_i);
    shoulderR=getshoulderpos(mid.query(tempData,STRING( "/human/rh/index")).subRange(0,2),rp_i);



if(button & BTN_Y)
{

        poselhthumbmaxopen.clear();
        poselhindexmaxopen.clear();
        distlhmaxopen =0;
        poserhthumbmaxopen.clear();
        poserhindexmaxopen.clear();
        distrhmaxopen =0;
}



   /*   // starting in calibraton phase:
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
   calibration_phase = false;*/

}

//void initG4Mapper::calibrate()
//{
         /* // arm position
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
          q_lh = - dist_closed * m_lh;*/
//}

/////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////G4 HUMAN TO ROBOT MAPPER///////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////


G4HutoRoMap::G4HutoRoMap()
{
}
void G4HutoRoMap::open()
{
    mid.load("g4mapping.kvg");
    cout<<"----------------------InitThread mapper Start-----------------"<<endl;
    cout<<"----------------Start moving freely,press Y for reset---------"<<endl;
}
void G4HutoRoMap::step()
{

    /////////InPut Check///////
    arr gpstate = gamepadState.get();
    CHECK(gpstate.N, "ERROR: No GamePad found");
    int button = gpstate(0);
    floatA tempData = poses.get();
    if(tempData.N == 0)
     return;
   // else
   // tempData = mid.query(tempData,STRING("/human/rl/rf"));
    ///////////////////////////

    centerpos =  mid.query(tempData,STRING("/human/torso/chest")).subRange(0,2);



    if(initphase)
    {
        doinit(tempData,button);
        return;

    }
    else
    {


    }

  //  transformPosition();
  //   transformOrientatiob();
}
void G4HutoRoMap::close()
{
}

/// Transform the human hand position into the unit sphere.
/*
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
*/
//void G4HuToRoMap::transform(const floatA& poses_raw)
//{

   /*   floatA cal_pose_rh, cal_pose_lh;

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
      calibrated_pose_lh.set() = cal_pose_lh;*/
//}



