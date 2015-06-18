#include "calibrator_module.h"
#include <Hardware/gamepad/gamepad.h>
#include <Ors/ors.h>
#include <Mocap/mocapdata.h>

//////////////////////////////////////////////////////////////////////////////
//////////////////////INIT THREAD/////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void G4HutoRoMap::getshoulderpos(floatA tempshoulderpos)
{

    tempshoulderpos.reshape(6);
    floatA tempcenter = centerpos;
    tempcenter.append(centerpos);
           arr temp(6);
       for(uint j = 0 ;j<tempshoulderpos.N;j++)
         temp(j)=(double)(tempshoulderpos(j)-tempcenter(j));






    dp_i.append(~temp);
    if(dp_i.d0 == SN)
    {
        rpmean =rpmean+(1./SN)*~(temp.subRange(0,2));
        lpmean =lpmean+(1./SN)*~(temp.subRange(3,5));
        rP=rP+(1./SN)*((temp.subRange(0,2))*~temp.subRange(0,2));
        lP=lP+(1./SN)*((temp.subRange(3,5))*~temp.subRange(3,5));
        rp=rp+(1./SN)*((temp.subRange(0,2)*~temp.subRange(0,2)*temp.subRange(0,2)));
        lp=lp+(1./SN)*((temp.subRange(3,5)*~temp.subRange(3,5)*temp.subRange(3,5)));
        rpp=rpp+((1./SN)*(~temp.subRange(0,2)*temp.subRange(0,2)))(0);
        lpp=lpp+((1./SN)*(~temp.subRange(3,5)*temp.subRange(3,5)))(0);
    }
    else if(dp_i.d0>SN)
    {
        rpmean =rpmean+(1./SN)*(~(temp.subRange(0,2))-~(dp_i[0].subRange(0,2)));
        lpmean =lpmean+(1./SN)*(~(temp.subRange(3,5))-~(dp_i[0].subRange(3,5)));
        rP=rP+((1./SN)*(((temp.subRange(0,2))*~(temp.subRange(0,2)))
        -((dp_i[0].subRange(0,2))*~(dp_i[0].subRange(0,2)))));
        lP=lP+((1./SN)*(((temp.subRange(3,5))*~(temp.subRange(3,5)))
        -((dp_i[0].subRange(3,5))*~(dp_i[0].subRange(3,5)))));
        rp=rp+(1./SN)*((temp.subRange(0,2)*~temp.subRange(0,2)*temp.subRange(0,2))
        -(dp_i[0].subRange(0,2)*~dp_i[0].subRange(0,2)*dp_i[0].subRange(0,2)));
        lp=lp+(1./SN)*((temp.subRange(3,5)*~temp.subRange(3,5)*temp.subRange(3,5))
        -(dp_i[0].subRange(3,5)*~dp_i[0].subRange(3,5)*dp_i[0].subRange(3,5)));
        rpp=rpp+((1./SN)*(~temp.subRange(0,2)*temp.subRange(0,2)
        -~dp_i[0].subRange(0,2)*dp_i[0].subRange(0,2)))(0);
        lpp=lpp+((1./SN)*(~temp.subRange(3,5)*temp.subRange(3,5)
        -~dp_i[0].subRange(3,5)*dp_i[0].subRange(3,5)))(0);

       arr tempshift = dp_i;

        dp_i.clear();

        for(uint i = 1;i<=SN;i++)
        {
           dp_i.append(tempshift.row(i));

        }

    }
    else
    {
        rpmean =rpmean+(1./SN)*~(temp.subRange(0,2));
        lpmean =lpmean+(1./SN)*~(temp.subRange(3,5));
        rP=rP+(1./SN)*((temp.subRange(0,2))*~temp.subRange(0,2));
        lP=lP+(1./SN)*((temp.subRange(3,5))*~temp.subRange(3,5));
        rp=rp+(1./SN)*((temp.subRange(0,2)*~temp.subRange(0,2)*temp.subRange(0,2)));
        lp=lp+(1./SN)*((temp.subRange(3,5)*~temp.subRange(3,5)*temp.subRange(3,5)));
        rpp=rpp+((1./SN)*(~temp.subRange(0,2)*temp.subRange(0,2)))(0);
        lpp=lpp+((1./SN)*(~temp.subRange(3,5)*temp.subRange(3,5)))(0);
       return;
    }



    rPmean = rpmean*~rpmean;
    lPmean = lpmean*~lpmean;
    sr=0.5*(inverse(rPmean-rP)*(rpp*rpmean-rp));

    sl=0.5*(inverse(lPmean-lP)*(lpp*lpmean-lp));


    if( (length(sr))>0.15 && (length(sr))<0.4 && length(rpmean)<8. && length(rpmean) > 0.5 )
    {
        shoulderR.resize(3,1);
        for(uint i = 0 ; i<3 ; i++)
        {
             shoulderR(i,0)=(float)(sr(i,0));
        }
        shoulderRori = centerORI;
    }
    if((length(sl))>0.15 && (length(sl))<0.4 && length(lpmean)<8. && length(lpmean) > 0.5)
    {
        shoulderL.resize(3,1);
        for(uint i = 0 ; i<3 ; i++)
        {
             shoulderL(i,0)=(float)(sl(i,0));
        }
        shoulderLori = centerORI;
    }



}
void G4HutoRoMap::getARMradius(floatA temp)
{
    float tempR= length(temp[0]-(~shoulderR+centerpos));
    float tempL= length(temp[1]-(~shoulderL+centerpos));

    if(tempR>calarm_r_r)
    {
        calarm_r_r = tempR;

    }
    if(tempL>calarm_r_l)
    {
        calarm_r_l= tempL;
    }
   cout<<tempR<<"    "<<tempL<<endl;
}
void G4HutoRoMap::getUnitPos(floatA temp)
{
    UnitPosR =(temp[0]-centerpos-shoulderR)/calarm_r_r;
    UnitPosL =(temp[1]-centerpos-shoulderL)/calarm_r_l;
    cout<<"UNIT POS R     UNIT POS L"<<endl;
    cout<<UnitPosR<<"    "<<UnitPosL<<endl;
}

void G4HutoRoMap::gripperinit(floatA tempgripperPos)
{
    float tempdistRG=length(tempgripperPos[0]-tempgripperPos[1]);
    float tempdistLG=length(tempgripperPos[2]-tempgripperPos[3]);
    if(tempdistRG > distrhmaxopen)
    {
        poserhthumbmaxopen = tempgripperPos[0];
        poserhindexmaxopen = tempgripperPos[1];
        distrhmaxopen =length(poserhthumbmaxopen-poserhindexmaxopen);
    }
    if(tempdistRG < distrhminopen && tempdistRG != 0)
    {
        poserhthumbminopen = tempgripperPos[0];
        poserhindexminopen = tempgripperPos[1];
        distrhminopen =length(poserhthumbminopen-poserhindexminopen);
    }

    if(tempdistLG > distlhmaxopen)
    {
        poselhthumbmaxopen = tempgripperPos[2];
        poselhindexmaxopen = tempgripperPos[3];
        distlhmaxopen =length(poselhthumbmaxopen-poselhindexmaxopen);
    }
    if(tempdistLG < distlhminopen && tempdistLG != 0)
    {
        poselhthumbminopen = tempgripperPos[2];
        poselhindexminopen = tempgripperPos[3];
        distlhminopen =length(poselhthumbminopen-poselhindexminopen);
    }


}


void G4HutoRoMap::doinit(floatA tempData,int button)
{


    cout<<cout<<"\x1B[2J\x1B[H";
    cout<<"PRESS X FOR ACCEPTING ,B SHOULDER RESET,Y GRIPPER RESET"<<endl;
    cout<<"Max dist open             Min dist open"<<endl;
    cout<<"left hand : "<<distlhmaxopen<<"          "<<distlhminopen<<endl;
    cout<<"right hand: "<<distrhmaxopen<<"          "<<distrhminopen<<endl<<endl;
    cout<<"----RS vektor----"<<endl<<"ABS_RS  "<<length(shoulderR)<<shoulderR<<"  "<<endl<<"-----LS vektor----"<<endl<<"ABS_LS  "<<length(shoulderL)<<"  "<<shoulderL<<endl;

    cout<<"----- CENTERPOS-----"<<endl<<centerpos<<endl;
    cout<<"---calarmR   calarmL------"<<endl;
    cout<<calarm_r_r<<"    "<<calarm_r_l<<endl;

    //cout<<tempData<<endl;

    gripperinit(mid.query(tempData,{ "/human/rh/thumb","/human/rh/index","/human/lh/thumb","/human/lh/index"}).cols(0,3));

if(shoulderinit)
{
    getshoulderpos(mid.query(tempData,{ "/human/rh/index","/human/lh/index"}).cols(0,3));
}
else
{
    getARMradius(mid.query(tempData,{ "/human/rh/index","/human/lh/index"}).cols(0,3));
    getUnitPos(mid.query(tempData,{ "/human/rh/index","/human/lh/index"}).cols(0,3));
}
if(button & BTN_X)
{
    shoulderinit =false;
}
if(button & BTN_B)
{
    calarm_r_r = 0;
    calarm_r_l = 0;
    shoulderinit = true;

}

if(button & BTN_Y)
{
        poselhthumbmaxopen.clear();
        poselhindexmaxopen.clear();
        poselhthumbminopen.clear();
        poselhindexminopen.clear();
        distlhmaxopen =0;
        distlhminopen =100;
        poserhthumbmaxopen.clear();
        poserhindexmaxopen.clear();
        poserhthumbminopen.clear();
        poserhindexminopen.clear();
        distrhmaxopen = 0;
        distrhminopen =100;
}

if(button & BTN_A)
{
    initphase = false;
}



}

/////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////G4 HUMAN TO ROBOT MAPPER///////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
floatA CORDtranstoRo(const floatA& input)
{
    //-90° from G4 to roboto

    ors::Vector PosToRobot;
    ors::Quaternion OrToRobot;
    ors::Quaternion trans;
    trans.setDeg(-90,0.,0.,1.);

    PosToRobot.set(input(0),input(1),input(2));
    OrToRobot.set(input(3) , input(4), input(5), input(6));

    PosToRobot = trans* PosToRobot;
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

G4HutoRoMap::G4HutoRoMap()
{
}
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

      // discard lost frames
      if (length(tempData.row(0)) == 0 || length(tempData.row(1)) == 0 ||
          length(tempData.row(3)) == 0 || length(tempData.row(4)) == 0 ||
          length(tempData.row(5))==0)
      return;


    //transform to robot cords
    // tempData = transcenter( tempData,mid.query(tempData,STRING("/human/torso/chest")));

    for(uint i= 0 ; i<tempData.d0;i++)
    {
        tempData[i]=CORDtranstoRo(tempData[i]);
    }



    centerpos =  mid.query(tempData,STRING("/human/torso/chest")).subRange(0,2);
    centerORI =  mid.query(tempData,STRING("/human/torso/chest")).subRange(3,6);


    // pusblish raw sensor data
    poses_rh.set() = mid.query(tempData, {"/human/rh/thumb", "/human/rh/index"})+centerpos+shoulderR;
    poses_lh.set() = mid.query(tempData, {"/human/lh/thumb", "/human/lh/index"})+centerpos+shoulderL;


    if(initphase)
    {
        doinit(tempData,button);
        return;

    }
    else
    {
      //  shoulderR=transfshoulder(~shoulderR, shoulderRori, mid.query(tempData,STRING("/human/torso/chest")));

      //  shoulderL=transfshoulder(~shoulderL, shoulderLori, mid.query(tempData,STRING("/human/torso/chest")));
        transform(tempData);

    }

}
void G4HutoRoMap::close()
{
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

void G4HutoRoMap::transform(const floatA& poses_raw)
{
    //  cout<<"\x1B[2J\x1B[H";

   //   cout<<"---- PUBLISHED DATA -----"<<endl;

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
      cal_pose_rh.append(transformOrientation(poses_thumb_rh, poses_index_rh));
      cal_pose_lh.append(transformOrientation(poses_thumb_lh, poses_index_lh));

      // Gripper
      float dummy;
      // calibrated_gripper_rh.set() = clip(
      //     length(poses_thumb_rh.sub(0, 2) - poses_index_rh.sub(0, 2)) * m_rh + q_rh,
      //     0.f, 1.f);
      dummy = length(poses_thumb_rh.subRange(0, 2) - poses_index_rh.subRange(0, 2)) * 1./(distrhmaxopen) -distrhminopen/distrhmaxopen;
      clip(dummy, 0.f, 1.f);
      calibrated_gripper_rh.set() = dummy;
    //  cout<<"calibrated_gripper_rh "<<dummy<<endl;
      dummy = 0;
      // calibrated_gripper_lh.set() = clip(
      //     length(poses_thumb_lh.sub(0, 2) - poses_index_lh.sub(0, 2)) * m_lh + q_lh,
      //     0.f, 1.f);
      dummy = length(poses_thumb_lh.subRange(0, 2) - poses_index_lh.subRange(0, 2)) *  1./(distlhmaxopen) - distlhminopen/distlhmaxopen ;
      clip(dummy, 0.f, 1.f);
      calibrated_gripper_lh.set() = dummy;

    //  cout<<"calibrated_gripper_lh "<<dummy<<endl;

      // setting access variables
      calibrated_pose_rh.set() = cal_pose_rh;
      calibrated_pose_lh.set() = cal_pose_lh;

    //  cout<<"calibrated_pose_rh "<<cal_pose_rh<<endl;
    //  cout<<"calibrated_pose_lh "<<cal_pose_lh<<endl;
}



