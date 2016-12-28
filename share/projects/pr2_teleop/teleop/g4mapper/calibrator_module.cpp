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
        rpmean =rpmean+(1./SN)*~(temp.refRange(0,2));
        lpmean =lpmean+(1./SN)*~(temp.refRange(3,5));
        rP=rP+(1./SN)*((temp.refRange(0,2))*~temp.refRange(0,2));
        lP=lP+(1./SN)*((temp.refRange(3,5))*~temp.refRange(3,5));
        rp=rp+(1./SN)*((temp.refRange(0,2)*~temp.refRange(0,2)*temp.refRange(0,2)));
        lp=lp+(1./SN)*((temp.refRange(3,5)*~temp.refRange(3,5)*temp.refRange(3,5)));
        rpp=rpp+((1./SN)*(~temp.refRange(0,2)*temp.refRange(0,2)))(0);
        lpp=lpp+((1./SN)*(~temp.refRange(3,5)*temp.refRange(3,5)))(0);
    }
    else if(dp_i.d0>SN)
    {
        rpmean =rpmean+(1./SN)*(~(temp.refRange(0,2))-~(dp_i[0].refRange(0,2)));
        lpmean =lpmean+(1./SN)*(~(temp.refRange(3,5))-~(dp_i[0].refRange(3,5)));
        rP=rP+((1./SN)*(((temp.refRange(0,2))*~(temp.refRange(0,2)))
        -((dp_i[0].refRange(0,2))*~(dp_i[0].refRange(0,2)))));
        lP=lP+((1./SN)*(((temp.refRange(3,5))*~(temp.refRange(3,5)))
        -((dp_i[0].refRange(3,5))*~(dp_i[0].refRange(3,5)))));
        rp=rp+(1./SN)*((temp.refRange(0,2)*~temp.refRange(0,2)*temp.refRange(0,2))
        -(dp_i[0].refRange(0,2)*~dp_i[0].refRange(0,2)*dp_i[0].refRange(0,2)));
        lp=lp+(1./SN)*((temp.refRange(3,5)*~temp.refRange(3,5)*temp.refRange(3,5))
        -(dp_i[0].refRange(3,5)*~dp_i[0].refRange(3,5)*dp_i[0].refRange(3,5)));
        rpp=rpp+((1./SN)*(~temp.refRange(0,2)*temp.refRange(0,2)
        -~dp_i[0].refRange(0,2)*dp_i[0].refRange(0,2)))(0);
        lpp=lpp+((1./SN)*(~temp.refRange(3,5)*temp.refRange(3,5)
        -~dp_i[0].refRange(3,5)*dp_i[0].refRange(3,5)))(0);

       arr tempshift = dp_i;

        dp_i.clear();

        for(uint i = 1;i<=SN;i++)
        {
           dp_i.append(tempshift.row(i));

        }

    }
    else
    {
        rpmean =rpmean+(1./SN)*~(temp.refRange(0,2));
        lpmean =lpmean+(1./SN)*~(temp.refRange(3,5));
        rP=rP+(1./SN)*((temp.refRange(0,2))*~temp.refRange(0,2));
        lP=lP+(1./SN)*((temp.refRange(3,5))*~temp.refRange(3,5));
        rp=rp+(1./SN)*((temp.refRange(0,2)*~temp.refRange(0,2)*temp.refRange(0,2)));
        lp=lp+(1./SN)*((temp.refRange(3,5)*~temp.refRange(3,5)*temp.refRange(3,5)));
        rpp=rpp+((1./SN)*(~temp.refRange(0,2)*temp.refRange(0,2)))(0);
        lpp=lpp+((1./SN)*(~temp.refRange(3,5)*temp.refRange(3,5)))(0);
       return;
    }



    rPmean = rpmean*~rpmean;
    lPmean = lpmean*~lpmean;
    sr=0.5*(inverse(rPmean-rP)*(rpp*rpmean-rp));

    sl=0.5*(inverse(lPmean-lP)*(lpp*lpmean-lp));


    if( (length(sr))>0.1 && (length(sr))<0.6 && length(rpmean)<8. && length(rpmean) > 0.3 )
    {
        shoulderR.resize(3,1);
        for(uint i = 0 ; i<3 ; i++)
        {
             shoulderR(i,0)=(float)(sr(i,0));
        }
        shoulderRori = centerORI;
    }
    if((length(sl))>0.10 && (length(sl))<0.6 && length(lpmean)<8. && length(lpmean) > 0.3)
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
   // initphase = false;
}



}
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
    if(button & BTN_X)
    {


            initphase = false;
            demoidle = false;
            cout<<"---------TELEOP IS LIVE--------------"<<endl;
            initmapper.set() = initphase;


    }
    else if(button & BTN_B)
    {
        initphase = true;
        demoidle = false;
        initmapper.set() = initphase;
    }
/*    else if(button & BTN_A)
    {
        initphase = false;
        initmapper.set() = initphase;
        demoidle = true;
    }
*/

}

/////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////G4 HUMAN TO ROBOT MAPPER///////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
floatA CORDtranstoRo(const floatA& input,floatA centera)
{
    //-90° from G4 to roboto

    mlr::Vector PosToRobot;
    mlr::Quaternion OrToRobot;
    mlr::Quaternion trans;
    trans.setDeg(-90,0.,0.,1.);

    mlr::Vector center;
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
    mlr::Transformation transform;

    mlr::Vector refVector;
    mlr::Quaternion refOrien;
    refVector.set(ref(0),ref(1),ref(2));
    refOrien.set(ref(3),ref(4),ref(5),ref(6));
    mlr::Quaternion flip;
    flip.setDeg(180,0.,0.,1.);

    refOrien.alignWith(Vector_z);
    transform.addRelativeTranslation(refVector);
    //transform.addRelativeRotation(refOrien);
    transform.setInverse(transform);

    for(uint i =0 ;i<tempData.d0;i++)
    {
        mlr::Vector tempV;
        tempV.set(tempData[i](0),tempData[i](1),tempData[i](2));
        mlr::Quaternion tempQ;
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
    mlr::Transformation transform;

    mlr::Quaternion refOrien;
    refOrien.set(ref(0),ref(1),ref(2),ref(3));

    transform.addRelativeRotation(refOrien);
    refOrien.set(shoulderOR(3),shoulderOR(4),shoulderOR(5),shoulderOR(6));

    transform.addRelativeRotation(refOrien.invert());
    //transform.setInverse(transform);

    mlr::Vector shoulderV;
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
    mlr::Quaternion orsquats;
    orsquats.set(double(quats(0)),double(quats(1)),double(quats(2)),double(quats(3)));
    x = (double)TI_vec(0);
    y = (double)TI_vec(1);
    orsquats.alignWith(Vector_z);
    phi = orsquats.getRad();


    if(x <= 0.15 && x>=-0.15 && y <= 0.15 && y>=-0.15 &&  decayed)
    {
        phistand = orsquats.getRad();
        calisaysokay.set()=true;
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
        calisaysokay.set()=false;
    }



}

void G4HutoRoMap::initdriving(floatA tempData,int button)
{  
    if(button & BTN_X)
    {
        if(!btnpressed)
        {
            btnpressed = true;
        }
        else
        {
            return;
        }
    }
    else
    {
        btnpressed = false;
        return;
    }

    cout<<"\x1B[2J\x1B[H";
    floatA poses_index_rh = mid.query(tempData, STRING("/human/rh/index")).refRange(0,2);

    floatA pose_foot=mid.query(tempData,STRING("/human/rl/rf")).refRange(0,2);

   if(button & BTN_X && length(driveposX) == 0)
    {
            driveposX = poses_index_rh;
            return;

    }
    else if( length(driveposX) == 0)
    {
            cout<<"Point to max pos x and Press X"<<endl;
            return;

    }


    if(button & BTN_X &&  length(drivenegX) == 0)
    {
            drivenegX = poses_index_rh;
            return;

    }
    else if( length(drivenegX) == 0)
    {
            cout<<"Point to max neg x and Press X"<<endl;
            return;

    }


    if(button & BTN_X &&  length(driveRY) == 0.)
    {
            driveRY = poses_index_rh;
            return;

    }

    else if( length(driveRY) == 0.)
    {
            cout<<"Point to right y and Press X"<<endl;
            return;

    }

    if(button & BTN_X &&  length(driveLY) == 0.)
    {
            driveLY = poses_index_rh;
            return;

    }
    else if( length(driveLY) == 0.)
    {
            cout<<"Point to max pos x and Press X"<<endl;
            return;

    }

    if(button & BTN_X &&  length(turnL) == 0.)
    {
            turnL =  pose_foot;
            return;

    }
    else if( length(turnL) == 0.)
    {
            cout<<"Point to max pos x and Press X"<<endl;
            return;

    }
    if(button & BTN_X &&  length(turnR) == 0.)
    {
            turnR = pose_foot;
            return;

    }
    else if( length(turnR) == 0.)
    {
            cout<<"Point to max pos x and Press X"<<endl;
            return;

    }

    driveready = true;


}

void G4HutoRoMap::patterndriving(floatA tempData)
{
    floatA poses_index_rh = mid.query(tempData, STRING("/human/rh/index")).refRange(0,2);
    floatA poses_rf = mid.query(tempData, STRING("/human/rl/rf")).refRange(0,2);
    double turnrate = 0.;
    
    drive.set()={0.,0.,0.};
    if(length(poses_rf-turnR)<0.05)
    {
          turnrate = 0.005;
          calisaysokay.set()=true;

    cout<<"turnrate "<<turnrate<<endl;
    }
    else if(length(poses_rf-turnL)<0.05)
    {
         turnrate = -0.005;
         calisaysokay.set()=true;

    cout<<"turnrate "<<turnrate<<endl;
    }


    if(poses_index_rh(2)>=driveposX(2)-0.02 && poses_index_rh(2)<=driveposX(2)+0.02 )
    {
        floatA poses_index_rht =poses_index_rh.refRange(0,1);
        floatA driveposXt=driveposX.refRange(0,1);
        floatA drivenegXt=drivenegX.refRange(0,1);
        floatA driveLYt=driveLY.refRange(0,1);
        floatA driveRYt=driveRY.refRange(0,1);
        floatA xd = {2.f,0.f};
        floatA yd = {0.f,2.f};

       // drivenegX(2)=0.;
       // driveRY(2)=0.;
       // dirveLY(2)=0.;
        floatA xcompv = xd/(driveposXt-drivenegXt)% poses_index_rht -  xd/(driveposXt-drivenegXt)%(driveposXt+drivenegXt)/2.f;

        floatA ycompv = yd/(driveRYt-driveLYt)% poses_index_rht     -  yd/(driveRYt-driveLYt)%(driveRYt+driveLYt)/2.f;
        double xcomp = (double)(xcompv(0));
        double ycomp = -(double)(ycompv(1));
        cout<<"  drive "<<xcomp<<" "<<ycomp<<endl;
        if( xcomp >1. || ycomp >1.)
        {
            if(turnrate == 0.)
            {
                drive.set()={0.,0.,0. };
                calisaysokay.set()=false;
            }
            else
            {
                drive.set()={0.,0.,turnrate };
            }
        }
        else
        {

            calisaysokay.set()=true;
            if(sqrt(xcomp*xcomp)<0.3)
            {
                if(sqrt(ycomp*ycomp)>0.2)
                {
                    drive.set()={0.,0.002*ycomp,turnrate };
                    return;
                }
                else
                {
                    drive.set()={0.,0.,turnrate};
                    return;
                }
            }
            else if(sqrt(ycomp*ycomp)<0.2)
            {
                if(sqrt(xcomp*xcomp)>0.2)
                {
                    drive.set()={0.002*xcomp,0.,turnrate};
                    return;
                }
                else
                {
                    drive.set()={0.,0.,turnrate};
                    return;
                }
            }
            else
            {
                drive.set()={0.002*xcomp,0.002*ycomp,turnrate};
            }
        }

    }
}


G4HutoRoMap::G4HutoRoMap()
{
}
void G4HutoRoMap::open()
{
    calisaysokay.set()=false;
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
void G4HutoRoMap::step()
{

    //////////////////////////////////InPut Check////////////////////////////
    arr gpstate = gamepadState.get();
    CHECK(gpstate.N, "ERROR: No GamePad found");
    int button = gpstate(0);
    floatA tempData = poses.get();
floatA temp;

    
    if(tempData.N == 0)
     return;

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

    bool tappedacc ;
         tappedacc   = taped.get();
    ///////////////////////////////////////////////////////////////////////


    //centerpos =  mid.query(tempData,STRING("/human/torso/chest")).refRange(0,2);

   // centerORI =  mid.query(tempData,STRING("/human/torso/chest")).refRange(3,6);

   // tempData=transcenter(tempData,centerORI);
    // pusblish raw sensor data
   // poses_rh.set() = y+centerpos+shoulderR;
   // poses_lh.set() = mid.query(tempData, {"/human/lh/thumb", "/human/lh/index"})+centerpos+shoulderL;


   // if(initphase)
   // {
       // doinit(tempData,button);//MY VERSION
       // doinitandrea(tempData,button);
   //cout<<tappedacc<<endl;
   //

   if(!driveready)//routine to init driving pattern;
   {
        this->initdriving(tempData,button);
        return;
   }

    doinitpresaved(button);
    if(tappedacc)
    {

        if(!initphase)
        {
            this->patterndriving(tempData);
           // calcparameters(tempData);  hand driving mode;
        }
    }
    else
    {
        decayed =true;
        calisaysokay.set()=false;
    }

    doinitsendROS(tempData);
     //   return;

    //}
    //else
   // {
      //  shoulderR=transfshoulder(~shoulderR, shoulderRori, mid.query(tempData,STRING("/human/torso/chest")));

       // shoulderL=transfshoulder(~shoulderL, shoulderLori, mid.query(tempData,STRING("/human/torso/chest")));


       // transform(tempData);

       // transform_andrea(tmpPoses);
   // }

}



void G4HutoRoMap::close()
{
}



/// Transform the human hand position into the unit sphere.

floatA transformPosition(const floatA& thumb, const floatA& index, const floatA& center, float radius) {

  floatA pos_mean = (thumb.sub(0, 2) + index.sub(0, 2)) / 2.f ;







  return {pos_mean(0), pos_mean(1), pos_mean(2)};
/*
 // pos
 floatA pos_mean = (thumb.sub(0, 2) + index.sub(0, 2)) / 2.f - center;
  if (length(pos_mean) >= radius)
    pos_mean /= length(pos_mean);
  else
    pos_mean /= radius;



  return {pos_mean(0), pos_mean(1), pos_mean(2)};*/
}



floatA transformOrientation(const floatA &pose_thumb, const floatA &pose_index,bool right) {
  mlr::Quaternion quat;
  mlr::Vector x_thumb, x_index;
  mlr::Vector pos_thumb, pos_index;
  mlr::Vector x_pr2, y_pr2, z_pr2;



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
      cal_pose_rh.append(transformOrientation(poses_thumb_rh, poses_index_rh,false));
      cal_pose_lh.append(transformOrientation(poses_thumb_lh, poses_index_lh,true));

      // Gripper
      float dummy;
      // calibrated_gripper_rh.set() = clip(
      //     length(poses_thumb_rh.sub(0, 2) - poses_index_rh.sub(0, 2)) * m_rh + q_rh,
      //     0.f, 1.f);
      dummy = length(poses_thumb_rh.refRange(0, 2) - poses_index_rh.refRange(0, 2)) ;// * 1./(distrhmaxopen) -distrhminopen/distrhmaxopen;
      clip(dummy, 0.f, 0.9f);
      calibrated_gripper_rh.set() = dummy;
    //  cout<<"calibrated_gripper_rh "<<dummy<<endl;
      dummy = 0;
      // calibrated_gripper_lh.set() = clip(
      //     length(poses_thumb_lh.sub(0, 2) - poses_index_lh.sub(0, 2)) * m_lh + q_lh,
      //     0.f, 1.f);
      dummy = length(poses_thumb_lh.refRange(0, 2) - poses_index_lh.refRange(0, 2)) ;// *  1./(distlhmaxopen) - distlhminopen/distlhmaxopen ;
      clip(dummy, 0.f, 0.9f);
      calibrated_gripper_lh.set() = dummy;

    //  cout<<"calibrated_gripper_lh "<<dummy<<endl;

      // setting access variables
      calibrated_pose_rh.set() = cal_pose_rh;
      calibrated_pose_lh.set() = cal_pose_lh;

    //  cout<<"calibrated_pose_rh "<<cal_pose_rh<<endl;
    //  cout<<"calibrated_pose_lh "<<cal_pose_lh<<endl;
}

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
      float dummy;
      // calibrated_gripper_rh.set() = clip(
      //     length(poses_thumb_rh.sub(0, 2) - poses_index_rh.sub(0, 2)) * m_rh + q_rh,
      //     0.f, 1.f);
      dummy = length(poses_thumb_rh.refRange(0, 2) - poses_index_rh.refRange(0, 2))-0.05;
      clip(dummy, 0.0f, 0.09f);
      calibrated_gripper_rh.set() = dummy;
    //  cout<<"calibrated_gripper_rh "<<dummy<<endl;
      dummy = 0;
      // calibrated_gripper_lh.set() = clip(
      //     length(poses_thumb_lh.sub(0, 2) - poses_index_lh.sub(0, 2)) * m_lh + q_lh,
      //     0.f, 1.f);
      dummy = length(poses_thumb_lh.refRange(0, 2) - poses_index_lh.refRange(0, 2))-0.05;
      clip(dummy, 0.f, 0.09f);
      calibrated_gripper_lh.set() = dummy;

    //  cout<<"calibrated_gripper_lh "<<dummy<<endl;

      // setting access variables
      calibrated_pose_rh.set() = cal_pose_rh;
      calibrated_pose_lh.set() = cal_pose_lh;

    //  cout<<"calibrated_pose_rh "<<cal_pose_rh<<endl;
    //  cout<<"calibrated_pose_lh "<<cal_pose_lh<<endl;

    //drived.set()={x,y,phi};

}


