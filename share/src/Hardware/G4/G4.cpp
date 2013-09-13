#include "G4.h"
#include <G4TrackIncl.h>

REGISTER_MODULE(G4Poller)

void lib_hardware_G4(){ cout <<"force loading lib/hardware/G4" <<endl; }

void G4Poller::open(){
  const char *src_cfg_file = "../../../configurations/g4_source_configuration.g4c";

  //-- initialization
  int res;
  for(uint i=0;i<100;i++){
    res = g4_init_sys(&sysId, src_cfg_file, NULL);
    if(res==G4_ERROR_NONE) break; //success!
    MT::wait(.001, false);
  }
  if(res) HALT("G4Tracker initialization failed 100 times. g4_init_sys returned " <<res);
  cout <<"G4Tracker opened" <<endl;
  MT::wait(.1);

  //-- query #hubs
  G4_CMD_STRUCT cs;
  cs.cmd = G4_CMD_GET_ACTIVE_HUBS;
  cs.cds.id = G4_CREATE_ID(sysId, 0, 0);
  cs.cds.action=G4_ACTION_GET;
  cs.cds.pParam=NULL;
  hubs=0;
  for(uint i=0;i<100;i++){
    res = g4_set_query(&cs);
    hubs = cs.cds.iParam;
    //if(hubs) break; //success!
    MT::wait(.001, false);
  }
  cout <<"#hubs from G4Tracker = " <<hubs <<endl;
  if(!hubs){ close(); HALT("no hubs found after 100 tries!"); }

  //-- allocate buffers: create hub list and G4_FRAMEDATA array
  hubList = new int[hubs];
  cs.cds.pParam = hubList;
  res = g4_set_query(&cs);
  if(res!=G4_ERROR_NONE){ close(); HALT(""); }
  framedata = new G4_FRAMEDATA[hubs];
  poses.resize(hubs, G4_SENSORS_PER_HUB, 7);
  poses.setZero();

  //-- configure: set quaternions and meters
  int quat_unit=G4_TYPE_QUATERNION;
  cs.cmd=G4_CMD_UNITS;
  cs.cds.id=G4_CREATE_ID(sysId,0,0);
  cs.cds.action=G4_ACTION_SET;
  cs.cds.iParam=G4_DATA_ORI;
  cs.cds.pParam=(void*)&quat_unit;
  res = g4_set_query(&cs);
  if(res!=G4_ERROR_NONE){ close(); HALT(""); }

  int meter_unit=G4_TYPE_METER;
  cs.cds.iParam=G4_DATA_POS;
  cs.cds.pParam=(void*)&meter_unit;
  res = g4_set_query(&cs);
  if(res!=G4_ERROR_NONE){ close(); HALT(""); }
}

void G4Poller::step(){
  int res=g4_get_frame_data(framedata, sysId, hubList, hubs);
  int num_hubs_read=res&0xffff;
  //  int tot_sys_hubs=res>>16;

  // cout <<"#existing hubs=" <<tot_sys_hubs
  // 	   <<" #data-avail hubs=" <<num_hubs_read <<endl;

  if(!num_hubs_read) return;
  poses.setZero();

  for(int h=0; h<num_hubs_read; h++) {
    for(uint s=0; s<G4_SENSORS_PER_HUB; s++) {
      if(framedata[h].stationMap&(0x01<<s)){ // we have data on hub h and sensor s
#if 0
        cout <<" hub " <<framedata[h].hub
            <<" sensor " <<s
           <<" frame " <<framedata[h].frame
          <<" id=" <<framedata[h].sfd[s].id
         <<" pos=" <<framedata[h].sfd[s].pos[0] <<' '<<framedata[h].sfd[s].pos[1] <<' ' <<framedata[h].sfd[s].pos[2]
        <<" ori=" <<framedata[h].sfd[s].ori[0] <<' '<<framedata[h].sfd[s].ori[1] <<' ' <<framedata[h].sfd[s].ori[2] <<' ' <<framedata[h].sfd[s].ori[3]
        <<endl;
#endif
        memmove(&poses(h,s,0), framedata[h].sfd[s].pos, 7*poses.sizeT); //low level copy of data
      }
    }
  }

  currentPoses.set() = poses;
}

void G4Poller::close(){
  g4_close_tracker();
}
