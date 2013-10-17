#include "G4.h"
#include <G4TrackIncl.h>

REGISTER_MODULE(G4Poller)

void lib_hardware_G4(){ cout <<"force loading lib/hardware/G4" <<endl; }

void G4Poller::open(){
  const char *src_cfg_file = "../../../configurations/g4_source_configuration.g4c";

  uint numHubs = MT::getParameter<int>("g4_numHubs");

  //-- initialization
  int res;
  for(uint i=0;i<10;i++){
    res = g4_init_sys(&sysId, src_cfg_file, NULL);
    if(res==G4_ERROR_NONE) break; //success!
    MT::wait(.1, false);
  }
  if(res!=G4_ERROR_NONE)
    HALT("G4Tracker initialization failed 100 times. g4_init_sys returned " <<res);
  cout <<"G4Tracker opened" <<endl;

  //-- query #hubs
  G4_CMD_STRUCT cs;
  cs.cmd = G4_CMD_GET_ACTIVE_HUBS;
  cs.cds.id = G4_CREATE_ID(sysId, 0, 0);
  cs.cds.action=G4_ACTION_GET;
  cs.cds.pParam=NULL;
  hubs=-1;
  cout << "Looking for " << numHubs << "hubs. Found: ";
  while(hubs != (int)numHubs) {
    res = g4_set_query(&cs);
    hubs = cs.cds.iParam;
    cout << hubs << ", " << flush;
    MT::wait(.01, false);
  }
  cout << "success!" << endl;

  //-- allocate buffers: create hub list and G4_FRAMEDATA array
  hubList = new int[hubs];
  cs.cds.pParam = hubList;
  res = g4_set_query(&cs);
  if(res!=G4_ERROR_NONE){ close(); HALT(""); }
  framedata = new G4_FRAMEDATA[hubs];
  poses.resize(hubs, G4_SENSORS_PER_HUB, 7);
  poses.setZero();

  //-- allocate hubMap converter from hub ID to sequential hub ID
  hubMapSize = 0;
  for(int i = 0; i < hubs; i++)
    if(hubList[i] > hubMapSize)
      hubMapSize = hubList[i];
  hubMapSize++;

  hubMap = new int[hubMapSize];
  memset(hubMap, -1, hubMapSize*sizeof(int));
  for(int i = 0; i < hubs; i++) {
    cout << "mapping hubID " << hubList[i] << " -> " << i << endl;
    hubMap[hubList[i]] = i;
  }

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

  /*
  int tot_sys_hubs=res>>16;
  cout <<"#existing hubs=" <<tot_sys_hubs
        <<" #data-avail hubs=" <<num_hubs_read <<endl;
  */

  if(!num_hubs_read) return; //-- assuming that g4 either returns all hubs or none

  // TODO by all means this is exactly the same as if the new positions were
  // identical to the previous ones.. is that reasonable? maybe it would be
  // better to avoid setting to 0, at least the previous values are maintained,
  // in that case. Keep this here just for now to see if that can actually
  // happen.
  poses.setZero();

  int h_id, s_id;
  for(int h=0; h<num_hubs_read; h++) {
    for(uint s=0; s<G4_SENSORS_PER_HUB; s++) {
      if(framedata[h].stationMap&(0x01<<s)){ // we have data on hub h and sensors
#if 0
        cout <<" hub " <<framedata[h].hub
            <<" sensor " <<s
           <<" frame " <<framedata[h].frame
          <<" id=" <<framedata[h].sfd[s].id
         <<" pos=" <<framedata[h].sfd[s].pos[0] <<' '<<framedata[h].sfd[s].pos[1] <<' ' <<framedata[h].sfd[s].pos[2]
        <<" ori=" <<framedata[h].sfd[s].ori[0] <<' '<<framedata[h].sfd[s].ori[1] <<' ' <<framedata[h].sfd[s].ori[2] <<' ' <<framedata[h].sfd[s].ori[3]
        <<endl;
#endif
        h_id = hubMap[framedata[h].hub];
        s_id = framedata[h].sfd[s].id;
        //memmove(&poses(h, s, 0), framedata[h].sfd[s].pos, 7*poses.sizeT); //low level copy of data
        memmove(&poses(h_id, s_id, 0), framedata[h].sfd[s].pos, 7*poses.sizeT); //low level copy of data
      }
    }
  }

  currentPoses.set() = poses;
}

void G4Poller::close(){
  g4_close_tracker();
}
