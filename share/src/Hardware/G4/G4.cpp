#ifdef G4_INSTALLED

#include "G4.h"
#include <G4TrackIncl.h>
#include <string>
#include <iostream>

REGISTER_MODULE(G4Poller)

void lib_hardware_G4(){ cout <<"force loading lib/hardware/G4" <<endl; }

struct sG4Poller{
  int sysId;
  int hubs;
  intA hubList;
  intA hubMap;

  MT::Array<G4_FRAMEDATA> framedata;
  floatA poses;
};

namespace {
  std::string errcode2string(int errcode) {
    switch(errcode) {
      case G4_ERROR_NONE:
        return "success";
      case G4_ERROR_NO_FRAME_DATA_AVAIL:
        return "no frame data available";
      case G4_ERROR_UNSUPPORTED_ACTION:
        return "unsupported action";
      case G4_ERROR_UNSUPPORTED_COMMAND:
        return "unsupported command";
      case G4_ERROR_NO_CONNECTION:
        return "no connection";
      case G4_ERROR_NO_HUBS:
        return "no hubs";
      case G4_ERROR_FRAMERATE_SET:
        return "error framerate set";
      case G4_ERROR_MEMORY_ALLOCATION:
        return "error memory allocation";
      case G4_ERROR_INVALID_SYSTEM_ID:
        return "invalid system id";
      case G4_ERROR_SRC_CFG_FILE_OPEN:
        return "error opening src cfg file";
      case G4_ERROR_INVALID_SRC_CFG_FILE:
        return "invalid src cfg file";
      case G4_ERROR_UNABLE_TO_START_TIMER:
        return "unable to start timer";
      case G4_ERROR_HUB_NOT_ACTIVE:
        return "hub not active";
      case G4_ERROR_SYS_RESET_FAIL:
        return "system reset failed";
      case G4_ERROR_DONGLE_CONNECTION:
        return "error dongle connection";
      case G4_ERROR_DONGLE_USB_CONFIGURATION:
        return "error on dongle usb configuration";
      case G4_ERROR_DONGLE_USB_INTERFACE_0:
        return "error on dongle usb interface";
      case G4_ERROR_DUPLICATE_SYS_IDS:
        return "duplicate system ids";
      case G4_ERROR_INVALID_WILDCARD_USE:
        return "invalid wildcard use";
      case G4_ERROR_TOTAL:
        return "total";
      default:
      {
        std::ostringstream msg;
        msg << "unknown error, code " << errcode;
        return msg.str();
      }      
    }
  }
}

G4Poller::G4Poller():Module("G4Tracker"){
  s = new sG4Poller;
}

void G4Poller::open(){
  const char *src_cfg_file = "../../../configurations/g4_source_configuration.g4c";
  uint numHubs = MT::getParameter<int>("g4_numHubs");
  G4_CMD_STRUCT cs;
  int res;

  //--outer initialization loop - try multiple times
  bool isInitialized = false;
  while(!isInitialized){

    //-- initialization
    cout <<"G4 initialization ..." <<flush;
    for(uint i=0;i<10;i++){
      res = g4_init_sys(&s->sysId, src_cfg_file, NULL);
      if(res==G4_ERROR_NONE) { 
        break; //success!
      } else {
        std::clog << "Error initializing G4 system: " << errcode2string(res) << std::endl;
      }
      MT::wait(.1, false);
    }
    if(res!=G4_ERROR_NONE)
      HALT("G4Tracker initialization failed 10 times. g4_init_sys returned " <<res);
    cout <<" success" <<endl;

    //-- query #hubs
    cout <<"G4 quering #hubs, should be = " <<numHubs <<" ..." <<flush;
    cs.cmd = G4_CMD_GET_ACTIVE_HUBS;
    cs.cds.id = G4_CREATE_ID(s->sysId, 0, 0);
    cs.cds.action=G4_ACTION_GET;
    cs.cds.pParam=NULL;
    s->hubs=-1;
    uint k=0;
    for(k=0;k<10;k++){
      res = g4_set_query(&cs);
      s->hubs = cs.cds.iParam;
      if(s->hubs == (int)numHubs) break;
      cout << s->hubs << ", " << flush;
      MT::wait(.01, false);
    }
    if(k==10){
      cout <<" FAILED ... restarting" <<endl;
      g4_close_tracker();
    }else{
      cout <<" Found: " <<s->hubs <<" success" << endl;
      isInitialized=true;
    }

  }

  //-- allocate buffers: create hub list and G4_FRAMEDATA array
  s->hubList.resize(s->hubs);
  cs.cds.pParam = s->hubList.p;
  res = g4_set_query(&cs);
  if(res!=G4_ERROR_NONE){ close(); HALT(""); }
  s->framedata.resize(s->hubs);
  s->poses.resize(s->hubs, G4_SENSORS_PER_HUB, 7);
  s->poses.setZero();

  //-- allocate hubMap converter from hub ID to sequential hub ID
  s->hubMap.resize(s->hubList.max() + 1);
  s->hubMap = -1;
  for(int i=0; i<s->hubs; i++) s->hubMap(s->hubList(i)) = i;
  cout << "mapping hubIDs " << s->hubList << " -> " <<s->hubMap << endl;

  //-- configure: set quaternions and meters
  int quat_unit=G4_TYPE_QUATERNION;
  cs.cmd=G4_CMD_UNITS;
  cs.cds.id=G4_CREATE_ID(s->sysId,0,0);
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
  int res=g4_get_frame_data(s->framedata.p, s->sysId, s->hubList.p, s->hubs);
  int num_hubs_read=res&0xffff;

  /*
  int tot_sys_hubs=res>>16;
  cout <<"#existing hubs=" <<tot_sys_hubs
        <<" #data-avail hubs=" <<num_hubs_read <<endl;
  */

  if(num_hubs_read!=s->hubs) return; //-- assuming that g4 either returns all hubs or none

  // TODO by all means this is exactly the same as if the new positions were
  // identical to the previous ones.. is that reasonable? maybe it would be
  // better to avoid setting to 0, at least the previous values are maintained,
  // in that case. Keep this here just for now to see if that can actually
  // happen.
  s->poses.resize(s->hubs, G4_SENSORS_PER_HUB, 7);
  s->poses.setZero();

  int h_id, s_id;
  for(int hub=0; hub<num_hubs_read; hub++) {
    for(uint sen=0; sen<G4_SENSORS_PER_HUB; sen++) {
      if(s->framedata(hub).stationMap&(0x01<<sen)){ // we have data on hub h and sensors
        h_id = s->hubMap(s->framedata(hub).hub);
        s_id = s->framedata(hub).sfd[sen].id;
        memmove(&s->poses(h_id, s_id, 0), s->framedata(hub).sfd[sen].pos, 7*s->poses.sizeT); //low level copy of data
#if 0
        cout <<" hub " <<s->framedata(h).hub
            <<" sensor " <<s
           <<" frame " <<s->framedata(h).frame
          <<" id=" <<s->framedata(h).sfd[s].id
         <<" pos=" <<s->framedata(h).sfd[s].pos[0] <<' '<<s->framedata(h).sfd[s].pos[1] <<' ' <<s->framedata(h).sfd[s].pos[2]
        <<" ori=" <<s->framedata(h).sfd[s].ori[0] <<' '<<s->framedata(h).sfd[s].ori[1] <<' ' <<s->framedata(h).sfd[s].ori[2] <<' ' <<s->framedata(h).sfd[s].ori[3]
        <<endl;
#endif
      }
    }
  }

  s->poses.reshape(s->hubs*G4_SENSORS_PER_HUB, 7);
  currentPoses.set() = s->poses; //publish the result
}

#include <unistd.h>

void G4Poller::close(){
  usleep(1000000l);
  g4_close_tracker();
  usleep(1000000l);
}

#endif // ifdef G4_INSTALLED

