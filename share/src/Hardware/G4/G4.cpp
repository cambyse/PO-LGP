#ifdef G4_INSTALLED

#include "G4.h"
#include <G4TrackIncl.h>
#include <string>
#include <iostream>
#include <time.h>

REGISTER_MODULE(G4Poller)

void lib_hardware_G4(){ cout <<"force loading lib/hardware/G4" <<endl; }

struct sG4Poller{
  int sysId;
  int hubs;
  intA hubList;
  intA hubMap;

  MT::Array<G4_FRAMEDATA> framedata;
  floatA poses;
  timespec tstamp;

  uint num_reads;
  uint num_data_reads;
  uint num_hubs_read;
  uint32_t last_frame;
  uint num_frames;
  uint rep_frames;
  uint dropped_frames;
  uint dropped_hubs;
  double dropped_frames_pct;
  double dropped_hubs_pct;
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
  String src_cfg_file = MT::getParameter<String>("g4_srcCfg");
  uint numHubs = MT::getParameter<int>("g4_numHubs");
  G4_CMD_STRUCT cs;
  int res;

  //--outer initialization loop - try multiple times
  bool isInitialized = false;
  while(!isInitialized){
    //-- initialization
    cout <<"G4 initialization ..." <<flush;
    for(uint i=0;i<100;i++){
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
    cout <<"G4 quering #hubs, should be = " <<numHubs <<" ... " <<flush;
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
      MT::wait(.1, false);
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

  s->num_reads = 0;
  s->num_data_reads = 0;
  s->num_hubs_read = 0;
  s->last_frame = 1<<16;
  s->num_frames = 0;
  s->rep_frames = 0;
  s->dropped_frames = 0;
  s->dropped_frames_pct = 0;
  s->dropped_hubs = 0;
  s->dropped_hubs_pct = 0;
}

void G4Poller::step(){
  int res=g4_get_frame_data(s->framedata.p, s->sysId, s->hubList.p, s->hubs);
  // get the earlies timestamp you can
  clock_gettime(CLOCK_REALTIME, &s->tstamp);
  if(res < 0) {
	std::clog << "Error reading frame data:" << errcode2string(res) << std::endl;
	return;
  }
  s->num_reads++;
  int num_hubs_read=res&0xffff;
  //int num_hubs_reg=res>>16;

  //cout << num_hubs_read << flush;
  if(!num_hubs_read) return;
  //cout << endl;
  uint32_t frame = s->framedata(0).frame;
  bool first_frame = (s->last_frame == 1<<16);

  if(!first_frame &&
      frame <= s->last_frame &&
      frame + 100 >= s->last_frame) {
    s->rep_frames++;
    return; // ignoring repeated frames, for the moment at least..
  }
  s->num_data_reads++;

  s->num_hubs_read += num_hubs_read;

  if(first_frame) {
    s->num_frames = 1;
  }
  else {
    long int dframe = (long int)frame - (long int)s->last_frame;
    if(dframe < 0)
      dframe += 1 << 16;
    s->dropped_frames += dframe - 1;
    s->num_frames += dframe;
  }
  //if(frame < s->last_frame) {
    //cout << "====================================" << endl;
    //cout << "last: " << s->last_frame << endl;
    //cout << "====================================" << endl;
  //}
  s->last_frame = frame;
  s->dropped_frames_pct = (100. * s->dropped_frames) / s->num_frames;
  //cout << "num_frames: " << s->num_frames << endl;
  //cout << "now:  " << frame << " (" << num_hubs_read << " hubs: ";

  s->dropped_hubs = (s->hubs * s->num_frames) - s->num_hubs_read;
  s->dropped_hubs_pct = (100. * s->dropped_hubs) / (s->hubs * s->num_frames);

  s->poses.resize(s->hubs, G4_SENSORS_PER_HUB, 7);
  s->poses.setZero();

  int h_id, s_id;
  for(int hub=0; hub<num_hubs_read; hub++) {
    //cout << s->framedata(hub).hub << ", ";
    for(uint sen=0; sen<G4_SENSORS_PER_HUB; sen++) {
      if(s->framedata(hub).stationMap&(0x01<<sen)){ // we have data on hub h and sensors
        h_id = s->hubMap(s->framedata(hub).hub);
        s_id = s->framedata(hub).sfd[sen].id;
        memmove(&s->poses(h_id, s_id, 0), s->framedata(hub).sfd[sen].pos, 3*s->poses.sizeT); //low level copy of data
        memmove(&s->poses(h_id, s_id, 3), s->framedata(hub).sfd[sen].ori, 4*s->poses.sizeT); //low level copy of data
#if 0
        cout <<" hub " <<s->framedata(hub).hub
            <<" sensor " <<s
           <<" frame " <<s->framedata(hub).frame
          <<" id=" <<s->framedata(hub).sfd[s].id
         <<" pos=" <<s->framedata(hub).sfd[s].pos[0] <<' '<<s->framedata(hub).sfd[s].pos[1] <<' ' <<s->framedata(hub).sfd[s].pos[2]
        <<" ori=" <<s->framedata(hub).sfd[s].ori[0] <<' '<<s->framedata(hub).sfd[s].ori[1] <<' ' <<s->framedata(hub).sfd[s].ori[2] <<' ' <<s->framedata(hub).sfd[s].ori[3]
        <<endl;
#endif
      }
    }
  }
  //cout << ")" << endl;

  s->poses.reshape(s->hubs*G4_SENSORS_PER_HUB, 7);
  //cout << "poses: " << s->poses << endl;
  //cout << "currentPoses: " << currentPoses.get() << endl;
  poses.writeAccess();
  poses() = s->poses;
  // modulo today
  poses.tstamp() = s->tstamp.tv_sec % 86400 + s->tstamp.tv_nsec / 1000000000.;
  poses.deAccess();
  //cout << "currentPoses: " << currentPoses.get() << endl;
}

#include <unistd.h>

void G4Poller::close(){
  cout << "stats: " << endl;
  cout << " - num_hubs_read: " << s->num_hubs_read << " (expected: " << (s->num_frames * s->hubs) << ")" << endl;
  cout << " - num_data_reads: " << s->num_data_reads << " (expected: " << s->num_frames << ", reads: " << s->num_reads << ")" << endl;
  cout << " - dropped_frames: " << s->dropped_frames << " (" 
	<< s->dropped_frames_pct << "%)" << endl;
  cout << " - dropped_hubs: " << s->dropped_hubs << " (" 
	<< s->dropped_hubs_pct << "%)" << endl;
  cout << " - repeated_frames: " << s->rep_frames << endl;

  usleep(1000000l);
  cout << "closing.. " << flush;
  g4_close_tracker();
  cout << "DONE" << endl;
  usleep(1000000l);
}

#endif // ifdef G4_INSTALLED

