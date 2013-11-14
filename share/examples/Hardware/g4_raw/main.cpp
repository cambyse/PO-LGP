// System calls
#include <cstdio>
#include <cerrno>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>
#include <fstream>
#include <string>
#include <iostream>
#include <cstdlib>
#include <unistd.h>  // usleep

#include"G4TrackIncl.h"

using namespace std;

int main(int argc, char **argv) {
  string src_cfg;
  int sysId;

  if(argc < 2)
    src_cfg = "../../../configurations/g4_source_configuration.g4c";
  else
    src_cfg = argv[1];

  usleep(10000);

  //-- initialization
  cout <<"Initializing." <<endl;
  int res, c;
  for(c = -1; c < 100; c++) {
    res = g4_init_sys(&sysId, src_cfg.c_str(), NULL);
    cout << "Initialization Res: " << res << endl;
    if(res == 0)
      break;
  }
  cout << "Init failed " << c << " times." << endl;
  cout << "SysId = " << sysId << endl;
  if(res) exit(0);

  usleep(10000);

  G4_CMD_STRUCT cs;
  int hubs = 1;

  cs.cmd = G4_CMD_GET_ACTIVE_HUBS;
  cs.cds.id = G4_CREATE_ID(sysId, 0, 0);
  cs.cds.action = G4_ACTION_GET;
  cs.cds.pParam = NULL;

  hubs = 0;
  while(hubs != 5) {
    usleep(10000);
    res = g4_set_query(&cs);
    hubs = cs.cds.iParam;      // number of hubs in cs.cds.iParam
    cout << "Hubs from thing = " << hubs << endl;
  }
  cout << "Final hubs from thing = " << hubs << endl;

  //create hub list and G4_FRAMEDATA array
  int* hubList = new int[hubs];
  cs.cds.pParam = hubList;
  res = g4_set_query(&cs);
  G4_FRAMEDATA* fd = new G4_FRAMEDATA[hubs];


  int quat_unit = G4_TYPE_QUATERNION;
  cs.cmd = G4_CMD_UNITS;
  cs.cds.id = G4_CREATE_ID(sysId,0,0);
  cs.cds.action = G4_ACTION_SET;
  cs.cds.iParam = G4_DATA_ORI;
  cs.cds.pParam = (void*)&quat_unit;
  res = g4_set_query(&cs);         // sets orientation units to quaternions

  int meter_unit = G4_TYPE_METER;
  cs.cds.iParam = G4_DATA_POS;
  cs.cds.pParam = (void*)&meter_unit;
  res = g4_set_query(&cs);         // sets orientation units to quaternions

  //for(int i = 0; i < 1000; i++) {
  for(uint t = 0;; t++) {
    res = g4_get_frame_data(fd,sysId,hubList,hubs);
    int num_hubs_read = res&0xffff;
    int tot_sys_hubs = res>>16;

    // cout <<"#existing hubs=" <<tot_sys_hubs
    // 	   <<" #data-avail hubs=" <<num_hubs_read <<endl;

    for(int h = 0; h < num_hubs_read; h++) {
      // cout << "[" << h << "] Hub        = " << fd[h].hub << endl;
      // cout << "[" << h << "] Frame      = " << fd[h].frame << endl;
      // cout << "[" << h << "] StationMap = " << fd[h].stationMap << endl;
      for(uint s = 0; s < G4_SENSORS_PER_HUB; s++){
        if(fd[h].stationMap&(0x01<<s)){
          cout << " hub " << fd[h].hub
                << " sensor " << s
                << " frame " << fd[h].frame
                << " id=" << fd[h].sfd[s].id
                << " pos=" << fd[h].sfd[s].pos[0] << ' ' << fd[h].sfd[s].pos[1]
                      << ' ' << fd[h].sfd[s].pos[2]
                << " ori=" << fd[h].sfd[s].ori[0] << ' ' << fd[h].sfd[s].ori[1]
                      << ' ' << fd[h].sfd[s].ori[2] << ' ' <<fd[h].sfd[s].ori[3]
                << endl;
        }
      }
    }
    cout << "." << t;
    usleep(855000000l);
  }

  cout << "Closing." << endl;
  g4_close_tracker();

  return 0;
}

