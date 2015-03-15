#include <iostream>
#include <unistd.h>

#ifdef G4_INSTALLED

#include"G4TrackIncl.h"

using namespace std;

void g4_main();
void create_thread();
void *thread_main(void *);

const char *src_cfg = "../../configurations/g4_source_configuration.g4c";

int main(int argc, char **argv) {
  if(argc == 1) // run non-threaded
    g4_main();
  else // run threaded
    create_thread();
}

void g4_main() {
  int sysId;

  usleep(10000);

  //-- initialization
  cout <<"Initializing G4." <<endl;
  int nfailures, res = -1;
  for(nfailures = 0; nfailures < 100 && res != 0; nfailures++) {
    res = g4_init_sys(&sysId, src_cfg, NULL);
    cout << "Initialization Res: " << res << endl;
    if(res == 0) {
      break;
    }
  }
  cout << "Init failed " << nfailures << " times." << endl;
  cout << "SysId = " << sysId << endl;
  if(res) exit(0);

  usleep(10000);

  G4_CMD_STRUCT cs;
  cs.cmd = G4_CMD_GET_ACTIVE_HUBS;
  cs.cds.id = G4_CREATE_ID(sysId, 0, 0);
  cs.cds.action = G4_ACTION_GET;
  cs.cds.pParam = NULL;

  int numhubs = 0;
  while(numhubs != 1) {
    usleep(10000);
    res = g4_set_query(&cs);
    numhubs = cs.cds.iParam; // number of hubs in cs.cds.iParam
    cout << "numhubs: " << numhubs << endl;
  }
  cout << "Final numhubs: " << numhubs << endl;

  // create hub list and G4_FRAMEDATA array
  int* hubList = new int[numhubs];
  cs.cds.pParam = hubList;
  res = g4_set_query(&cs);
  G4_FRAMEDATA* fd = new G4_FRAMEDATA[numhubs];

  // setting G4 parameters
  int quat_unit = G4_TYPE_QUATERNION;
  cs.cmd = G4_CMD_UNITS;
  cs.cds.id = G4_CREATE_ID(sysId,0,0);
  cs.cds.action = G4_ACTION_SET;
  cs.cds.iParam = G4_DATA_ORI;
  cs.cds.pParam = (void*)&quat_unit;
  res = g4_set_query(&cs); // sets orientation units to quaternions

  int meter_unit = G4_TYPE_METER;
  cs.cds.iParam = G4_DATA_POS;
  cs.cds.pParam = (void*)&meter_unit;
  res = g4_set_query(&cs); // sets orientation units to quaternions

  for(int counter = 0; counter < 20; ) {
    res = g4_get_frame_data(fd, sysId, hubList, numhubs);
    int numhubs_read = res&0xffff;
    counter += numhubs_read;

    for(int h = 0; h < numhubs_read; h++)
      for(uint s = 0; s < G4_SENSORS_PER_HUB; s++)
        if(fd[h].stationMap&(0x01<<s))
          cout << " frame " << fd[h].frame
                << " hub " << fd[h].hub
                << " sensor " << s
                << " id=" << fd[h].sfd[s].id
                << " pos=" << fd[h].sfd[s].pos[0] << ' ' << fd[h].sfd[s].pos[1]
                      << ' ' << fd[h].sfd[s].pos[2]
                << " ori=" << fd[h].sfd[s].ori[0] << ' ' << fd[h].sfd[s].ori[1]
                      << ' ' << fd[h].sfd[s].ori[2] << ' ' <<fd[h].sfd[s].ori[3]
                << endl;
  }

  cout << "Closing G4." << endl;
  g4_close_tracker();
}

void create_thread() {
  pthread_t thread;
  if(pthread_create(&thread, NULL, thread_main, NULL)) {
    cout << "Error creating thread!" << endl;
    exit(1);
  }
  pthread_join(thread, NULL);
}

void *thread_main(void *) {
  cout << "START THREAD" << endl;
  g4_main();
  cout << "EXIT THREAD" << endl;
  pthread_exit(0);
}

#else //G4_INSTALLED

int main(int argc, char **argv) {
  std::cout <<"G4 is not installed - can't run raw example" <<std::endl;
}

#endif
