// System calls
#include<cstdio>
#include<cerrno>
#include<ctime>
#include<sys/stat.h>
#include<sys/types.h>
#include<fstream>
#include<string>
#include<iostream>
#include<cstdlib>
#include <unistd.h>  // usleep

#include"G4TrackIncl.h"

// built with
// g++ main.cpp -O2 -g -o test_app -lG4Track

// Default file names
#define DEFAULT_SRC_CFG "g4_setup.g4c"

using namespace std;

int init(string src_cfg);
void close();
void log(string msg);
void err(string msg);
void test(bool t, string msg);

int main(int argc, char **argv) {
    string src_cfg;
    int sysId;

    if(argc < 2)
        src_cfg = DEFAULT_SRC_CFG;
    else
        src_cfg = argv[1];

    usleep(10000);
    sysId = init(src_cfg);
    usleep(10000);

    G4_FRAMEDATA frames;

    int res;
                                                  
    G4_CMD_STRUCT cs;
    int hubs = 1;

    cs.cmd = G4_CMD_GET_ACTIVE_HUBS;
    cs.cds.id = G4_CREATE_ID(sysId,0,0);
    cs.cds.action=G4_ACTION_GET;
    cs.cds.pParam=NULL;

    do {
        res = g4_set_query(&cs);
        hubs = cs.cds.iParam;      // number of hubs in cs.cds.iParam
        cout << "Hubs from thing = " << hubs << endl;
    //} while(hubs != 2);
    } while(hubs != 1);
    cout << "Final hubs from thing = " << hubs << endl;

    //create hub list and G4_FRAMEDATA array
    int* hubList = new int[hubs];
    cs.cds.pParam = hubList;
    res=g4_set_query(&cs);
    G4_FRAMEDATA* fd=new G4_FRAMEDATA[hubs];

    res=g4_get_frame_data(fd,sysId,hubList,hubs);
    int num_hubs_read=res&0xffff;
    int tot_sys_hubs=res>>16;
    
    //    for(int i = 0; i < 10000; i++) {
    for(int i = 0; i < 100; i++) {
        res = g4_get_frame_data(fd, sysId, hubList, hubs);
        cout<< "num_hubs="<<(res&0xffff)<<endl;

        //cout << "Low Bits   = " << (res & 0xffff) << endl;
        //cout << "High Bits  = " << (res >> 16) << endl;

        for(int h = 0; h < (res&0xffff); h++) {
            cout << "[" << h << "] Hub        = " << fd[h].hub << endl;
            cout << "[" << h << "] Frame      = " << fd[h].frame << endl;
            cout << "[" << h << "] StationMap = " << fd[h].stationMap << endl;
        }
        usleep(8550);
    }

    close();

    return 0;
}

int init(string src_cfg) {
    int sysId;
    log("Initializing.");
    int res;
    int c = -1;
    do {
        res = g4_init_sys(&sysId, src_cfg.c_str(), NULL);
        cout << "Res: " << res << endl;
        c++;
    } while(res != 0);
    cout << "Init failed " << c << " times." << endl;
    cout << "SysId = " << sysId << endl;

    //   exit(1);
    return sysId;
}

void close() {
    cout << "Closing." << endl;
    g4_close_tracker();
}

void log(string msg) {
    cout << msg << endl;
}

void err(string msg) {
    cout << "Error: " << msg << endl;
}

void test(bool t, string msg) {
    if(!t) {
        err(msg);
        exit(1);
    }
}
