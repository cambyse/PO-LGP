// System calls
#include<cstdio>
#include<cerrno>
#include<ctime>
#include<sys/stat.h>
#include<sys/types.h>
#include<fstream>
#include<string>

// ML code
#include<MT/util.h>
#include<MT/array.h>
#include<MT/opengl.h>
#include<MT/opencv.h>
#include<MT/videoWriter.h>

#include"G4TrackIncl.h"

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

    sysId = init(src_cfg);

    G4_FRAMEDATA frames;
    int hub = 0;

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
    } while(hubs != 2);
    cout << "HUBS FROM THING = " << hubs << endl;

    //create hub list and G4_FRAMEDATA array
    int* hubList = new int[hubs];
    cs.cds.pParam = hubList;
    res=g4_set_query(&cs);
    G4_FRAMEDATA* fd=new G4_FRAMEDATA[hubs];

    res=g4_get_frame_data(fd,sysId,hubList,hubs);
    int num_hubs_read=res&0xffff;
    int tot_sys_hubs=res>>16;
    
    for(int i = 0; i < 10000; i++) {
        res = g4_get_frame_data(fd, sysId, hubList, hubs);
        //test(res == 0, "Failed get_frame_data().");

        //cout << "Low Bits   = " << (res & 0xffff) << endl;
        //cout << "High Bits  = " << (res >> 16) << endl;

        for(int h = 0; h < (res&0xffff); h++) {
            cout << "[" << h << "] Hub        = " << fd[h].hub << endl;
            cout << "[" << h << "] Frame      = " << fd[h].frame << endl;
            cout << "[" << h << "] StationMap = " << fd[h].stationMap << endl;
        }
        //cout << endl;

        /*
        log(STRING("   StationMap   = " << frames.stationMap));
        log(STRING("   Dig_IO       = " << frames.dig_io));
        log(STRING(endl));
        */
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
        c++;
        //cout << "res = " << res << endl;
    } while(res != 0);
    cout << "Init failed " << c << " times." << endl;
    //test(res == 0, "Failed g4_init_sys().");
    log(STRING("SysId = " << sysId));

    return sysId;
}

void close() {
    log("Closing.");
    g4_close_tracker();
}

void log(string msg) {
    cout << msg << endl;
}

void err(string msg) {
    log(STRING("Error: " << msg));
}

void test(bool t, string msg) {
    if(!t) {
        cout << "MSG: " << msg << endl;
        err(msg);
        exit(1);
    }
}

