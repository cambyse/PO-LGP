#include <System/engine.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <sys/time.h>

void lib_hardware_G4();

struct G4System:System{
  ACCESS(floatA, currentPoses);
  G4System(){
    addModule("G4Poller", "G4Poller", ModuleThread::loopWithBeat, .001);
    connect();
  }
};

void getNowString(MT::String &str) {
  time_t t = time(0);
  struct tm *now = localtime(&t);

  char s[19]; //-- just enough
  sprintf(s, "%02d-%02d-%02d--%02d-%02d-%02d",
    now->tm_year-100,
    now->tm_mon+1,
    now->tm_mday,
    now->tm_hour,
    now->tm_min,
    now->tm_sec);

  str.clear() << s;
}

void threadedRun(){
  G4System S;
  OpenGL gl;
  ors::Graph ors;
  init(ors, gl, "g4_markers.ors");
  floatA poses;
  timeval time;
  MT::String nowStr, timesStr, poseStr;
  getNowString(nowStr);
  timesStr << "z." << nowStr << ".g4_times.dat";
  poseStr << "z." << nowStr << ".g4_pose.dat";
  FILE *fil = fopen((const char*)timesStr,"w");
  ofstream file_poses((const char*)poseStr);

  engine().open(S);
  uint t;
  for(t=0;/*t<100*/;t++){
    if(engine().shutdown) break;
    S.currentPoses.var->waitForNextWriteAccess();
    gettimeofday(&time, 0);
    poses = S.currentPoses.get();
    poses.reshape(poses.N/7,7);
    //cout <<t <<" #poses=" <<poses.d0 /*<<poses*/ <<endl;
    for(uint b=0; b+1<ors.bodies.N && b<poses.d0; b++){
      ors.bodies(b+1)->X.pos.set(poses(b,0), poses(b,1), poses(b,2));
      ors.bodies(b+1)->X.rot.set(poses(b,3), poses(b,4), poses(b,5), poses(b,6));
    }
    ors.calcShapeFramesFromBodies();
    gl.update();

    fprintf(fil, "%4i %8li.%06li\n", t, time.tv_sec&0xffffff, time.tv_usec);
    fflush(fil);

    file_poses <<poses <<endl; //-- just for clarity, change it to binary later
    //file_poses.write((char*)poses.p, poses.sizeT*poses.N);
    //file_poses.flush();
  }
  cout <<"fps = " <<t/MT::timerRead() <<endl;

  fclose(fil);
  file_poses.close();
  cout <<"bye bye" <<endl;
  engine().close(S);
}

int main(int argc, char **argv) {
  MT::initCmdLine(argc, argv);
  lib_hardware_G4();

  threadedRun();
  return 0;
}

