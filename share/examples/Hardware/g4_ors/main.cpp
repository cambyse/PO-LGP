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

void threadedRun(){
  G4System S;
  OpenGL gl;
  ors::Graph ors;
  init(ors, gl, "g4_markers.ors");
  floatA poses;
  timeval time;
  MT::String nowStr, timesStr, poseStr;
  MT::getNowString(nowStr);
  timesStr << "z." << nowStr << ".g4_times.dat";
  poseStr << "z." << nowStr << ".g4_pose.dat";
  FILE *fil = fopen((const char*)timesStr,"w");
  ofstream file_poses((const char*)poseStr);
  char ts[25];

  engine().open(S);
  uint t;
  for(t=0;/*t<100*/;t++){
    if(engine().shutdown) break;
    cout << "waiting" << endl;
    S.currentPoses.var->waitForNextWriteAccess();
    cout << "done" << endl;
    gettimeofday(&time, 0);
    poses = S.currentPoses.get();
    poses.reshape(poses.N/7,7);
    //cout <<t <<" #poses=" <<poses.d0 /*<<poses*/ <<endl;
    for(uint b=0; b+1<ors.bodies.N && b<poses.d0; b++){
      ors.bodies(b+1)->X.pos.set(poses(b,0), poses(b,1), poses(b,2));
      ors.bodies(b+1)->X.rot.set(poses(b,3), poses(b,4), poses(b,5), poses(b,6));
    }
    sprintf(ts, "%4i %8li.%06li", t, time.tv_sec&0xffffff, time.tv_usec);
    gl.text.clear() << ts;

    ors.calcShapeFramesFromBodies();
    gl.update();

    fprintf(fil, "%s\n", ts);
    fflush(fil);

    file_poses <<poses <<endl; //-- TODO just for clarity, change it to binary later
    //file_poses.write((char*)poses.p, poses.sizeT*poses.N);
    //file_poses.flush();
  }
  cout <<"fps = " <<t/MT::timerRead() <<endl;

  fclose(fil);
  file_poses.close();
  cout << "bye bye" << endl;
  engine().close(S);
}

int main(int argc, char **argv) {
  MT::initCmdLine(argc, argv);
  lib_hardware_G4();

  threadedRun();
  return 0;
}

