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


void setup_opengl_for_g4(ors::Graph& ors, OpenGL& gl, uint hubs){
  bindOrsToOpenGL(ors, gl);
  gl.camera.setPosition(7., .5, 3.);
  gl.camera.focus(0, .5, .5);
  gl.camera.upright();

  ors::Shape *s = new ors::Shape(ors, NULL);
  s->type = ors::markerST;
  s->size[0] = .5;

  for(uint m=0;m<hubs;m++){
    ors::Shape *s = new ors::Shape(ors, NULL);
    s->type = ors::boxST;
    memmove(s->size ,ARR(.10, .04, .01, 0).p, 4*sizeof(double));
    memmove(s->color,ARR(1, 0, 0).p, 3*sizeof(double));
  }
}


void threadedRun(){
  G4System S;
  OpenGL gl;
  ors::Graph ors;

  uint sensors = 3 * MT::getParameter<uint>("g4_numHubs");

  setup_opengl_for_g4(ors, gl, sensors);

  MT::String nowStr;
  MT::getNowString(nowStr);
  ofstream file_times(STRING("z." << nowStr << ".g4_times.dat"));
  ofstream file_poses(STRING("z." << nowStr << ".g4_pose.dat"));

  floatA poses;
  timeval time;
  char ts[25];

  engine().open(S);
  uint t;
  for(t=0;/*t<100*/;t++){
    if(engine().shutdown) break;

    S.currentPoses.var->waitForNextWriteAccess();

    gettimeofday(&time, 0);
    sprintf(ts, "%4i %8li.%06li", t, time.tv_sec&0xffffff, time.tv_usec);

    poses = S.currentPoses.get();
    poses.reshape(poses.N/7,7);
    if(!(t%100)){
      cout <<"frame=" <<t <<endl;
    }
    if(true){
      CHECK_EQ(poses.d0, sensors, "poses dim is wrong");
      CHECK_EQ(ors.shapes.N, 1+sensors, "ors.shapes dim is wrong")
      for(uint b=0; b+1<ors.shapes.N && b<sensors; b++){
        ors.shapes(b+1)->X.pos.set(poses(b,0), poses(b,1), poses(b,2));
        ors.shapes(b+1)->X.rot.set(poses(b,3), poses(b,4), poses(b,5), poses(b,6));
      }
      gl.text.clear() <<"frame " <<t <<" time" <<ts;
      gl.update();
    }

    file_times <<ts <<endl;
    file_poses <<poses <<endl; //-- TODO just for clarity, change it to binary later
    //file_poses.write((char*)poses.p, poses.sizeT*poses.N);
    //file_poses.flush();
  }
  cout <<"fps = " <<t/MT::timerRead() <<endl;

  file_times.close();
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

