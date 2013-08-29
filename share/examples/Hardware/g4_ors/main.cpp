#include <System/engine.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>

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

  engine().open(S);
  for(uint i=0;i<10000;i++){
    S.currentPoses.var->waitForNextWriteAccess();
    poses = S.currentPoses.get();
    poses.reshape(poses.N/7,7);
    for(uint b=0; b<ors.bodies.N && b<poses.d0; b++){
      ors.bodies(0)->X.pos.set(poses(b,0), poses(b,1), poses(b,2));
      ors.bodies(0)->X.rot.set(poses(b,3), poses(b,4), poses(b,5), poses(b,6));
    }
    ors.calcShapeFramesFromBodies();
    gl.update();
  }
  engine().close(S);
}

int main(int argc, char **argv) {
  lib_hardware_G4();

  threadedRun();
  return 0;
}

