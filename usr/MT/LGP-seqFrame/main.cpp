#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Ors/ors.h>
#include <Optim/optimization.h>

#include <LGP/LGP.h>
#include <LGP/manipulationTree.h>

//===========================================================================

void LGPplayer(){
  TowerProblem_new towers;

  ors::KinematicWorld display=towers.world_root;
  towers.world_root.gl().update("root");

  ManipulationTree_Node root(towers);

  ManipulationTree_Node *node = &root;

  uintA decisions = {0u,3,2,4};

  for(uint s=0;s<decisions.N;s++){
    node->expand();

    node = node->children(decisions(s));
    node->solvePoseProblem();

    display=node->effKinematics;
    display.watch(false);
  }

  node->solveSeqProblem();

  root.getGraph().writeDot(FILE("z.dot"));
  system("dot -Tpdf z.dot > z.pdf; evince z.pdf;");
  cout <<"BYE BYE" <<endl;
}

//===========================================================================

int main(int argc,char **argv){
  mlr::initCmdLine(argc, argv);
//  rnd.clockSeed();
  rnd.seed(mlr::getParameter<int>("seed",0));

  LGPplayer();

  return 0;
}
