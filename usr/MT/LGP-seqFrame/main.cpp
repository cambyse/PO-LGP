#include <Ors/ors.h>
#include <Ors/orsviewer.h>
#include <Gui/opengl.h>
#include <Gui/graphview.h>
#include <Ors/ors.h>
#include <Optim/optimization.h>

#include <LGP/LGP.h>
#include <LGP/manipulationTree.h>


//===========================================================================

void LGPexample(){
  TowerProblem_new towers;

  ors::KinematicWorld display=towers.world_root;
  towers.world_root.gl().update("root");

  ManipulationTree_Node root(towers);

  ManipulationTree_Node *node = &root;

  uintA decisions = {0u,3,2,4};

  node->expand();
  for(uint s=0;s<decisions.N;s++){

    node = node->children(decisions(s));
    node->expand();
    node->solvePoseProblem();

    display=node->effKinematics;
    display.watch(false);
  }

  node->solveSeqProblem(2);
//  node->solvePathProblem(20);

  root.getGraph().writeDot(FILE("z.dot"));
  system("dot -Tpdf z.dot > z.pdf; evince z.pdf &");
  cout <<"BYE BYE" <<endl;
}

//===========================================================================

void LGPplayer(){
  SticksProblem towers;
  makeConvexHulls(towers.world_root.shapes);

  ManipulationTree_Node root(towers);

  ManipulationTree_Node *node = &root;
  node->expand();

  system("evince z.pdf &");

  OrsViewer poseView("pose");
  OrsPathViewer seqView("sequence", 1.);
  OrsPathViewer pathView("path");
  threadOpenModules(true);

  charA cmds={ 'p', '0', '3', '1'};//, 'p', '4', 'p', 's', 'q' };
  cmds={ /*'2', 'u',*/ '0', 'p', 'u', '1', 'u', '2', 'q' };
  bool interactive=true;
  bool autoCompute=true;

  bool go=true;
  for(uint s=0;go;s++){
    //-- display stuff
#if 1
    if(node->poseProblem && node->poseProblem->configurations.N)
      poseView.modelWorld.set() = *node->poseProblem->configurations(0);
#else
    poseView.modelWorld.set() = node->effKinematics;
#endif
    if(node->seqProblem && node->seqProblem->configurations.N)
      seqView.setConfigurations(node->seqProblem->configurations);
    else seqView.clear();
    if(node->pathProblem && node->pathProblem->configurations.N)
      pathView.setConfigurations(node->pathProblem->configurations);
    else pathView.clear();

    root.getGraph().writeDot(FILE("z.dot"), false, false, 0, node->graphIndex);
    system("dot -Tpdf z.dot > z.pdf");

    //-- query UI
    cout <<"********************" <<endl;
    cout <<"NODE:\n" <<*node <<endl;
    cout <<"--------------------" <<endl;
    cout <<"\nCHOICES:" <<endl;
    cout <<"(q) quit" <<endl;
    cout <<"(u) up" <<endl;
    cout <<"(p) pose problem" <<endl;
    cout <<"(s) sequence problem" <<endl;
    cout <<"(x) path problem" <<endl;
    uint c=0;
    for(ManipulationTree_Node* a:node->children){
      cout <<"(" <<c++ <<") DECISION: " <<*a->folDecision <<endl;
    }

    char cmd='q';
    if(interactive) std::cin >>cmd; else cmd=cmds(s);
    cout <<"COMMAND: '" <<cmd <<"'" <<endl;

    if(cmd>='0' && cmd<='9'){
      node = node->children(int(cmd-'0'));
      if(!node->isExpanded){
        node->expand();
        if(autoCompute){
          node->solvePoseProblem();
//          node->solveSeqProblem();
//          node->solvePathProblem(20);
        }
      }
    }else switch(cmd){
      case 'q': go=false; break;
      case 'u': if(node->parent) node = node->parent; break;
      case 'p': node->solvePoseProblem(); break;
      case 's': node->solveSeqProblem(); break;
      case 'x': node->solvePathProblem(20); break;
      default: LOG(-1) <<"command '" <<cmd <<"' not known";
    }
  }

  mlr::wait();
  threadCloseModules();
  cout <<"BYE BYE" <<endl;
}

//===========================================================================

int main(int argc,char **argv){
  mlr::initCmdLine(argc, argv);
//  rnd.clockSeed();
  rnd.seed(mlr::getParameter<int>("seed",0));

  switch(mlr::getParameter<int>("mode",1)){
    case 0: LGPexample(); break;
    case 1: LGPplayer(); break;
  }

  return 0;
}
