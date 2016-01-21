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

void LGPplayer(){
  TowerProblem_new towers;

  ManipulationTree_Node root(towers);

  ManipulationTree_Node *node = &root;
  node->expand();

  system("evince z.pdf &");

  OrsViewer poseView;
  OrsViewer seqView;
  threadOpenModules(true);

  for(bool go=true;go;){
    //-- display stuff
    if(node->poseProblem.configurations.N)
      poseView.modelWorld.set()=*node->poseProblem.configurations(0);
    if(node->seqProblem.configurations.N)
      seqView.modelWorld.set()=*node->poseProblem.configurations(0);
    root.getGraph().writeDot(FILE("z.dot"), false, false, 0, node->graphIndex);
    system("dot -Tpdf z.dot > z.pdf");

    //-- query UI
    cout <<"********************" <<endl;
    cout <<"NODE: " <<*node <<endl;
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

    char cmd='u';
    std::cin >>cmd;
    cout <<"COMMAND: '" <<cmd <<"'" <<endl;

    if(cmd>='0' && cmd<='9'){
      node = node->children(int(cmd-'0'));
      if(!node->isExpanded) node->expand();
    }else switch(cmd){
      case 'q': go=false; break;
      case 'u': if(node->parent) node = node->parent; break;
      case 'p': node->solvePoseProblem(); break;
      case 's': node->solveSeqProblem(); break;
      case 'x': node->solvePathProblem(10); break;
      default: LOG(-1) <<"command '" <<cmd <<"' not known";
    }
  }

  threadCloseModules();
  cout <<"BYE BYE" <<endl;
}

//===========================================================================

int main(int argc,char **argv){
  mlr::initCmdLine(argc, argv);
//  rnd.clockSeed();
  rnd.seed(mlr::getParameter<int>("seed",0));

//  LGPexample();
  LGPplayer();

  return 0;
}
