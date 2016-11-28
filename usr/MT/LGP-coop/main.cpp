#include <Ors/ors.h>
#include <Ors/orsViewer.h>
#include <Gui/opengl.h>
#include <Gui/graphview.h>
#include <Ors/ors.h>
#include <Optim/optimization.h>

#include <LGP/LGP.h>
#include <LGP/manipulationTree.h>

//===========================================================================

void LGPplayer(){
  ors::KinematicWorld kin("LGP-coop-kin.g");
  FOL_World fol(FILE("LGP-coop-fol.g"));
  makeConvexHulls(kin.shapes);

  ManipulationTree_Node root(kin, fol);

  ManipulationTree_Node *node = &root;
  node->expand();

  system("evince z.pdf &");

  OrsViewer poseView("pose");
  OrsPathViewer seqView("sequence", 1.);
  OrsPathViewer pathView("path");
  threadOpenModules(true);

  charA cmds={ 'p', '0', '3', '1'};//, 'p', '4', 'p', 's', 'q' };
  cmds={ 'p', 'q' };
  bool interactive=true;
  bool autoCompute=true;

  bool go=true;
  for(uint s=0;go;s++){
    //-- display stuff
    if(node->poseProblem && node->poseProblem->configurations.N)
      poseView.modelWorld.set() = *node->poseProblem->configurations(0);
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

  LGPplayer();

  return 0;
}
