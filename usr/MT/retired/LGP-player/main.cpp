#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <Kin/kin.h>
#include <Optim/optimization.h>

#include <LGP/LGP.h>
#include <LGP/manipulationTree.h>

//===========================================================================

void LGPplayer(){
  mlr::KinematicWorld display;

  TowerProblem_new towers; //generates a randomize towers problem

//  cout <<towers.fol_root.KB <<endl;
  display=towers.world_root;
  towers.world_root.gl().update("root");

  ManipulationTree_Node root(towers);

  ManipulationTree_Node *node = &root;

  for(bool go=true;go;){
    bool terminal = node->fol.is_terminal_state();

    cout <<"==========================================================================" <<endl;
    cout <<"*** CURRENT NODE:\n" <<*node;
    node->expand();

    cout <<"\n*** CHOICES:" <<endl;
    cout <<"(q) quit" <<endl;
//    cout <<"(r) reset_state" <<endl;
//    cout <<"(m) make_current_initial" <<endl;
    cout <<"------------------------" <<endl;
    uint c=0;
    if(!terminal) for(ManipulationTree_Node *n:node->children){
      cout <<"(" <<c++ <<") DECISION: " <<*n->decision <<endl;
    }
    cout <<"------------------------" <<endl;

    if(!c){
      cout <<"NO CHOICES!" <<endl;
      break;
    }

    char cmd='1';
//    std::cin >>cmd;
    cout <<"COMMAND: '" <<cmd <<"'" <<endl;

    if(!terminal && cmd>='0' && cmd<='9'){
      node = node->children(int(cmd-'0'));
      cout <<"choosing decision " <<*node->decision <<endl;
      cout <<"------------------------" <<endl;
      node->solvePoseProblem();
      display=node->effKinematics;
      display.watch();
//      display.glAnimate();
      cout <<"------------------------" <<endl;
      node->solvePathProblem(20);
//      node->pathProblem.displayTrajectory(1, "KOMO planned trajectory", -0.01);
//      ::displayTrajectory(node->path, 1, towers.world_root, node->pathProblem.switches, "KOMO planned trajectory", -0.01);

      cout <<"------------------------" <<endl;
    //      display.gl().watch("child pose");

    }else switch(cmd){
      case 'q': go=false; break;
//      case 'r': world.reset_state(); break;
//      case 'm': world.make_current_state_new_start(); break;
      default: LOG(-1) <<"command '" <<c <<"' not known";
    }
  }

//#if 1
//        n->solvePoseProblem();
//#else
//        EffectivePoseProblem effectivePoseProblem(n->effKinematics, n->fol.KB, *node->folState, *n->folState, 0);
//        n->effPoseCost = effectivePoseProblem.optimize(n->effPose);
////        cout <<"n->effPoseCost=" <<n->effPoseCost <<" : " <<n->effPose <<endl;
//#endif

//        //      cout <<n->effKinematics <<endl;
//        display=n->effKinematics;
//        display.glAnimate();
//        display.gl().watch("child pose");

//        //      PathProblem pathProblem(towers.world_root, n->effKinematics, n->fol.KB, 20, 0);
//        //      n->pathCosts = pathProblem.optimize(n->path);
//        newFringe.append(n);
//      }
//      cout <<"NEW SUBTREE:" <<endl;
//      cout <<*node <<endl;
//    }
//    fringe = newFringe;
//  }

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
