#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Ors/ors.h>
#include <Optim/optimization.h>

#include <LGP/LGP.h>
#include <LGP/manipulationTree.h>

//===========================================================================

#if 1
void ijcaiExperiment(){
  orsDrawJoints=false;
  orsDrawAlpha=1.;

  ofstream fil("data/samples.dat");
  fil <<"experiment #objects MCTS_time lev1_time f_bestEnd lev2_time lev3_time" <<endl;

  for(uint k=0;k<50;k++){
    TowerProblem_new towers; //generates a randomize towers problem

    ManipulationTree_Node *root = new ManipulationTree_Node(towers);

    ors::KinematicWorld world_display;
    double MCTS_time=0., lev1_time=0., lev2_time=0.;

    uint repeat;
    for(repeat=0;repeat<200;repeat++){
      //generate a new 'symbolic node' (here by fully unrolling)
      ManipulationTree_Node *node = new ManipulationTree_Node(root, NoHandle);
      mlr::timerRead(true);
      runMonteCarlo(node->fol.KB);
      MCTS_time += mlr::timerRead(true);

      //generate the respective effective pose problem
      EffectivePoseProblem effectivePoseProblem(node->effKinematics, node->fol.KB, *root->folState, *node->folState, 0);
      node->effPoseCost = effectivePoseProblem.optimize(node->effPose);
      double rx = towers.psi(node->effKinematics, node->fol.KB);
      lev1_time += mlr::timerRead(true);
      cout <<"fx=" <<node->effPoseCost <<endl;
      cout <<"reward=" <<rx <<endl;
      node->effPoseReward = rx-node->effPoseCost;
      world_display=node->effKinematics;
      world_display.gl().update();
    }
    cout <<"BEST:" <<endl;
    ManipulationTree_Node *best=NULL;
    for(auto* n:root->children) if(!best || n->effPoseReward>best->effPoseReward) best=n;
    best->effKinematics >>FILE("z.world_best.kvg");
    best->fol.KB >>FILE("z.symbols_best.kvg");

    world_display=best->effKinematics;
    world_display.gl().update();

    mlr::timerRead(true);
    PathProblem pathProblem(towers.world_root, best->effKinematics, best->fol.KB, 20, 0);
    best->pathCosts = pathProblem.optimize(best->path);
    lev2_time = mlr::timerRead(true);
    cout <<"f_path=" <<best->pathCosts <<endl;

    fil <<k <<' ' <<towers.nObjects <<' ' <<MCTS_time/repeat <<' ' <<lev1_time/repeat <<' ' <<best->effPoseReward <<' ' <<lev2_time <<endl;
  }
}
#endif
//===========================================================================

void newMethod(){
  ors::KinematicWorld display;

  TowerProblem_new towers; //generates a randomize towers problem

//  cout <<towers.fol_root.KB <<endl;
  display=towers.world_root;
  display.gl().watch("root");

  ManipulationTree_Node root(towers);

//  ManipulationTree_Node *node = &root;
  ManipulationTree_NodeL fringe={&root};

  for(uint k=0;k<2;k++){
    ManipulationTree_NodeL newFringe;
    for(ManipulationTree_Node *node:fringe){
      cout <<"EXPANDING:\n" <<*node;
      node->expand();
      //    for(ManipulationTree_Node *n:node->children) n->expand();
      for(ManipulationTree_Node *n:node->children){
        FILE("z.fol").getOs() <<n->fol.KB <<endl <<n->folState->isNodeOfParentGraph->keys.last();
        EffectivePoseProblem effectivePoseProblem(n->effKinematics, n->fol.KB, *node->folState, *n->folState, 0);
        n->effPoseCost = effectivePoseProblem.optimize(n->effPose);
//        cout <<"n->effPoseCost=" <<n->effPoseCost <<" : " <<n->effPose <<endl;

        //      cout <<n->effKinematics <<endl;
        display=n->effKinematics;
        display.gl().watch("child pose");

        //      PathProblem pathProblem(towers.world_root, n->effKinematics, n->fol.KB, 20, 0);
        //      n->pathCosts = pathProblem.optimize(n->path);
        newFringe.append(n);
      }
      cout <<"NEW SUBTREE:" <<endl;
      cout <<*node <<endl;
    }
    fringe = newFringe;
  }

  cout <<"** FULL TREE" <<endl;
  root.write();
  root.fol.KB.checkConsistency();

  cout <<root.fol.KB <<endl;

  cout <<"BYE BYE" <<endl;

//  for(;;){
//    T.addRollout(); //uses a tree policy to walk to a leaf, expands, r
//    T.optimEffPose(T.getRndNode());
//    T.optimPath(T.getRndNode());
//  }

}

//===========================================================================

int main(int argc,char **argv){
  mlr::initCmdLine(argc, argv);
//  rnd.clockSeed();
  rnd.seed(mlr::getParameter<int>("seed",0));

//  ijcaiExperiment();
  newMethod();

  return 0;
}
