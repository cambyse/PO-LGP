#include <Core/util.tpp>
#include <Gui/opengl.h>

#include <Motion/motion.h>
#include <Motion/motionHeuristics.h>
#include <Motion/taskMaps.h>

#include <Motion/komo.h>

#include <Ors/ors_swift.h>
#include <LGP/manipulationTree.h>

#include <Ors/ors.h>
#include <Ors/orsviewer.h>
#include <Gui/opengl.h>
#include <Gui/graphview.h>
#include <Ors/ors.h>
#include <Optim/optimization.h>

#include <LGP/LGP.h>
#include <LGP/manipulationTree.h>

//===========================================================================

void coop(){
  //-- prepare kinematic world
  ors::KinematicWorld kin("LGP-coop-kin.g");

  kin.watch();

  BodyL box;
  ors::Body *tableC = kin.getBodyByName("table");
  ors::Body *tableL = kin.getBodyByName("tableL");
  ors::Body *tableR = kin.getBodyByName("tableR");
  mlr::Array<ors::Transformation> targetAbs, targetRel;

  { //grab desired final configuration & create initial configuration, placing objects far on the table

    for(ors::Body *b:kin.bodies) if(b->name.startsWith("/toolbox")) box.append(b);

    targetAbs.resize(box.N);
    targetRel.resize(box.N, box.N);
    for(uint i=0;i<box.N;i++){
      targetAbs(i) = box(i)->X;
      for(uint j=0;j<i;j++) targetRel(i,j).setDifference(box(j)->X, box(i)->X);
    }

    double xpos = -.6;
    for(ors::Body *b:box){
      ors::Joint *j = b->inLinks.scalar();
      tableC->outLinks.removeValue(j);
      j->from = tableL;
      tableL->outLinks.append(j);
      kin.checkConsistency();

      j->B.setZero();
      j->B.addRelativeTranslation(xpos, 0,0);
      j->B.addRelativeRotationDeg(90,0,0,1);
      xpos += .15;
    }
    kin.calc_fwdPropagateFrames();
  }
  kin.watch(true);

  //-- prepare logic world
  FOL_World fol(FILE("LGP-coop-fol.g"));
  for(ors::Body *b:box) fol.addObject(b->name);
  fol.addObject("screwdriver");
  fol.addObject("screwbox");
  fol.addAgent("baxterL");
  fol.addAgent("baxterR");
  fol.addAgent("handL");
  fol.addAgent("handR");

  FILE("z.fol") <<fol;

  //-- start manipulation tree
  ManipulationTree_Node root(kin, fol);

  ManipulationTree_Node *node = &root;
  node->expand();

//  system("evince z.pdf &");

  OrsViewer poseView("pose");
  OrsPathViewer seqView("sequence", 1.);
  OrsPathViewer pathView("path");
  threadOpenModules(true);

  StringA cmds={ "p", "0", "3", "1"};//, "p", "4", "p", "s", "q" };
  cmds={ "0", "23", "x", "q" };
  bool interactive=false;
  bool autoCompute=true;

  bool go=true;
  for(uint s=0;go;s++){
    //-- display stuff
    if(node->poseProblem && node->poseProblem->MP->configurations.N)
      poseView.modelWorld.set() = *node->poseProblem->MP->configurations.last();
    if(node->seqProblem && node->seqProblem->MP->configurations.N)
      seqView.setConfigurations(node->seqProblem->MP->configurations);
    else seqView.clear();
    if(node->pathProblem && node->pathProblem->MP->configurations.N)
      pathView.setConfigurations(node->pathProblem->MP->configurations);
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

    mlr::String cmd;
    if(interactive){
      std::string tmp;
      getline(std::cin, tmp);
      cmd=tmp.c_str();
    }else{
      cmd=cmds(s);
    }
    cout <<"COMMAND: '" <<cmd <<"'" <<endl;

    if(cmd=="q") go=false;
    else if(cmd=="u"){ if(node->parent) node = node->parent; }
    else if(cmd=="p") node->solvePoseProblem();
    else if(cmd=="s") node->solveSeqProblem();
    else if(cmd=="x") node->solvePathProblem(20);
    else{
      int choice;
      cmd >>choice;
      cout <<"CHOICE=" <<choice <<endl;
      node = node->children(choice);
      if(!node->isExpanded){
        node->expand();
        if(autoCompute){
          node->solvePoseProblem();
          //          node->solveSeqProblem();
          //          node->solvePathProblem(20);
        }
      }
    }
  }

  mlr::wait();
  threadCloseModules();
  cout <<"BYE BYE" <<endl;
}

//for(uint k=0;k<100;k++){
    //MCTS: add 100 rollouts
    //pick most promising leave
    //compute poses sequentially
    // -> feasible?
    // -> feedback costs to MCTS
//  }

//}

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  orsDrawAlpha = 1.;
  coop();

  return 0;
}
