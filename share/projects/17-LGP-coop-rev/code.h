#include <Core/util.tpp>
#include <Gui/opengl.h>

#include <Kin/taskMaps.h>
#include <KOMO/komo.h>

#include <Kin/kin_swift.h>
#include <LGP/manipulationTree.h>
#include <MCTS/solver_AStar.h>

#include <Kin/kin.h>
#include <Kin/kinViewer.h>
#include <Gui/opengl.h>
#include <Gui/graphview.h>
#include <Kin/kin.h>
#include <Optim/optimization.h>

#include <LGP/LGP.h>
#include <LGP/manipulationTree.h>

#include <Algo/priorityQueue.h>

struct Coop{
  mlr::KinematicWorld kin;
  BodyL box;
  mlr::Body *tableC;
  mlr::Body *tableL;
  mlr::Body *tableR;
  mlr::Array<mlr::Transformation> targetAbs, targetRel;

  FOL_World fol;

  ManipulationTree_Node *root, *node;

  OrsPathViewer poseView;
  OrsPathViewer seqView;
  OrsPathViewer pathView;

  bool autoCompute = false;

  //-- these are lists or queues; I don't maintain them sorted because their evaluation (e.g. f(n)=g(n)+h(n)) changes continuously
  // while new bounds are computed. Therefore, whenever I pop from these lists, I find the minimum w.r.t. a heuristic. The
  // heuristics are defined in main.cpp currently
  mlr::Array<ManipulationTree_Node*> expandFringe;       //list of nodes to be expanded next
  mlr::Array<ManipulationTree_Node*> terminals;   //list of found terminals
  mlr::Array<ManipulationTree_Node*> poseFringe;  //list of nodes that can be pose tested (parent has been tested)
  mlr::Array<ManipulationTree_Node*> pose2Fringe; //list of nodes towards a terminal -> scheduled for pose testing
  mlr::Array<ManipulationTree_Node*> seqFringe;   //list of terminal nodes that have been pose tested
  mlr::Array<ManipulationTree_Node*> pathFringe;  //list of terminal nodes that have been seq tested
  mlr::Array<ManipulationTree_Node*> done;        //list of terminal nodes that have been path tested

  Coop();

  void prepareKin();
  void prepareFol(bool smaller=false);
  void prepareTree();
  void prepareAStar();
  void prepareDisplay();

  void updateDisplay();
  void displayTree(){ int r=system("evince z.pdf &");  if(r) LOG(-1) <<"could not startup evince"; }

  void printChoices();
  mlr::String queryForChoice();
  bool execChoice(mlr::String cmd);
  bool execRandomChoice();

  void expandNode(){  node->expand(); }
};
