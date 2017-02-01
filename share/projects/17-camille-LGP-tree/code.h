#include <Core/util.tpp>
#include <Gui/opengl.h>

#include <Motion/motion.h>
//#include <Motion/motionHeuristics.h>
#include <Motion/taskMaps.h>

#include <Motion/komo.h>

#include <Kin/kin_swift.h>
#include <LGP/manipulationTree.h>

#include <Kin/kin.h>
#include <Kin/kinViewer.h>
#include <Gui/opengl.h>
#include <Gui/graphview.h>
#include <Kin/kin.h>
#include <Optim/optimization.h>

#include <LGP/LGP.h>
#include <LGP/manipulationTree.h>

struct Coop{
  mlr::KinematicWorld kin;
  BodyL box;
  mlr::Body *tableC;
  mlr::Body *tableL;
  mlr::Body *tableR;
  mlr::Array<mlr::Transformation> targetAbs, targetRel;

  FOL_World fol;

  ManipulationTree_Node *root,*node;

  OrsPathViewer poseView;
  OrsPathViewer seqView;
  OrsPathViewer pathView;

  bool autoCompute = false;

  mlr::Array<ManipulationTree_Node*> mcFringe;
  mlr::Array<ManipulationTree_Node*> terminals;
  mlr::Array<ManipulationTree_Node*> poseFringe;
  mlr::Array<ManipulationTree_Node*> seqFringe;
  mlr::Array<ManipulationTree_Node*> pathFringe;
  mlr::Array<ManipulationTree_Node*> done;

  Coop();

  void prepareKin();
  void prepareFol(bool smaller=false);
  void prepareTree();
  void prepareDisplay();

  void updateDisplay();
  void displayTree(){ int r=system("evince z.pdf &");  if(r) LOG(-1) <<"could not startup evince"; }

  void printChoices();
  mlr::String queryForChoice();
  bool execChoice(mlr::String cmd);
  bool execRandomChoice();

  void expandNode(){  node->expand(); }
};
