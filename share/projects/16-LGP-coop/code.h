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

struct Coop{
  ors::KinematicWorld kin;
  BodyL box;
  ors::Body *tableC;
  ors::Body *tableL;
  ors::Body *tableR;
  mlr::Array<ors::Transformation> targetAbs, targetRel;

  FOL_World fol;

  ManipulationTree_Node *root,*node;

  OrsViewer poseView;
  OrsPathViewer seqView;
  OrsPathViewer pathView;

  bool autoCompute = false;

  Coop();

  void prepareKin();
  void prepareFol();
  void prepareTree();
  void prepareDisplay();

  void updateDisplay();
  void displayTree(){ system("evince z.pdf &"); }

  void printChoices();
  mlr::String queryForChoice();
  bool execChoice(mlr::String cmd);
  bool execRandomChoice();

  void expandNode(){  node->expand(); }
};
