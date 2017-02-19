#pragma once

#include <Logic/fol_mcts_world.h>
#include <Kin/kinViewer.h>
#include "partially_observable_node.h"

class SearchSpaceTree{
public: // public members
  //FOL_World fol;      // first order logic symbols
  mlr::Array< std::shared_ptr<FOL_World> > folWorlds_;
  arr bs_;

  PartiallyObservableNode *root,*node; // root and "current" node

  OrsPathViewer poseView;
  OrsPathViewer seqView;
  OrsPathViewer pathView;

  mlr::Array<PartiallyObservableNode*> mcFringe;
  mlr::Array<PartiallyObservableNode*> terminals;
  mlr::Array<PartiallyObservableNode*> poseFringe;
  mlr::Array<PartiallyObservableNode*> seqFringe;
  mlr::Array<PartiallyObservableNode*> pathFringe;
  mlr::Array<PartiallyObservableNode*> done;

public: // public methods
  SearchSpaceTree( const KOMOFactory & );

  // modifiers
  void prepareKin( const std::string & kinematicDescription );
  void prepareFol( const std::string & folDescription );
  void prepareTree();
  void prepareDisplay();

  void updateDisplay();

  // technically const but let as such, as the semantic is not const
  bool execRandomChoice();
  void expandNode() { node->expand(); }
  bool execChoice( mlr::String cmd );

  // const
  void displayTree() const{ int r=system("evince z.pdf &");  if(r) LOG(-1) <<"could not startup evince"; }
  void printChoices() const;
  mlr::String queryForChoice() const;

private: // private methods

private: // private members
  mlr::KinematicWorld kin;

  KOMOFactory komoFactory_;

  bool autoCompute = false;
};

//============free functions==============================================

double poseHeuristic(PartiallyObservableNode* n);
double mcHeuristic(PartiallyObservableNode* n);
double seqHeuristic(PartiallyObservableNode* n);
double poseCost(PartiallyObservableNode* n);
double seqCost(PartiallyObservableNode* n);
double pathHeuristic(PartiallyObservableNode* n);
double pathCost(PartiallyObservableNode* n);
PartiallyObservableNode* getBest(mlr::Array<PartiallyObservableNode*>& fringe, double heuristic(PartiallyObservableNode*));
PartiallyObservableNode* popBest(mlr::Array<PartiallyObservableNode*>& fringe, double heuristic(PartiallyObservableNode*));

typedef ActionNode MNode;

double poseHeuristic(MNode* n);
double mcHeuristic(MNode* n);
double seqHeuristic(MNode* n);
double poseCost(MNode* n);
double seqCost(MNode* n);
double pathHeuristic(MNode* n);
double pathCost(MNode* n);
MNode* getBest(mlr::Array<MNode*>& fringe, double heuristic(MNode*));
MNode* popBest(mlr::Array<MNode*>& fringe, double heuristic(MNode*));
