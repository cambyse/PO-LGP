#pragma once

#include <Logic/fol_mcts_world.h>
#include <Kin/kinViewer.h>
#include "action_node.h"

class SearchSpaceTree{
public: // public members
  FOL_World fol;      // first order logic symbols

  ActionNode *root,*node; // root and "current" node

  OrsPathViewer poseView;
  OrsPathViewer seqView;
  OrsPathViewer pathView;

  mlr::Array<ActionNode*> mcFringe;
  mlr::Array<ActionNode*> terminals;
  mlr::Array<ActionNode*> poseFringe;
  mlr::Array<ActionNode*> seqFringe;
  mlr::Array<ActionNode*> pathFringe;
  mlr::Array<ActionNode*> done;

public: // public methods
  SearchSpaceTree();

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

//  BodyL box;
//  mlr::Body *tableC;
//  mlr::Body *tableL;
//  mlr::Body *tableR;

//  mlr::Array<mlr::Transformation> targetAbs, targetRel;

  bool autoCompute = false;
};

//============free functions==============================================

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
