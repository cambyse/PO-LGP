#pragma once

#include <Logic/fol_mcts_world.h>
#include <Kin/kinViewer.h>
#include "polgp_node.h"


class AOSearch
{
public: // public methods
  AOSearch( const KOMOFactory & );

  // modifiers
  void prepareFol( const std::string & folDescription );
  void prepareKin( const std::string & kinDescription );
  void prepareTree();
  void prepareDisplay();

  void solveSymbolically();
  void optimizePoses();
  void optimizeSequences();
  void optimizePaths();
  void optimizeJointPaths();

  void updateDisplay( const WorldID & w, bool poses, bool seqs, bool paths );

private:
  void optimizePoses( POLGPNode * );
  void optimizeSequences( POLGPNode * );

public:

  // getters

  mlr::Array< POLGPNode * > getNodesToExpand() const;
  mlr::Array< POLGPNode * > getNodesToExpand( POLGPNode * ) const;

  mlr::Array< POLGPNode * > getTerminalNodes() const;
  mlr::Array< POLGPNode * > getTerminalNodes( POLGPNode * ) const;

  POLGPNode * getTerminalNode( const WorldID & w ) const;
  POLGPNode * getTerminalNode( POLGPNode *, const WorldID & w ) const;

  bool isSymbolicallySolved() const { return root_->isSymbolicallySolved(); }
  bool isPoseSolved() const { return root_->isPoseSolved(); }
  bool isSequenceSolved() const { return root_->isSequenceSolved(); }
  bool isPathSolved() const { return root_->isPathSolved(); }
  bool isJointPathSolved() const { return root_->isJointPathSolved(); }

  // helpers
  void printPolicy( std::iostream & ) const;

private:
  void printPolicy( POLGPNode * node, std::iostream & ) const;

private:
  //FOL_World fol;      // first order logic symbols
  mlr::Array< std::shared_ptr<FOL_World> > folWorlds_;
  mlr::Array< std::shared_ptr< const mlr::KinematicWorld > > kinematics_;

  arr bs_;
  POLGPNode * root_; // root and "current" node

  const KOMOFactory & komoFactory_;

  // display
  mlr::Array< std::shared_ptr< OrsPathViewer > > poseViews_;
  mlr::Array< std::shared_ptr< OrsPathViewer > > seqViews_;
  mlr::Array< std::shared_ptr< OrsPathViewer > > pathViews_;

  // params
  const mlr::String beliefStateTag_  = "BELIEF_START_STATE";
};

//===========================================================================

//class SearchSpaceTree{
//public: // public members
//  //FOL_World fol;      // first order logic symbols
//  mlr::Array< std::shared_ptr<FOL_World> > folWorlds_;
//  arr bs_;

//  PartiallyObservableNode *root,*node; // root and "current" node

//  OrsPathViewer poseView;
//  OrsPathViewer seqView;
//  OrsPathViewer pathView;

//  mlr::Array<PartiallyObservableNode*> mcFringe;
//  mlr::Array<PartiallyObservableNode*> terminals;
//  mlr::Array<PartiallyObservableNode*> poseFringe;
//  mlr::Array<PartiallyObservableNode*> seqFringe;
//  mlr::Array<PartiallyObservableNode*> pathFringe;
//  mlr::Array<PartiallyObservableNode*> done;

//public: // public methods
//  SearchSpaceTree( const KOMOFactory & );

//  // modifiers
//  void prepareKin( const std::string & kinematicDescription );
//  void prepareFol( const std::string & folDescription );
//  void prepareTree();
//  void prepareDisplay();

//  void updateDisplay();

//  // technically const but let as such, as the semantic is not const
//  bool execRandomChoice();
//  bool execChoice( mlr::String cmd );

//  // const
//  void displayTree() const{ int r=system("evince z.pdf &");  if(r) LOG(-1) <<"could not startup evince"; }
//  void printChoices() const;
//  mlr::String queryForChoice() const;

//private: // private methods

//private: // private members
//  mlr::KinematicWorld kin;

//  KOMOFactory komoFactory_;

//  bool autoCompute = false;
//};

//============free functions==============================================

//double poseHeuristic(PartiallyObservableNode* n);
//double mcHeuristic(PartiallyObservableNode* n);
//double seqHeuristic(PartiallyObservableNode* n);
//double poseCost(PartiallyObservableNode* n);
//double seqCost(PartiallyObservableNode* n);
//double pathHeuristic(PartiallyObservableNode* n);
//double pathCost(PartiallyObservableNode* n);
//PartiallyObservableNode* getBest(mlr::Array<PartiallyObservableNode*>& fringe, double heuristic(PartiallyObservableNode*));
//PartiallyObservableNode* popBest(mlr::Array<PartiallyObservableNode*>& fringe, double heuristic(PartiallyObservableNode*));

//typedef ActionNode MNode;

//double poseHeuristic(MNode* n);
//double mcHeuristic(MNode* n);
//double seqHeuristic(MNode* n);
//double poseCost(MNode* n);
//double seqCost(MNode* n);
//double pathHeuristic(MNode* n);
//double pathCost(MNode* n);
//MNode* getBest(mlr::Array<MNode*>& fringe, double heuristic(MNode*));
//MNode* popBest(mlr::Array<MNode*>& fringe, double heuristic(MNode*));
