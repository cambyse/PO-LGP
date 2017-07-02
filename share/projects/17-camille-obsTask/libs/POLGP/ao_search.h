#pragma once

#include <list>
#include <set>

#include <Logic/fol_mcts_world.h>
#include <Kin/kinViewer.h>
#include "polgp_node.h"
#include "policy.hpp"

// sort nodes so that the ones with the biggest rewards are first
//struct POLGPNodeCompare : public std::binary_function<POLGPNode*, POLGPNode*, bool>
//{
//    bool operator()( POLGPNode* lhs, POLGPNode* rhs) const
//    {
//        return ! ( lhs->expecteTotalReward() == rhs->expecteTotalReward() ) && ( lhs->expecteTotalReward() > rhs->expecteTotalReward() );
//    }
//};

class AOSearch
{
public: // public methods
  AOSearch( const KOMOFactory & );

  // modifiers
  void prepareFol( const std::string & folDescription );
  void prepareKin( const std::string & kinDescription );
  void prepareTree();
  void prepareDisplay();

  //void registerGeometricLevel( GeometricLevelFactoryBase::ptr const& factory );

  void solveSymbolically();
  bool isPolicyFringeEmpty() const { return currentPolicyFringeInitialized_ && ( currentPolicyFringe_.size() == 0 ); }
  void generateAlternativeSymbolicPolicy();
  void revertToPreviousPolicy();

  void solveGeometrically();

  void optimizePoses();
  void optimizePaths();
  void optimizeJointPaths();

  void updateDisplay( const WorldID & w, bool poses, bool seqs, bool paths );

private:
  void optimizePosesFrom( POLGPNode * );

public:
  // getters
  bool isSymbolicallySolved() const { return root_->isSymbolicallySolved(); }
  bool isPoseSolved() const { return root_->isPoseSolved(); }
  bool isPathSolved() const { return root_->isPathSolved(); }
  bool isJointPathSolved() const { return root_->isJointPathSolved(); }

  uint alternativeNumber() const { return alternativeNumber_; }

  // helpers
  Policy::ptr getPolicy() const;

  void printPolicy( const std::string & name, bool generatePng = true ) const;
  void printSearchTree( const std::string & name, bool generatePng = true ) const;
  void printPolicy( std::iostream & ) const;
  void printSearchTree( std::iostream & ) const;

private:
  mlr::Array< POLGPNode * > getNodesToExpand() const;   // go along the best solution so far and accumulates the nodes that haven't been expanded, it goes up to the "deepest nodes" of the temporary path
  mlr::Array< POLGPNode * > getNodesToExpand( POLGPNode * ) const;

  mlr::Array< POLGPNode * > getTerminalNodes() const;
  mlr::Array< POLGPNode * > getTerminalNodes( POLGPNode * ) const;

  POLGPNode * getTerminalNode( const WorldID & w ) const;

  void printPolicy( POLGPNode * node, std::iostream & ) const;
  void printSearchTree( POLGPNode * node, std::iostream & ss ) const;

private:
  //FOL_World fol;      // first order logic symbols
  mlr::Array< std::shared_ptr<FOL_World> > folWorlds_;
  mlr::Array< std::shared_ptr< const mlr::KinematicWorld > > kinematics_;

  arr bs_;
  POLGPNode * root_; // root and "current" node

  //
  //std::set< POLGPNode *, POLGPNodeCompare > openFringe_;        // fringe of the current search tree
  //std::set< POLGPNode *, POLGPNodeCompare > alternativeNodes_;  // when looking for alternatives, use the node in this set, when it becomes empty, backupthe current open fringe
  std::set< POLGPNodeL > currentPolicyFringe_;
  bool currentPolicyFringeInitialized_;
  uint alternativeNumber_;
  POLGPNode * alternativeStartNode_;
  POLGPNodeL nextFamilyBackup_;
  std::set< POLGPNodeL > currentPolicyFringeBackup_;  
  //

  // geometric levels
  const KOMOFactory & komoFactory_;
  mlr::Array< GeometricLevelFactoryBase::ptr > geometricLevelFactories_;

  // display
  mlr::Array< std::shared_ptr< OrsPathViewer > > poseViews_;
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
