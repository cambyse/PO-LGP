/*  ------------------------------------------------------------------
    Copyright 2016 Camille Phiquepal
    email: camille.phiquepal@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#pragma once

#include <memory>
#include <list>
#include <set>
#include <unordered_set>
#include <map>
#include <memory>

#include <Core/array.h>

#include <graph_node.h>

class MotionPlanningOrder
{
public:
  MotionPlanningOrder( uint policyId )
    : policyId_( policyId )
  {

  }

  uint policyId() const { return policyId_; }
  std::string getParam( const std::string & paramName ) const { CHECK( params_.find( paramName ) != params_.end(), "parameter doesn't exist!" ); return params_.find( paramName )->second; }
  void setParam( const std::string & paramName, const std::string & value ) { params_[ paramName ] = value; }

private:
  std::map< std::string, std::string > params_;  // used to store arbitrary info usefull for the motion palnning of a given policy
  uint policyId_;
};

class PolicyNode : public std::enable_shared_from_this< PolicyNode > // note: public inheritance
{
public:
  typedef std::shared_ptr< PolicyNode > ptr;
  typedef mlr::Array< PolicyNode::ptr > L;

  enum StatusType
  {
    SKELETON = 0,
    INFORMED
  };

public:
  // modifiers
  void setParent( const PolicyNode::ptr & parent ) { parent_ = parent; }
  void addChild( const PolicyNode::ptr & child )   { children_.append( child ); }
  void exchangeChildren( const mlr::Array< PolicyNode::ptr > & children );
  void setState( const mlr::Array< std::shared_ptr<Graph> > & states, const arr & bs ) { states_ = states; bs_ = bs; }
  void setNextAction( const std::string & action ) { nextAction_ = action; }
  void setTime( double t ) { time_ = t; }
  void setId( uint id ) { id_ = id; }
  void setP( double p ) { p_ = p; }
  void setQ( double q ) { q_ = q; }
  void setLastReward( double reward ) { lastReward_ = reward; }
  void setValue( double v ) { value_ = v; }
  void setStatus( const enum StatusType & status ){ status_ = status; }

  // utility
  void setDifferentiatingFact( const std::set< std::string > & facts ) { differentiatingFacts_ = facts; }

  // getters
  PolicyNode::ptr parent() const { return parent_; }
  bool isRoot() const { return parent_ == nullptr; }
  mlr::Array< PolicyNode::ptr > children() const { return children_; }
  mlr::Array< std::shared_ptr<Graph> > states() const { return states_; }
  arr bs() const { return bs_; }
  std::string nextAction()      const { return nextAction_; }
  uint N()  const  { return bs_.N; }
  double time() const { return time_; }
  uint id() const  { return id_; }
  double p() const { return p_; }
  double q() const { return q_; }
  //double prefixReward() const { return prefixReward_; }
  double lastReward() const { return lastReward_; }
  double value() const { return value_; }
  PolicyNode::ptr clone() const;
  void cloneFrom( const PolicyNode::ptr & node ) const;
  StatusType status() const { return status_; }
  // io
//  void save( std::ostream& os );
//  void load( std::istream& is );

  // utility
  std::set< std::string > differentiatingFacts() const { return differentiatingFacts_; }

private:
  PolicyNode::ptr parent_;
  mlr::Array< PolicyNode::ptr > children_;
  // state / belief state
  mlr::Array< std::shared_ptr<Graph> > states_;
  arr bs_;
  // action
  std::string nextAction_; // action to take at this node
  //
  double time_;
  uint id_;

  double p_;  // probability of reaching this node
  double q_;  // probability of reaching this node given that fact that its parent is reached
  double lastReward_;
  double value_; // expected future rewards
  enum StatusType status_;

  // utility
  std::set< std::string > differentiatingFacts_;  ///< used only for debugging purposes
};

class Policy
{
public:
  typedef std::shared_ptr< Policy > ptr;

  enum StatusType
  {
    SKELETON = 0,
    INFORMED
  };

public:
  Policy();

  // modifier
  void init( uint N );
  void setRoot( const PolicyNode::ptr & root )    { root_ = root; }
  void addLeaf( const PolicyNode::ptr & leaf )    { leafs_.append( leaf ); }
  void resetLeafs()    { leafs_.clear(); }
  void setValue( double v )           { value_ = v; }
  void setStatus( const enum StatusType & status ){ status_ = status; }

  // getter
  uint id() const { return id_; }
  uint N()  const { return N_; }
  PolicyNode::ptr root() const { return root_; }
  PolicyNode::L  leafs() const { return leafs_; }
  double value() const { return value_; }
  bool feasible()        const { return value_ > - std::numeric_limits< double >::infinity(); }
  Policy::ptr clone() const;

  // io
  void save( std::ostream& os );
  void load( std::istream& is );

private:
  void saveFrom( const PolicyNode::ptr & node, std::ostream& os );

private:
  uint id_;                 // identifier of the policy, meant to be unique
  uint N_;                  // size of the belief state
  PolicyNode::ptr root_;    // start belief state
  PolicyNode::L leafs_;     // terminal belief states

  // value
  double value_; // expected cumulated rewards
  enum StatusType status_;
};

//class NewPolicyNodeData
//{
//public:

//  enum StatusType
//  {
//    SKELETON = 0,
//    INFORMED
//  };

//public:
//  // modifiers
//  //void setNextAction( const std::string & action ) { nextAction_ = action; }
//  void setTime( double t ) { time_ = t; }
//  void setP( double p ) { p_ = p; }
//  void setQ( double q ) { q_ = q; }
//  void setLastReward( double reward ) { lastReward_ = reward; }
//  void setValue( double v ) { value_ = v; }
//  void setStatus( const enum StatusType & status ){ status_ = status; }

//  // utility
//  void setDifferentiatingFact( const std::set< std::string > & facts ) { differentiatingFacts_ = facts; }

//  // getters
//  arr bs() const { return bs_; }
//  //std::string nextAction()      const { return nextAction_; }
//  uint N()  const  { return bs_.N; }
//  double time() const { return time_; }
//  double p() const { return p_; }
//  double q() const { return q_; }
//  //double prefixReward() const { return prefixReward_; }
//  double lastReward() const { return lastReward_; }
//  double value() const { return value_; }
//  PolicyNode::ptr clone() const;
//  void cloneFrom( const PolicyNode::ptr & node ) const;
//  StatusType status() const { return status_; }

//  // utility
//  std::set< std::string > differentiatingFacts() const { return differentiatingFacts_; }

//private:
//  // state / belief state
//  mlr::Array< std::shared_ptr<Graph> > states_;
//  arr bs_;
//  // action
//  //std::string nextAction_; // action to take at this node
//  //
//  double time_;

//  double p_;  // probability of reaching this node
//  double q_;  // probability of reaching this node given that fact that its parent is reached
//  double lastReward_;
//  double value_; // expected future rewards
//  enum StatusType status_;

//  // utility
//  std::set< std::string > differentiatingFacts_;  ///< used only for debugging purposes
//};

struct NewPolicyNodeData
{
  std::vector< double      > beliefState;
  std::string komoTag;
  bool terminal;
  double startTime;
  double endTime;
  double markovianReturn;
};

class NewPolicy
{
public:
  using ptr = std::shared_ptr< GraphNode< NewPolicyNodeData > >;
  using GraphNodeType = GraphNode< NewPolicyNodeData >;
  using GraphNodeTypePtr = std::shared_ptr< GraphNode< NewPolicyNodeData > >;

  enum StatusType
  {
    SKELETON = 0,
    INFORMED
  };

public:
  NewPolicy();
  NewPolicy( const GraphNodeTypePtr & root );

  NewPolicy( const NewPolicy & );
  NewPolicy & operator= ( const NewPolicy & );


  // modifier
  //void init( uint N );
  //void setRoot( const GraphNodeTypePtr & root )    { root_ = root; }
  void addLeaf( const GraphNodeTypePtr & leaf )    { leafs_.push_back( leaf ); }
  void resetLeafs()    { leafs_.clear(); }
  //void setValue( double v )           { value_ = v; }
  void setStatus( const enum StatusType & status ){ status_ = status; }

  // getter
  bool empty() const { return root_ == nullptr; }
  uint id() const { return id_; }
  //uint N()  const { return N_; }
  GraphNodeTypePtr root() const { return root_; }
  std::list< std::weak_ptr< GraphNodeType > >  leafs() const { return leafs_; }
  double value() const { return value_; }
  bool feasible()        const { return value_ > - std::numeric_limits< double >::infinity(); }
  //NewPolicy::ptr clone() const;

private:
  void copy( const NewPolicy & );
  //void saveFrom( const PolicyNode::ptr & node, std::ostream& os );

private:
  uint id_;                  // identifier of the policy, meant to be unique
  //uint N_;                 // size of the belief state
  GraphNodeTypePtr root_;    // start belief state
  std::list< std::weak_ptr< GraphNodeType > > leafs_;     // terminal belief states

  // value
  double value_; // expected cumulated rewards
  enum StatusType status_;
};

// utility free functions
PolicyNode::L getPathTo( const PolicyNode::ptr & node );

// sort nodes so that the ones with the biggest rewards are first
bool policyCompare( Policy::ptr lhs, Policy::ptr rhs );

// test if two skeletons are equals
bool skeletonEquals( Policy::ptr lhs, Policy::ptr rhs );

// return a list from a tree
std::list< PolicyNode::ptr > serialize( const Policy::ptr & policy );

// return a list of segments from a tree
std::vector< std::list< PolicyNode::ptr > > segment( const Policy::ptr & policy );

// the series contain the same nodes
bool equivalent( const std::list< PolicyNode::ptr > & s1, const std::list< PolicyNode::ptr > & s2 );

// joint two policies
Policy::ptr fuse( Policy::ptr base, Policy::ptr over );



