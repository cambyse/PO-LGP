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

#include <Core/array.h>

class PolicyNode
{
public:
  typedef std::shared_ptr< PolicyNode > ptr;

public:
  // modifiers
  void setParent( const PolicyNode::ptr & parent ) { parent_ = parent; }
  void addChild( const PolicyNode::ptr & child ) { children_.append( child ); }
  void setState( const mlr::Array< std::shared_ptr<Graph> > & states, const arr & bs ) { states_ = states; bs_ = bs; }
  void setNextAction( const std::string & action ) { nextAction_ = action; }
  void setTime( double t ) { time_ = t; }
  void setId( uint id ) { id_ = id; }
  void setP( double p ) { p_ = p; }
  void setQ( double q ) { q_ = q; }
  void setG( double g ) { g_ = g; }
  void setH( double h ) { h_ = h; }

  // utility
  void setDifferentiatingFact( const std::set< std::string > & facts ) { differentiatingFacts_ = facts; }

  // getters
  PolicyNode::ptr parent() const { return parent_; }
  bool isRoot() const { return parent_ == nullptr; }
  mlr::Array< PolicyNode::ptr > children() const { return children_; }
  mlr::Array< std::shared_ptr<Graph> > states() const { return states_; }
  arr bs() const { return bs_; }
  std::string nextAction() const { return nextAction_; }
  uint N()  const  { return bs_.N; }
  uint id() const  { return id_; }
  double p() const { return p_; }

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
  double g_;  // reward so far
  double h_;  // future rewards

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
  void setStatus( const enum StatusType & status ){ status_ = status; }

  // getter
  uint N() const { return N_; }
  PolicyNode::ptr root() const { return root_; }
  std::list< PolicyNode::ptr > leafs() const { return leafs_; }

private:
  uint N_;
  PolicyNode::ptr root_;
  std::list< PolicyNode::ptr > leafs_;

  enum StatusType status_;
};

class PolicyPrinter
{
public:
  PolicyPrinter( std::ostream & ss )
    : ss_( ss )
  {

  }

  void print( const Policy::ptr & );

private:
  void printFromNode( const PolicyNode::ptr & node );

private:
  std::ostream & ss_;
};

// sort nodes so that the ones with the biggest rewards are first
//struct PolicyCompare : public std::binary_function<Policy::ptr, Policy::ptr, bool>
//{
//  bool operator()( Policy::ptr lhs, Policy::ptr rhs) const
//  {
//    return ! ( lhs->cost() == rhs->cost() ) && ( lhs->cost() < rhs->cost() );
//  }
//};

//class PolicyVisualizer
//{
//public:
//  PolicyVisualizer( const Policy::ptr & policy, const std::string & name );

//private:
//  std::vector< std::shared_ptr< OrsPathViewer > > views_;
//};
