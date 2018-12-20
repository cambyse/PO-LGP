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

#include <Core/array.h>

#include <graph_node.h>

#include <boost/serialization/vector.hpp>

class MotionPlanningParameters
{
public:
  MotionPlanningParameters( uint policyId )
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

struct SkeletonNodeData
{
  std::vector< double      > beliefState;
  std::vector< std::string > leadingKomoArgs;
  double markovianReturn = 0;
  double p = 0; // probability to reach this node given the parent
  uint decisionGraphNodeId = 0; // id of the corresponding node in decision graph

  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & beliefState;
    ar & leadingKomoArgs;
    ar & markovianReturn;
    ar & p;
    ar & decisionGraphNodeId;
  }
};

class Skeleton
{
public:
  using ptr = std::shared_ptr< GraphNode< SkeletonNodeData > >;
  using GraphNodeType = GraphNode< SkeletonNodeData >;
  using GraphNodeTypePtr = std::shared_ptr< GraphNode< SkeletonNodeData > >;

  enum StatusType
  {
    SKELETON = 0,
    INFORMED
  };

public:
  Skeleton();
  Skeleton( const GraphNodeTypePtr & root );

  Skeleton( const Skeleton & );
  Skeleton & operator= ( const Skeleton & );

  // modifier
  void addLeaf( const GraphNodeTypePtr & leaf )    { leafs_.push_back( leaf ); }
  void resetLeafs()    { leafs_.clear(); }
  void setValue( double v )           { value_ = v; }
  void setStatus( const enum StatusType & status ){ status_ = status; }

  // getter
  bool empty() const { return root_ == nullptr; }
  uint id() const { return id_; }
  GraphNodeTypePtr root() const { return root_; }
  std::list< std::weak_ptr< GraphNodeType > >  leafs() const { return leafs_; }
  double value() const { return value_; }
  enum StatusType status() const { return status_; }
  bool feasible()        const { return value_ > - std::numeric_limits< double >::infinity(); }

  uint N() const { return root_->data().beliefState.size(); }

  // io
  void save( const std::string & file ) const;
  void load( const std::string & file );
  void saveToGraphFile( const std::string & file ) const;
  void saveAll( const std::string & suffix = "" ) const;

  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & id_;
    ar & root_;
    ar & leafs_;
    ar & value_;
    ar & status_;
  }

  size_t hash() const;

private:
  void copy( const Skeleton & );
  //void saveFrom( const PolicyNode::ptr & node, std::ostream& os );

private:
  uint id_;                  // identifier of the policy, meant to be unique
  GraphNodeTypePtr root_;    // start belief state
  std::list< std::weak_ptr< GraphNodeType > > leafs_;     // terminal belief states

  // value
  double value_; // expected cumulated rewards
  enum StatusType status_;
};

bool operator== ( const Skeleton & a, const Skeleton & b );
bool operator!= ( const Skeleton & a, const Skeleton & b );

std::list< Skeleton::GraphNodeTypePtr > getPathTo( const Skeleton::GraphNodeTypePtr & node );

struct SkeletonHasher
{
    std::size_t operator()( const Skeleton & s ) const noexcept
    {
        return s.hash();
    }
};
