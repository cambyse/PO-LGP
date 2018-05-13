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

struct NewPolicyNodeData
{
  std::vector< double      > beliefState;
  std::vector< std::string > leadingKomoArgs;
  bool terminal;
  double markovianReturn;

  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & beliefState;
    ar & leadingKomoArgs;
    ar & terminal;
    ar & markovianReturn;
  }
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

  // io
  void save( const std::string & file ) const;
  void load( const std::string & file );
  void saveToGraphFile( const std::string & file ) const;

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

private:
  void copy( const NewPolicy & );
  //void saveFrom( const PolicyNode::ptr & node, std::ostream& os );

private:
  uint id_;                  // identifier of the policy, meant to be unique
  GraphNodeTypePtr root_;    // start belief state
  std::list< std::weak_ptr< GraphNodeType > > leafs_;     // terminal belief states

  // value
  double value_; // expected cumulated rewards
  enum StatusType status_;
};
