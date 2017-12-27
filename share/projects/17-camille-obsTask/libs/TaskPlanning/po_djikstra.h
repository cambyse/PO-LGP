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

#include "po_graph.h"
#include "policy.h"

namespace tp
{
class Dijkstra
{
public:
  Dijkstra( const mlr::Array< std::shared_ptr<FOL_World> > & folEngines );

  Policy::ptr solve( const POGraph::ptr & graph, const POGraphNode::ptr & from, GraphEdgeRewards::ptr = nullptr );

private:
  void dijkstra( const std::list < POGraphNode::ptr > & terminals, const GraphEdgeRewards::ptr & mask );
  bool extractSolutionFrom( const POGraphNode::ptr &, const GraphEdgeRewards::ptr & mask );
  bool buildPolicy( const POGraphNode::ptr & );
  bool buildPolicyFrom( const POGraphNode::ptr & node, const POGraphNode::ptr & start );

private:
  mlr::Array< std::shared_ptr< FOL_World > > folEngines_;
  std::vector< double > values_;
  // policy reconstruction
  POGraph::ptr graph_;
  std::vector< int >   bestFamily_;     // action to take in this bs and i
  std::vector< POGraphNode::ptr > parents_;
  Policy::ptr policy_;
  std::map< POGraphNode::ptr, PolicyNode::ptr > PO2Policy_;
  std::map< PolicyNode::ptr, POGraphNode::ptr > Policy2PO_;
};
}
