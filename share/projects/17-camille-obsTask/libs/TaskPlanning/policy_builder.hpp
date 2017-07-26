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

#include <policy.h>

#include "po_node.h"

namespace tp
{
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

class PolicyBuilder
{
public:
  PolicyBuilder( PONode::ptr root );
  //virtual ~PolicyBuilder();

  Policy::ptr getPolicy() const;

private:
  void process( PONode::ptr node );

private:
  Policy::ptr policy_;
  std::map< PONode::ptr, PolicyNode::ptr > PO2Policy_;
};

}
