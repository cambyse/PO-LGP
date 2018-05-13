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

#include "new_policy.h"

#include <queue>

static int policyNumber = 0;

//----NewPolicy-------------------------//
NewPolicy::NewPolicy()
  : status_( SKELETON )
  , id_( policyNumber )
{

}

NewPolicy::NewPolicy( const GraphNodeTypePtr & root )
  : status_( SKELETON )
  , id_( policyNumber )
  , root_( root )
{
  policyNumber++;
}

NewPolicy::NewPolicy( const NewPolicy & policy )
{
  copy( policy );
}

NewPolicy & NewPolicy::operator= ( const NewPolicy & policy )
{
  copy( policy );

  return *this;
}

void NewPolicy::save( const std::string & file ) const
{

}

void NewPolicy::copy( const NewPolicy & policy )
{
  if( policy.root_ )
  {
    id_ = policy.id_;
    value_ = policy.value_;
    status_ = policy.status_;

    auto rootData = policy.root_->data();

    root_ = GraphNodeType::root( rootData );

    std::queue< std::pair < GraphNodeTypePtr, GraphNodeTypePtr > > Q;

    Q.push( std::make_pair( policy.root_, root_ ) ); // original - copy

    while( ! Q.empty() )
    {
      auto u = Q.front();
      Q.pop();

      auto uOriginal = u.first;
      auto uCopy     = u.second;

      for( auto v : uOriginal->children() )
      {
        auto vCopy = uCopy->makeChild( v->data() );

        Q.push( std::make_pair( v, vCopy ) );

        if( vCopy->data().terminal )
        {
          leafs_.push_back( vCopy );
        }
      }
    }
  }
}


