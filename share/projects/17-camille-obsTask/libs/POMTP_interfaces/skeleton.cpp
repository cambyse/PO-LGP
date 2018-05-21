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

#include "skeleton.h"

#include <queue>

#include <skeleton_printer.h>

static int skeletonNumber = 0;

//----Skeleton-------------------------//
Skeleton::Skeleton()
  : status_( SKELETON )
  , id_( skeletonNumber )
{

}

Skeleton::Skeleton( const GraphNodeTypePtr & root )
  : status_( SKELETON )
  , id_( skeletonNumber )
  , root_( root )
{
  skeletonNumber++;

  // reconstruct the leafs from root
  std::list < GraphNodeTypePtr > Q;
  Q.push_back( root );

  while ( ! Q.empty() )
  {
    auto n = Q.front();
    Q.pop_front();

    if( n->children().size() == 0 )
    {
      leafs_.push_back( n );
    }
    else
    {
      for( auto c : n->children() )
      {
        Q.push_back( c );
      }
    }
  }
}

Skeleton::Skeleton( const Skeleton & policy )
{
  copy( policy );
}

Skeleton & Skeleton::operator= ( const Skeleton & policy )
{
  copy( policy );

  return *this;
}

void Skeleton::save( const std::string & file ) const
{
  std::ofstream ofs( file );
  boost::archive::text_oarchive oa(ofs);
  oa << *this;
}

void Skeleton::load( const std::string & file )
{
  std::ifstream ifs( file );
  boost::archive::text_iarchive ia(ifs);
  ia >> *this;
}

void Skeleton::saveToGraphFile( const std::string & filename ) const
{
  if( ! root_ )
  {
    return;
  }

  std::ofstream file;
  file.open( filename );

  SkeletonPrinter printer( file );
  printer.print( *this );

  file.close();

  // png
  std::string nameCopy( filename );
  const std::string ext( ".gv" );
  std::string newName = nameCopy.replace( nameCopy.find( ext ), ext.length(), ".png" );

  std::stringstream ss;
  ss << "dot"   << " ";
  ss << "-Tpng" << " ";
  ss << "-o"    << " ";
  ss << newName << " ";
  ss << filename;

  system( ss.str().c_str() );
}

void Skeleton::copy( const Skeleton & policy )
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

        if( v->children().size() == 0 )
        {
          leafs_.push_back( vCopy );
        }
      }
    }
  }
}

std::list< Skeleton::GraphNodeTypePtr > getPathTo( const Skeleton::GraphNodeTypePtr & node )
{
  std::list< Skeleton::GraphNodeTypePtr > path;

  auto n = node;

  path.push_front( n );

  while( n->parent() )
  {
    path.push_front( n->parent() );
    n = n->parent();
  }

  return path;
}
