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

#include <object_pair_collision_avoidance.h>

//-----VerticalVelocity----------------//

void VerticalVelocity::phi( arr& y, arr& J, const mlr::KinematicWorld& G, int t )
{
  arr tmp_y = zeros( 2 );
  arr tmp_J = zeros( 2, G.q.N );

  auto body = G.getBodyByName( bobyName_ );
  arr p, Jp;
  G.kinematicsPos( p, Jp, body, mlr::Vector( 0, 0, 0 ) );


  // commit results
  const double w = 10;
  tmp_y( 0 ) = w * p( 0 );
  tmp_y( 1 ) = w * p( 1 );

  tmp_J.setMatrixBlock( w * Jp.row( 0 ), 0, 0 );
  tmp_J.setMatrixBlock( w * Jp.row( 1 ), 1, 0 );


  //tmp_J.setMatrixBlock( Jp, 0, 0 );

  //  // commit results
  y = tmp_y;
  if(&J) J = tmp_J;
}

//-----AxisAlignment----------------//

void AxisAlignment::phi( arr& y, arr& J, const mlr::KinematicWorld& G, int t )
{
  arr tmp_y = zeros( 1 );
  arr tmp_J = zeros( 1, G.q.N );

  auto body = G.getBodyByName( bobyName_ );

  arr bodyAxisDirection, bodyJAxisDirection;
  G.kinematicsVec( bodyAxisDirection, bodyJAxisDirection, body, axis_ );

  double dot_product = dot( bodyAxisDirection, axis_ );

  double cost = 1 - dot_product;

  tmp_y( 0 ) = cost;
  tmp_J.setMatrixBlock( - ( ~ axis_ ) * bodyJAxisDirection, 0, 0 );

  // commit results
  y = tmp_y;
  if(&J) J = tmp_J;
}

//-------ShapePairCollisionConstraint----------------//

void ShapePairCollisionConstraint::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t)
{
  arr tmp_y = zeros( 1 );
  arr tmp_J = zeros( 1, G.q.N );

  //std::cout << G.shapes( i_ )->name << "-" << G.shapes( j_ )->name << std::endl;

//  for( auto j : G.joints )
//  {
//    if( G.shapes( i_ )->name == j->from->name && G.shapes( j_ )->name == j->to->name
//        ||
//        G.shapes( i_ )->name == j->to->name && G.shapes( j_ )->name == j->from->name
//      )
//    {
//      std::cout << "pb!!" << std::endl;
//    }
//    //std::cout << "joint:" << j->from->name << " " << j->to->name << std::endl;
//  }

  for( mlr::Proxy *p: G.proxies )
  {
//    if( G.shapes( p->a )->name == "handR" && G.shapes( p->b )->name == "tableC"
//        ||
//        G.shapes( p->b )->name == "handR" && G.shapes( p->a )->name == "tableC"
//      )
//    {
//      std::cout << "ici" << std::endl;
//    }
    //std::cout << "proxy:" << G.shapes( p->a )->name << "-" << G.shapes( p->b )->name << std::endl;

    if((p->a==i_ && p->b==j_) || (p->a==j_ && p->b==i_))
    {
      //std::cout << "active proxy:" << G.shapes( p->a )->name << "-" << G.shapes( p->b )->name << std::endl;
      if( p->d > 0 )
      {
        phiNoCollision( tmp_y, tmp_J, G, p );
      }
      else
      {
        // collision already!
        phiCollision( tmp_y, tmp_J, G, p );
      }
      break;
    }
  }

  // commit results
  y = tmp_y;
  if(&J) J = tmp_J;
}

void ShapePairCollisionConstraint::phiNoCollision( arr& y, arr& J, const mlr::KinematicWorld& G, mlr::Proxy * p )
{
  mlr::Shape *a = G.shapes(p->a);
  mlr::Shape *b = G.shapes(p->b);

  auto arel=a->body->X.rot/(p->posA-a->body->X.pos);
  auto brel=b->body->X.rot/(p->posB-b->body->X.pos);

  arr posA;
  arr posB;
  arr JposA;
  arr JposB;
  G.kinematicsPos(posA, JposA, a->body, arel);
  G.kinematicsPos(posB, JposB, b->body, brel);

  double d = norm2( posA - posB );
  arr JnormD = Jnorm( posA - posB );

  const double w = 10;
  y( 0 ) = w * ( margin - d );
  J = w * ( - JnormD * ( JposA - JposB ) );
}

void ShapePairCollisionConstraint::phiCollision( arr& y, arr& J, const mlr::KinematicWorld& G, mlr::Proxy * p )
{
//  mlr::Shape *a = G.shapes(p->a);
//  mlr::Shape *b = G.shapes(p->b);

//  auto arel=a->body->X.rot/(a->X.pos-a->body->X.pos);
//  auto brel=b->body->X.rot/(b->X.pos-b->body->X.pos);

//  arr posA;
//  arr posB;
//  arr JposA;
//  arr JposB;
//  G.kinematicsPos(posA, JposA, a->body, arel);
//  G.kinematicsPos(posB, JposB, b->body, brel);

//  double d = norm2( posA - posB );
//  arr JnormD = Jnorm( posA - posB );

//  const double w = 10;
//  y( 0 ) = w * ( margin /*- d*/ );
//  J = w * ( - JnormD * ( JposA - JposB ) );
}

