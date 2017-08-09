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

//-----VelocityDirection----------------//

void VelocityDirection::phi( arr& y, arr& J, const mlr::KinematicWorld& G, int t )
{

}

void VelocityDirection::phi(arr& y, arr& J, const WorldL& G, double tau, int t)
{
  auto G0 = G.elem(0);
  auto G1 = G.elem(1);

//  static bool Watch = false;
//  if( ! Watch )
//  {
//    G0->watch();
//    G1->watch();
//    Watch = true;
//  }

  arr tmp_y = zeros( 1 );
  arr tmp_J = zeros( 1, G0->q.N );

  auto body0 = G0->getBodyByName( bobyName_ );

  arr p0, jP0;
  G0->kinematicsPos( p0, jP0, body0, mlr::Vector( 0, 0, 0 ) );

  auto body1 = G1->getBodyByName( bobyName_ );

  arr p1, jP1;
  G1->kinematicsPos( p1, jP1, body1, mlr::Vector( 0, 0, 0 ) );

  arr v =  ( p1 - p0 ) / tau;
  arr Jv = ( jP1 - jP0 ) / tau;

  //std::cout << "v:" << v << std::endl;

  //std::cout << "dir_:" << dir_ << std::endl;

  arr v1, Jv1;
  v1 = normalizedX( v, Jv, Jv1 );

//  static int n = 0;
//  n++;
//  if( n % 10 == 0 )
//  {
//    std::cout << "t:" << t << std::endl;
//    std::cout << "bodyV:" << v << std::endl;
//    std::cout << "bodyV1:" << bodyV1 << std::endl;
//  }

  double dot_product = dot( v1, dir_ );

  double cost = ( 1 - dot_product );

  tmp_y( 0 ) = cost;
  tmp_J.setMatrixBlock( - ( ~ dir_ ) * Jv1, 0, 0 );

  // commit results
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

      mlr::Shape *a = G.shapes(p->a);
      mlr::Shape *b = G.shapes(p->b);

      auto arel=a->body->X.rot/(p->posA-a->body->X.pos);
      auto brel=b->body->X.rot/(p->posB-b->body->X.pos);

      auto arel_2=a->X.rot/(p->posA-a->X.pos);
      auto brel_2=b->X.rot/(p->posB-b->X.pos);

      if( arel != arel_2 || brel != brel_2 )
      {
        int a = 0;
      }

      arr posA;
      arr posB;
      arr JposA;
      arr JposB;
      G.kinematicsPos(posA, JposA, a->body, arel);
      G.kinematicsPos(posB, JposB, b->body, brel);

      double d = norm2( posA - posB );
      arr JnormD = Jnorm( posA - posB );

      if( p->d > 0 )
      {
        const double w = 1;
        tmp_y( 0 ) = w * ( margin - d );
        tmp_J = w * ( - JnormD * ( JposA - JposB ) );
      }
      else
      {
        // collision already!
        int a = 0; a++;
      }

      //std::cout << p->d << " " << d1 << " " << d2 << std::endl;

      break;
    }
  }

  // commit results
  y = tmp_y;
  if(&J) J = tmp_J;
}

