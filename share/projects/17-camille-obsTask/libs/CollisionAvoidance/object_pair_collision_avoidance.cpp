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
#include <Geo/mesh.h>

#include <fcl/traversal/traversal_node_bvhs.h>
#include <fcl/collision_node.h>
#include <fcl/collision.h>
#include <fcl/distance.h>

using namespace fcl;

static CollisionObject * createObjectModel( mlr::Shape * s );

//-------ShapePairCollisionConstraint----------------//

void ShapePairCollisionConstraint::phi( arr& y, arr& J, const mlr::KinematicWorld& G, int t )
{
  arr tmp_y = zeros( 1 );
  arr tmp_J = zeros( 1, G.q.N );

  for( mlr::Proxy *p: G.proxies )
  {
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
        phiCollision( tmp_y, tmp_J, G );
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
  y( 0 ) = w * ( margin_ - d );
  J = w * ( - JnormD * ( JposA - JposB ) );
}

void ShapePairCollisionConstraint::phiCollision( arr& y, arr& J, const mlr::KinematicWorld& G )
{
  mlr::Shape *s1 = i_<0?NULL: G.shapes(i_);
  mlr::Shape *s2 = j_<0?NULL: G.shapes(j_);
  CHECK(s1 && s2,"");
  CHECK(s1->mesh.V.N,"");
  CHECK(s2->mesh.V.N,"");

  auto m1 = createObjectModel( s1 );
  auto m2 = createObjectModel( s2 );

  // set the distance request structure, here we just use the default setting
  DistanceRequest request;
  request.enable_nearest_points = true;
  // result will be returned via the collision result structure
  DistanceResult result;
  // perform distance test
  distance(m1, m2, request, result);

  auto p1 = result.nearest_points[ 0 ];
  auto p2 = result.nearest_points[ 1 ];

  mlr::Vector pposA( p1[ 0 ], p1[ 2 ], p1[ 2 ] );
  mlr::Vector pposB( p2[ 0 ], p2[ 2 ], p2[ 2 ] );

  /////////
  auto a = s1;
  auto b = s2;
  auto arel=a->body->X.rot/(pposA-a->body->X.pos);
  auto brel=b->body->X.rot/(pposB-b->body->X.pos);

  arr posA;
  arr posB;
  arr JposA;
  arr JposB;
  G.kinematicsPos(posA, JposA, a->body, arel);
  G.kinematicsPos(posB, JposB, b->body, brel);

  double d = norm2( posA - posB );
  arr JnormD = Jnorm( posA - posB );

  const double w = 10;
  y( 0 ) = w * ( margin_ + d );
  J = w * ( JnormD * ( JposA - JposB ) );
}

static CollisionObject * createObjectModel( mlr::Shape * s )
{
  CHECK( s, "" );
  CHECK( s->mesh.V.N, "" );

  /// 1 - set mesh triangles and vertice indices
  std::vector<Vec3f> vertices;
  std::vector<Triangle> triangles;

  // code to set the vertices and triangles
  for( uint i = 0; i < s->mesh.V.d0; i++ )
  {
    vertices.push_back( Vec3f( s->mesh.V( i, 0 ), s->mesh.V( i, 1 ), s->mesh.V( i, 2 ) ) );
  }

  for( uint i = 0; i < s->mesh.T.d0; i++ )
  {
    triangles.push_back( Triangle( s->mesh.T( i, 0 ), s->mesh.T( i, 1 ), s->mesh.T( i, 2 ) ) );
  }

  auto model = std::make_shared< BVHModel<OBBRSS> >();
  // add the mesh data into the BVHModel structure
  model->beginModel();
  model->addSubModel(vertices, triangles);
  model->endModel();

  /// 2 - Set transforms
  Matrix3f R;
  Vec3f T;
  // code for setting R and T
  T = Vec3f( s->X.pos.x,  s->X.pos.y,  s->X.pos.z );

  auto m = s->X.rot.getMatrix();
  R = Matrix3f( m.m00, m.m01, m.m02,
                m.m10, m.m11, m.m12,
                m.m20, m.m21, m.m22 );

  // transform is configured according to R and T
  Transform3f pose( R, T );

  /// 3 - Combine them together
  CollisionObject * obj = new CollisionObject( model, pose );

  return obj;
}

//void ShapePairCollisionConstraint::phiCollision( arr& v, arr& J, const mlr::KinematicWorld& W, mlr::Proxy * p )
//{
//  bool exact = false;
//  bool negScalar = true;

//  mlr::Shape *s1 = i<0?NULL: W.shapes(i);
//  mlr::Shape *s2 = j<0?NULL: W.shapes(j);
//  CHECK(s1 && s2,"");
//  CHECK(s1->sscCore.V.N,"");
//  CHECK(s2->sscCore.V.N,"");
//  mlr::Vector p1, p2, e1, e2;
//  GJK_point_type pt1, pt2;

//  GJK_sqrDistance(s1->sscCore, s2->sscCore, s1->X, s2->X, p1, p2, e1, e2, pt1, pt2);
//  //  if(d2<1e-10) LOG(-1) <<"zero distance";
//  arr y1, J1, y2, J2;

//  W.kinematicsPos(y1, (&J?J1:NoArr), s1->body, s1->body->X.rot/(p1-s1->body->X.pos));
//  W.kinematicsPos(y2, (&J?J2:NoArr), s2->body, s2->body->X.rot/(p2-s2->body->X.pos));

//  //std::cout << "J1:" << J1 << std::endl;
//  //std::cout << "J2:" << J2 << std::endl;

//  v = y1 - y2;
//  if(&J){
//    J = J1 - J2;
//    //std::cout << "J:" << J << std::endl;

//    if(exact){
//      if((pt1==GJK_vertex && pt2==GJK_face) || (pt1==GJK_face && pt2==GJK_vertex)){
//        arr vec, Jv, n = v/length(v);
//        J = n*(~n*J);
//        if(pt1==GJK_vertex) W.kinematicsVec(vec, Jv, s2->body, s2->body->X.rot/(p1-p2));
//        if(pt2==GJK_vertex) W.kinematicsVec(vec, Jv, s1->body, s1->body->X.rot/(p1-p2));
//        J += Jv;
//      }
//      if(pt1==GJK_edge && pt2==GJK_edge){
//        arr vec, Jv, n, a, b;
//        n = v/length(v);
//        J = n*(~n*J);

//        W.kinematicsVec(vec, Jv, s1->body, s1->body->X.rot/e1);
//        a=conv_vec2arr(e1);
//        b=conv_vec2arr(e2);
//        double ab=scalarProduct(a,b);
//        J += (a-b*ab) * (1./(1.-ab*ab)) * (~v*(b*~b -eye(3,3))) * Jv;

//        W.kinematicsVec(vec, Jv, s2->body, s2->body->X.rot/e2);
//        a=conv_vec2arr(e2);
//        b=conv_vec2arr(e1);
//        J += (a-b*ab) * (1./(1.-ab*ab)) * (~v*(b*~b -eye(3,3))) * Jv;
//      }
//      if((pt1==GJK_vertex && pt2==GJK_edge) || (pt1==GJK_edge && pt2==GJK_vertex)){
//        arr vec, Jv, n;
//        if(pt1==GJK_vertex) n=conv_vec2arr(e2); else n=conv_vec2arr(e1);
//        J = J - n*(~n*J);
//        if(pt1==GJK_vertex) W.kinematicsVec(vec, Jv, s2->body, s2->body->X.rot/(p1-p2));
//        if(pt2==GJK_vertex) W.kinematicsVec(vec, Jv, s1->body, s1->body->X.rot/(p1-p2));
//        J += n*(~n*Jv);
//      }
//    }
//  }
//  //std::cout << "J pre fac:" << J << std::endl;

///*  //reduce by radii
//  double l2=sumOfSqr(v), l=sqrt(l2);
////  double fac = (l-s1->size(3)-s2->size(3))/l;
////  if(&J){
////    arr d_fac = (1.-(l-s1->size(3)-s2->size(3))/l)/l2 *(~v)*J;
////    J = J*fac + v*d_fac;
////    std::cout << "J fac:" << J << std::endl;

////  }
////  v *= fac;

//  if(negScalar){
//    if(&J) J = ~(v/(-l))*J;
//    v = ARR(-l);
//  }*/

//  auto l = norm2( v ) + margin;
//  auto j = Jnorm( v );

//  v = ARR(l);
//  J = j;

//  //std::cout << "J final:" << J << std::endl;
//}

