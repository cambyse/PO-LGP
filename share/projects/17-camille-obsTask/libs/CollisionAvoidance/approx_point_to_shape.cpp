#include <approx_point_to_shape.h>

#include <Kin/kin.h>
#include <Kin/proxy.h>

//-------ApproxPointToShape----------------//

void ApproxPointToShape::phi( arr& y, arr& J, const mlr::KinematicWorld& G, int t )
{
  arr tmp_y = zeros( dim_phi( G ) );
  arr tmp_J = zeros( dim_phi( G ), G.q.N );

  for( mlr::Proxy *p: G.proxies )
  {
    if((p->a==i_ && p->b==j_) || (p->a==j_ && p->b==i_))
    {
      if( p->d > 0 )
      {
        phiProxy( tmp_y, tmp_J, G, p );
        //phiFCL( tmp_y, tmp_J, G );
      }
      else
      {
        // collision already!
        // -> let it 0..
      }
      break;
    }
  }

  // commit results
  y = tmp_y;
  if(&J) J = tmp_J;
}

void ApproxPointToShape::phiProxy( arr& y, arr& J, const mlr::KinematicWorld& G, mlr::Proxy * p )
{
  mlr::Frame *a = G.frames(p->a);
  mlr::Frame *b = G.frames(p->b);

  auto arel=a->X.rot/(p->posA-a->X.pos);
  auto brel=b->X.rot/(p->posB-b->X.pos);

  arr posA;
  arr posB;
  arr JposA;
  arr JposB;
  G.kinematicsPos(posA, JposA, a, arel);
  G.kinematicsPos(posB, JposB, b, brel);

  double d   = norm2( posA - posB );
  arr JnormD = Jnorm( posA - posB );

  const double w = 10;
  y( 0 ) = w * ( - d + 0.05 );
  J = w * ( - JnormD * ( JposA - JposB ) );
}
