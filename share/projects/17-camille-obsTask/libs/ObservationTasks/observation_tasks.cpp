#include <observation_tasks.h>

//===========================================================================

static double norm2( const arr & x )
{
  return sqrt( ( ( ~ x ) * x )( 0 ) );
}

static arr Jnorm( const arr & x )
{
  arr J( 1, x.N );

  // compute sqrNorm
  double norm = norm2( x );

  // compute each jacobian element
  if( norm > 0.000001 )
  {
    for( auto i = 0; i < x.N; ++i )
      J( 0, i ) = x( i ) / norm ;
  }
  else
  {
    J.setZero();
  }

  return J;
}

//===========================================================================

void HeadPoseMap::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t)
{
  mlr::Shape *head = G.getShapeByName("manhead");
  arr posHead, JposHead;
  G.kinematicsPos(posHead, JposHead, head->body);    // get function to minimize and its jacobian in state G

  arr quatHead, JquatHead;
  G.kinematicsQuat(quatHead, JquatHead, head->body); // get function to minimize and its jacobian in state G

  // concatenate y and J from position and orientation (quaternion)
  arr tmp_y=zeros(dim_);
  arr tmp_J=zeros(dim_, JposHead.dim(1));
  tmp_y.setVectorBlock(posHead, 0);
  tmp_y.setVectorBlock(quatHead, posHead.dim(0) - 1);

  tmp_J.setMatrixBlock(JposHead, 0, 0);
  tmp_J.setMatrixBlock(JquatHead, JposHead.dim(0) - 1, 0);

  // commit results
  y = tmp_y;
  if(&J) J = tmp_J;
}

arr HeadPoseMap::buildTarget( mlr::Vector const& position, double yaw_deg )
{
  mlr::Quaternion target_quat;
  target_quat.setDeg(yaw_deg, 0.0, 0.0, 1.0);

  arr target_arr(dim_);
  target_arr.setVectorBlock( position.getArr(), 0 );
  target_arr.setVectorBlock( conv_quat2arr(target_quat), position.getArr().dim(0) - 1 );

  return target_arr;
}

//===========================================================================

HeadGetSight::HeadGetSight( const arr& objectPosition, const arr& pivotPoint )
  : TaskMap()
  , objectPosition_( objectPosition )
  , pivotPoint_    ( pivotPoint )
  , headViewingDirection_( 0.0, -1.0, 0.0 )
  , moveAroundPivotDefined_( false )
{
  w1_ = objectPosition_ - pivotPoint_;

  if( norm2( w1_ ) > 0 ) // normalize if the vector is not null
  {
    w1_ = w1_ * 1. / norm2( w1_ );
    moveAroundPivotDefined_ = true;
  }

  // with current sensor position, determine which object occludes ( at least one triangle is traversed by the ray )

  // determine the pivot point
}

void HeadGetSight::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t)
{
  mlr::Shape *head = G.getShapeByName("manhead");
  arr headPosition, headJPosition;
  G.kinematicsPos( headPosition, headJPosition, head->body );

  arr headViewingDirection, headJViewingDirection;
  G.kinematicsVec( headViewingDirection, headJViewingDirection, head->body, headViewingDirection_ ); // get function to minimize and its jacobian in state G

  // build u : vector between head and object point
  arr u = objectPosition_ - headPosition;
  double normU = norm2( u );
  arr Ju = - headJPosition;
  arr JnormU = Jnorm( u );  // get Jacobian of the norm operator

  // build v : orientation vector of the head
  arr v1 = headViewingDirection;
  arr Jv1 = headJViewingDirection;

  // instantiate a temporary vector for cost and its Jacobian
  arr tmp_y = zeros( dim_ );
  arr tmp_J = zeros( dim_, headJPosition.dim(1) );

  // u - v
  tmp_y.setVectorBlock( u  - v1  * normU                      , 0 );    // cost
  tmp_J.setMatrixBlock( Ju -( v1 * JnormU * Ju + Jv1 * normU ),0 , 0 ); // jacobian

  // u - w
  if( moveAroundPivotDefined_ )
  {
    tmp_y.setVectorBlock( u -  w1_ * normU      , u.d0 - 1 );            // cost
    tmp_J.setMatrixBlock( Ju - w1_ * JnormU * Ju, Ju.d0 - 1, 0 );        // jacobian
  }

  // commit results
  y = tmp_y;
  if(&J) J = tmp_J;
}
