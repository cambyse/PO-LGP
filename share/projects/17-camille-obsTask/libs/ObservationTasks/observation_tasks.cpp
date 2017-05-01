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
    {
      J( 0, i ) = x( i ) / norm ;
    }
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
  arr u1 = u / normU;
  arr Ju1 = ( Ju * normU - u * JnormU * Ju ) / ( normU * normU ); // jacobian of u normalized

  // build v : orientation vector of the head
  arr v1 = headViewingDirection;
  arr Jv1 = headJViewingDirection;

  // instantiate a temporary vector for cost and its Jacobian
  arr tmp_y = zeros( dim_ );
  arr tmp_J = zeros( dim_, headJPosition.dim(1) );

  // u - v
  tmp_y.setVectorBlock( 10.0 * ( u1  - v1)     , 0 );    // cost
  tmp_J.setMatrixBlock( 10.0 * ( Ju1 - Jv1), 0 , 0 );    // jacobian

  // u - w
  if( moveAroundPivotDefined_ )
  {
    tmp_y.setVectorBlock( u1 -  w1_     , u.d0 - 1 );            // cost
    tmp_J.setMatrixBlock( Ju1, Ju.d0 - 1, 0 );                    // jacobian
  }

  // commit results
  y = tmp_y;
  if(&J) J = tmp_J;
}

//===========================================================================

HeadGetSightQuat::HeadGetSightQuat( const arr& objectPosition, const arr& pivotPoint )
  : TaskMap()
  , objectPosition_( objectPosition )
  , pivotPoint_    ( pivotPoint )
  , headViewingDirection_( 0.0, -1.0, 0.0 )
  , moveAroundPivotDefined_( false )
{
  w1_ = objectPosition_ - pivotPoint_;
  //std::cout << "w1:" << w1_ << std::endl;
  mlr::Quaternion targetQuat;
  targetQuat.setDiff( mlr::Vector( 0, -1.0, 0 ), w1_ / norm2( w1_ ) );

  targetQuat_ = conv_quat2arr( targetQuat );

  if( norm2( w1_ ) > 0 ) // normalize if the vector is not null
  {
    w1_ = w1_ * 1. / norm2( w1_ );
    moveAroundPivotDefined_ = true;
  }

  // with current sensor position, determine which object occludes ( at least one triangle is traversed by the ray )

  // determine the pivot point
}

void HeadGetSightQuat::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t)
{
  mlr::Shape *head = G.getShapeByName("manhead");
  arr headPosition, headJPosition;
  G.kinematicsPos( headPosition, headJPosition, head->body );

  arr headQuat, JheadQuat;
  G.kinematicsQuat( headQuat, JheadQuat, head->body ); // get function to minimize and its jacobian in state G

  //arr headViewingDirection, headJViewingDirection;
  //G.kinematicsQuat();
  //G.kinematicsVec( headViewingDirection, headJViewingDirection, head->body, headViewingDirection_ ); // get function to minimize and its jacobian in state G

  // build u : vector between head and object point
  arr u = objectPosition_ - headPosition;
  double normU = norm2( u );
  arr Ju = - headJPosition;
  arr JnormU = Jnorm( u );  // get Jacobian of the norm operator
  arr u1 = u / normU;
  arr Ju1 = ( Ju * normU - u * JnormU * Ju ) / ( normU * normU ); // jacobian of u normalized

  // build v : orientation vector of the head
  //arr v1 = headViewingDirection;
  //arr Jv1 = headJViewingDirection;

  // instantiate a temporary vector for cost and its Jacobian
  arr tmp_y = zeros( dim_ );
  arr tmp_J = zeros( dim_, headJPosition.dim(1) );

  // head orientation
  //tmp_y.setVectorBlock( 10.0*( u1  - v1)                       , 0 );    // cost
  //tmp_J.setMatrixBlock( 10.0 * ( Ju1 - Jv1), 0 , 0 ); // jacobian

  tmp_y.setVectorBlock( 1.0 * ( headQuat - targetQuat_ )       , 0 );    // cost
  tmp_J.setMatrixBlock( 1.0 * ( JheadQuat              )  , 0 , 0 ); // jacobian

  // head alignment
  if( moveAroundPivotDefined_ )
  {
    tmp_y.setVectorBlock( u1 -   w1_, headQuat.d0 - 1 );                    // cost
    tmp_J.setMatrixBlock( Ju1,   JheadQuat.d0 - 1, 0 );                    // jacobian
  }

  // head distance
  arr norm_a = zeros( 1 );
  norm_a( 0 ) = normU - 0.6;
  tmp_y.setVectorBlock( norm_a,         headQuat.d0 + u1.d0 - 1 );
  tmp_J.setMatrixBlock( JnormU * Ju,    JheadQuat.d0 + Ju1.d0 - 1, 0 );                    // jacobian

  // commit results
  y = tmp_y;
  if(&J) J = tmp_J;
}

//===========================================================================

ActiveGetSight::ActiveGetSight( mlr::String const& headName,
                                        mlr::String const& containerName,
                                        //arr const& aimPoint,
                                        arr const& pivotPoint,
                                        double preferedDistance )
  : TaskMap()
  , headName_     ( headName )
  , containerName_( containerName )
  //, aimPoint_     ( aimPoint )
  , pivotPoint_   ( pivotPoint )
  , preferedDistance_( preferedDistance )
{

}

void ActiveGetSight::phi( arr& y, arr& J, mlr::KinematicWorld const& G, int t )
{
  // get Object position and pivot position
  mlr::Body * container = G.getBodyByName( containerName_ );
  arr aimPosition, aimJPosition;
  G.kinematicsPos( aimPosition, aimJPosition, container );

  arr pivotPosition, pivotJPosition;
  G.kinematicsPos( pivotPosition, pivotJPosition, container, mlr::Vector( pivotPoint_ ) );

  //std::cout << "world aimPosition:" << aimPosition << std::endl;
  //std::cout << "world pivotPosition:" << pivotPosition << std::endl;

  // get Head position
  mlr::Shape * head = G.getShapeByName( headName_ );
  arr headPosition, headJPosition;
  G.kinematicsPos( headPosition, headJPosition, head->body );

  //std::cout << "headPosition:" << headPosition << std::endl;

  arr headQuat, JheadQuat;
  G.kinematicsQuat( headQuat, JheadQuat, head->body ); // get function to minimize and its jacobian in state G

  // intermediary computations
  // build w1
  arr w = aimPosition - pivotPosition;
  double normW = norm2( w );
  //std::cout << "normW:" << normW << std::endl;
  arr w1 = w * 1. / normW;
  arr JnormW = Jnorm( w );

  arr Jw = aimJPosition - pivotJPosition;
  arr Jw1 = ( Jw * normW - w * JnormW * Jw ) / ( normW * normW );
  //std::cout << "w1:" << w1 << std::endl;
  //mlr::Quaternion _targetQuat;
  //_targetQuat.setDiff( mlr::Vector( 0, -1.0, 0 ), w1 );
  //arr targetQuat = conv_quat2arr( _targetQuat );

  // build u : vector between head and aiming point
  arr u = aimPosition - headPosition;
  double normU = norm2( u );
  arr Ju = aimJPosition - headJPosition;
  arr JnormU = Jnorm( u );  // get Jacobian of the norm operator
  arr u1 = u / normU;
  arr Ju1 = ( Ju * normU - u * JnormU * Ju ) / ( normU * normU ); // jacobian of u normalized
  //std::cout << "u1:" << u1 << std::endl;

  // build v :
  arr v, Jv;
  G.kinematicsVec( v, Jv, head->body, mlr::Vector( 0, -1.0, 0 ) ); // get function to minimize and its jacobian in state G
  double normV = norm2( v );
  arr JnormV = Jnorm( v );  // get Jacobian of the norm operator
  arr v1 = v / normV;
  arr Jv1 = ( Jv * normV - v * JnormV * Jv ) / ( normV * normV ); // jacobian of u normalized

  // instantiate a temporary vector for cost and its Jacobian
  arr tmp_y = zeros( dim_ );
  arr tmp_J = zeros( dim_, headJPosition.dim(1) );

  // head orientation
  tmp_y.setVectorBlock( ( u1  - v1 )                       , 0 );    // cost
  tmp_J.setMatrixBlock( ( Ju1 - Jv1 ), 0 , 0 ); // jacobian

  // head alignment
  tmp_y.setVectorBlock( u1 -  w1,   u1.d0  );                    // cost
  tmp_J.setMatrixBlock( Ju1 -  Jw1, Ju1.d0, 0 );                    // jacobian

  // head distance
  double d = normU - preferedDistance_;
  tmp_y( 2*u1.d0 ) = d;
  tmp_J.setMatrixBlock( JnormU * Ju, 2 * Ju1.d0, 0 );                    // jacobian
  ///////
//  for( auto p : G.proxies )
//  {
//    std::cout << G.shapes( p->a )->name << "-" << G.shapes( p->b )->name << std::endl;
//  }

  // commit results
  y = tmp_y;
  if(&J) J = tmp_J;
}

