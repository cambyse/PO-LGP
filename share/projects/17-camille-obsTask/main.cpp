#include <Motion/komo.h>

using namespace std;

struct HeadPoseMap:TaskMap{

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1){
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

  virtual uint dim_phi(const mlr::KinematicWorld& G){
    return dim_;
  }

  virtual mlr::String shortTag(const mlr::KinematicWorld& G){ return mlr::String("HeadPoseMap"); }

  static arr buildTarget( mlr::Vector const& position, double yaw_deg )
  {
    mlr::Quaternion target_quat;
    target_quat.setDeg(yaw_deg, 0.0, 0.0, 1.0);

    arr target_arr(dim_);
    target_arr.setVectorBlock( position.getArr(), 0 );
    target_arr.setVectorBlock( conv_quat2arr(target_quat), position.getArr().dim(0) - 1 );

    return target_arr;
  }

private:
  static const uint dim_ = 7;
};

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

struct HeadGetSight:TaskMap{

  HeadGetSight( const arr& objectPosition, const arr& pivotPoint )
    : TaskMap()
    , objectPosition_( objectPosition )
    , pivotPoint_    ( pivotPoint )
    , headViewingDirection_( 0.0, -1.0, 0.0 )
  {
    w1_ = objectPosition_ - pivotPoint_;
    w1_ = w1_ * 1. / norm2( w1_ );
  }

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1){
    mlr::Shape *head = G.getShapeByName("manhead");
    arr headPosition, headJPosition;
    G.kinematicsPos( headPosition, headJPosition, head->body );

    arr headViewingDirection, headJViewingDirection;
    G.kinematicsVec( headViewingDirection, headJViewingDirection, head->body, headViewingDirection_ ); // get function to minimize and its jacobian in state G

    // build u : vector between head and pivot point
    arr u = pivotPoint_ - headPosition;
    double uLength = norm2( u );
    arr Ju = - headJPosition;

    // build v : orientation vector of the head
    arr v = headViewingDirection;
    arr Jv = headJViewingDirection;

    // get Jacobian of the norm operator
    arr JnormU = Jnorm( u );

    // instantiate a temporary vector for cost and its Jacobian
    arr tmp_y = zeros( dim_ );
    arr tmp_J = zeros( dim_, headJPosition.dim(1) );
    //arr tmp_J( dim_, headJPosition.dim(1) );

    tmp_y.setVectorBlock( u - w1_ * uLength , 0 );  // cost corresponding to the head position
    tmp_y.setVectorBlock( v - w1_ , u.d0 - 1 );     // cost corresponding to the head direction

    tmp_J.setMatrixBlock( Ju - w1_ * JnormU * Ju, 0, 0 );
    tmp_J.setMatrixBlock( Jv, Ju.d0 - 1, 0 );

    // commit results
    y = tmp_y;
    if(&J) J = tmp_J;

    // logs
    //std::cout << "u - w1_ * uLength:" << u - w1_ * uLength<< std::endl;
    //std::cout << "v - w1_:" << v - w1_ << std::endl;

    //std::cout << "headPosition:" << headPosition<< std::endl;
    //std::cout << "headViewingDirection" << headViewingDirection << std::endl;
    //std::cout << "u/uL:" << u / uLength<< std::endl;
    //std::cout << "w1_:" << w1_ << std::endl;
  }

  virtual uint dim_phi(const mlr::KinematicWorld& G){
    return dim_;
  }

  virtual mlr::String shortTag(const mlr::KinematicWorld& G){ return mlr::String("HeadGetSight"); }

private:
  // parameters
  static const uint dim_ = 6;
  const arr objectPosition_;
  const arr pivotPoint_;
  const mlr::Vector headViewingDirection_;  // is Vec to be compatible with the interface of G.kinematicsVec

  // state
  arr w1_;
};



//===========================================================================

void move(){
  
  KOMO komo;
  komo.setConfigFromFile();

  // mlr::Body *b = komo.world.getBodyByName("/human/base");
  // b->X.addRelativeTranslation(.3,0,0);

  //  komo.setHoming(-1., -1., 1e-1);
  //  komo.setSquaredQVelocities();
  komo.setSquaredFixJointVelocities();
  komo.setSquaredFixSwitchedObjects();
  komo.setSquaredQAccelerations();

  //komo.setPosition(1., 1.1, "humanL", "target", OT_sumOfSqr, NoArr, 1e2);

  //komo.setPosition(1., 2.0, "manhead", "target", OT_sumOfSqr, ARR(0,0,0), 1e2);  // objective to have the head on the target on this time slab
  //komo.setTask(startTime, endTime, new TaskMap_Default(posTMT, world, shape, NoVector, shapeRel, NoVector), type, target, prec);

  //arr targetArr1 = HeadPoseMap::buildTarget( mlr::Vector( 0, -0.3, 1.7 ), 80 );
  //komo.setTask(1.0, 3.0, new HeadPoseMap(), OT_sumOfSqr, targetArr1, 1e2);

  komo.setTask( 1.0, 2.0, new HeadGetSight( ARR(  0.0, -1.0, 1.9 ),    // object position
                                            ARR( -0.2, -0.5, 1.9 ) ),  // pivot position
                OT_sumOfSqr, NoArr, 1e2 );

  //komo.setTask(.3, .5, new HandPositionMap(), OT_sumOfSqr, ARR(.5,.5,1.3), 1e2);
  //komo.setTask(.8, 1., new HandPositionMap(), OT_sumOfSqr, ARR(.8,0.,1.3), 1e2);
  //komo.setTask(.8, 1., new TaskMap_Default(posDiffTMT, komo.world, "/human/humanR", NoVector, "target", NoVector), OT_sumOfSqr, NoArr, 1e2);

//  komo.setTask(.3, 1., new TaskMap_Default(gazeAtTMT, komo.world, "eyes", NoVector, "target", NoVector), OT_sumOfSqr, NoArr, 1e2);

  komo.setSlowAround( 3.0, .1, 1e3 );

  komo.reset();
  komo.run();
  komo.checkGradients();

  Graph result = komo.getReport(true);

  for(;;) komo.displayTrajectory(.1, true);
}

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  move();

  return 0;
}

//
//  mlr::Vector x(-1.0, 0.0, 0.0);
//  mlr::Vector dx(0.01, 0.01, 0.0);

//  mlr::Vector x_dx = x + dx;
//  double lx = x.length();
//  double lx_dx = x_dx.length();
//  auto J = Jnorm( x.getArr() );

//  std::cout << "J:" << J << std::endl;
//  std::cout << "lx_dx:" << lx_dx << " " << lx + ( J ) * ( dx.getArr() ) << std::endl;
//

// check final configuration
//  //auto N_final = komo.MP->configurations.N;
//  HeadGetSight t( ARR(  0.0, -1.0, 1.9 ),    // object position
//                  ARR( -0.2, -0.5, 1.9 ) );

//  const mlr::KinematicWorld & lastWorld = *komo.MP->configurations.last();
//  arr y, J;
//  t.phi( y, J, lastWorld );
// check

/*struct HeadGetSight:TaskMap{

  HeadGetSight( const arr& objectPosition, const arr& pivotPoint )
    : TaskMap()
    , objectPosition_( objectPosition )
    , pivotPoint_    ( pivotPoint )
    , headViewingDirection_( 0.0, -1.0, 0.0 )
  {
    w1_ = objectPosition_ - pivotPoint_;
    w1_.normalize();

    // build target quaternion
    arr vec = crossProduct( headViewingDirection_.getArr(), w1_.getArr() );

    mlr::Vector rotationAxis( vec );
    double angle = asin( rotationAxis.length() );
    rotationAxis.normalize();

    // here rotationAxis is a unit vector
    targetOrientation_.setVec( angle * rotationAxis );

    //mlr::Vector quaternionDefinitionVector( angle * );
    //std::cout << "vec:" << vec << std::endl;
    //std::cout << "angle :" << " " << angle * 180 / 3.1415 << std::endl;
    //std::cout << "rotation axis:" << rotationAxis << std::endl;
  }

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1){
    mlr::Shape *head = G.getShapeByName("manhead");
    arr headPosition, headJPosition;
    G.kinematicsPos( headPosition, headJPosition, head->body );

    //arr headOrientation, headJOrientation;
    //G.kinematicsVec( headOrientation, headJOrientation, head->body, headViewingDirection_ ); // get function to minimize and its jacobian in state G

    arr quatHead, JquatHead;
    G.kinematicsQuat(quatHead, JquatHead, head->body); // get function to minimize and its jacobian in state G

    // build u : vector between head and pivot point
    mlr::Vector u = pivotPoint_ - headPosition;

    // build v : orientation vector of the head
    //mlr::Vector v = head->body->X.rot * headViewingDirection_;

    // get Jacobian of the norm operator
    arr normJu = normJacobian( u.getArr() );
    //arr normJv = normJacobian( v.getArr() );

    // we have to minimize u(x) - w and v(r(j) * headViewingDirection) - w
    arr tmp_y = zeros( dim_ );
    arr tmp_J = zeros( dim_, headJPosition.dim(1) );

    tmp_y.setVectorBlock( ( u - w1_ * u.length() ).getArr() , 0 );
    //tmp_y.setVectorBlock( ( v - w1_ ).getArr() , u.getArr().dim( 0 ) - 1 );
    //auto orientationDiff = quatHead - conv_quat2arr( targetOrientation_ );
    //tmp_y.setVectorBlock( orientationDiff , u.getArr().dim( 0 ) - 1 );

    tmp_J.setMatrixBlock( -headJPosition + w1_ * normJu * headJPosition, 0, 0 );
    //tmp_J.setMatrixBlock( JquatHead, headJPosition.dim(0) - 1, 0 );

    // commit results
    y = tmp_y;
    if(&J) J = tmp_J;
  }

  virtual uint dim_phi(const mlr::KinematicWorld& G){
    return dim_;
  }

  virtual mlr::String shortTag(const mlr::KinematicWorld& G){ return mlr::String("HeadGetSight"); }

private:
  static const uint dim_ = 3;//7;
  const arr objectPosition_;
  const arr pivotPoint_;
  const mlr::Vector headViewingDirection_;
  mlr::Vector w1_;
  mlr::Quaternion targetOrientation_;
};*/

// the following version is ok but the quaternion objective is a too strong constraint
//struct HeadGetSight:TaskMap{

/*  HeadGetSight( const arr& objectPosition, const arr& pivotPoint, const arr& initialHeadPosition )
    : TaskMap()
    , objectPosition_( objectPosition )
    , pivotPoint_    ( pivotPoint )
    , initialHeadPosition_( initialHeadPosition )
    , headViewingDirection_( 0.0, -1.0, 0.0 )
  {
    mlr::Vector w1 = objectPosition_ - pivotPoint_;
    w1.normalize();

    // build target direction
    targetPosition_ = objectPosition_ + ( w1 * ( w1 * ( initialHeadPosition_ - objectPosition_ ) ) );

    // build target quaternion
    arr vec = crossProduct( headViewingDirection_.getArr(), w1.getArr() );

    mlr::Vector rotationAxis( vec );
    double angle = asin( rotationAxis.length() );
    rotationAxis.normalize();

    // here rotationAxis is a unit vector
    targetOrientation_.setVec( angle * rotationAxis );
  }

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1){
    mlr::Shape *head = G.getShapeByName("manhead");

    // retrieve kin info
    arr headPosition, headJPosition;
    G.kinematicsPos( headPosition, headJPosition, head->body );

    arr quatHead, JquatHead;
    G.kinematicsQuat(quatHead, JquatHead, head->body); // get function to minimize and its jacobian in state G

    // create tmp variables
    arr tmp_J = zeros( dim_, headJPosition.dim(1) );
    arr tmp_y = zeros( dim_ );

    tmp_y.setVectorBlock( ( headPosition - targetPosition_ ).getArr() , 0 );

    auto orientationDiff = quatHead - conv_quat2arr( targetOrientation_ );
    tmp_y.setVectorBlock( orientationDiff , targetPosition_.getArr().dim( 0 ) - 1 );

    tmp_J.setMatrixBlock( headJPosition, 0, 0 );
    tmp_J.setMatrixBlock( JquatHead, headJPosition.dim(0) - 1, 0 );

    // commit results
    y = tmp_y;
    if(&J) J = tmp_J;
  }

  virtual uint dim_phi(const mlr::KinematicWorld& G){
    return dim_;
  }

  virtual mlr::String shortTag(const mlr::KinematicWorld& G){ return mlr::String("HeadGetSight"); }

private:
  static const uint dim_ = 3;
  const arr objectPosition_;
  const arr pivotPoint_;
  const arr initialHeadPosition_;
  const mlr::Vector headViewingDirection_;
  mlr::Vector targetPosition_;
  mlr::Quaternion targetOrientation_;
};*/

