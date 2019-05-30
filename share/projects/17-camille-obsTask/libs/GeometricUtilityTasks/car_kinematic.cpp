#include <car_kinematic.h>

CarKinematic::CarKinematic( const std::string & object )
  : object_( object )
{

}

mlr::String CarKinematic::shortTag(const mlr::KinematicWorld& G)
{
  return mlr::String("CarKinematic");
}

void CarKinematic::phi(arr& y, arr& J, const WorldL& Gs, double tau, int t)
{
  //TaskMap::phi(y,J,Ks,tau,t);
  CHECK(order==1,"");
  CHECK(Gs.size() >= 1,"");

  mlr::Frame *object = Gs(1)->getFrameByName( object_.c_str() );
  const auto Xoffset = -0.5 * object->shape->size(0);

  // initialize y and J
  y.resize(1);//zeros(dim_phi(Gs, t));
  if(&J){
    uintA qidx(Gs.N);
    qidx(0)=0;
    for(uint i=1;i<Gs.N;i++) qidx(i) = qidx(i-1)+Gs(i-1)->q.N;
    J = zeros(y.N, qidx.last()+Gs.last()->q.N);
  }

  // get speed vector
  arr y_vel,Jvel;
  TaskMap_Default vel(posDiffTMT, object->ID, mlr::Vector(Xoffset,0,0));
  vel.order = 1;
  vel.phi(y_vel, Jvel, Gs, tau, t);

  // get orientation vector
  arr y_vec,Jvec;
  TaskMap_Default vec(vecTMT, object->ID, mlr::Vector(0,1,0));
  vec.order = 0;
  vec.phi(y_vec, Jvec, *Gs(1), t);

  // commit results
  y(0) = scalarProduct(y_vel, y_vec);
  if(&J)
  {
    const auto offset = J.d1 / Gs.N;

    CHECK( Jvec.d1 == offset, "" );

    for(auto i = 0; i < Gs.size(); ++i)
    {
      auto JvelSub = Jvel.sub(0, -1, i * offset, (i+1) * offset -1);
      if( i == 1 )
      {
        J.setMatrixBlock( ~y_vel * Jvec + ~y_vec * JvelSub, 0, i * offset );
      }
      else
      {
        J.setMatrixBlock( ~y_vec * JvelSub, 0, i * offset );
      }
    }
  }
}

void CarKinematic::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t)
{
  CHECK(false,"The phi function taking the list of kinematic worlds should be taken");
}

uint CarKinematic::dim_phi(const mlr::KinematicWorld& K)
{
  //CHECK(false,"The phi function taking the list of kinematic worlds should be taken");

  return dim_;
}
