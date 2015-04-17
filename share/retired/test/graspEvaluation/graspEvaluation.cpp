#include "graspEvaluation.h"
#include <Ors/ors.h>

#include <Ors/ors_physx.h>

struct sGraspEvaluation{
  SwiftInterface swift;
  
  //-- PhysX stuff
  PhysXInterface *physx;
  OpenGL *glPhysx;
};

GraspEvaluation::GraspEvaluation():gl("GraspEvaluation view"){
  s = new sGraspEvaluation;
}

void GraspEvaluation::closeFingers(){
  s->swift.init(grasp);
  s->swift.computeProxies(grasp, false);
  uint pi;
  ors::Proxy *p;
  double dmin, step=.01;
  arr q;
  grasp.getJointState(q);
  for(uint i=3;i<q.N;i++){
    for(uint t=0;t<100;t++){
      q(i) += step;
      grasp.setJointState(q);
      s->swift.computeProxies(grasp, false);
      //grasp.reportProxies();
      gl.update();
      dmin=1.;
      for_list(Type, p, grasp.proxies) if(p->d<dmin) dmin=p->d;
      if(dmin<1e-10){
	q(i) -= step;
	break;
      }
    }
  }
  grasp.setJointState(q);
  s->swift.computeProxies(grasp, false);
  //grasp.reportProxies();
//   for(uint i=3;i<q.N;i++) q(i) += step;
//   grasp.setJointState(q);
  //s->swift.computeProxies(grasp, false);
  //grasp.reportProxies();
}

void GraspEvaluation::getContactPoints(double distanceThreshold){
  if(!grasp.proxies.N){
    s->swift.init(grasp);
    s->swift.computeProxies(grasp, false);
  }
  //grasp.reportProxies();
  uint i;
  ors::Proxy *p;
  for_list(Type, p, grasp.proxies){
    ors::Vector contact, normal;
    if(grasp.shapes(p->a)->body->index==0){ //index==0 is the object
      contact=p->posA; normal=-p->normal;
    }else{
      contact=p->posB; normal= p->normal;
    }
    if(p->d<distanceThreshold){
      cout <<"contact " <<i <<" d=" <<p->d <<" contact=" <<contact <<" normal=" <<normal <<endl;
      contactPoints.append(ARRAY(contact));
      contactNormals.append(ARRAY(normal));
    }
  }
  contactPoints.reshape(contactPoints.N/3,3);
  contactNormals.reshape(contactNormals.N/3,3);
  //cout <<contactPoints <<endl <<contactNormals <<endl;
  
  arr mean = sum(contactPoints,0)/(double)contactPoints.d0;
  
  score_forceClosure  = forceClosureFromProxies(grasp, 0, 0.01, .2, 0.);
  score_torqueClosure = forceClosureFromProxies(grasp, 0, 0.01, .2, -1.);
  score_wrenchClosure = forceClosureFromProxies(grasp, 0, 0.01, .2, 1.);

  //forceClosureMeassure = forceClosure(contactPoints, contactNormals,
  //ors::Vector(mean),
  //5., 1., NULL);

  cout <<"force  closure = " <<score_forceClosure  <<endl;
  cout <<"torque closure = " <<score_torqueClosure <<endl;
  cout <<"wrench closure = " <<score_wrenchClosure <<endl;
}

void GraspEvaluation::copyGraspFromOrs(const ors::KinematicWorld& all,
				       const char* palmBodyName,
				       const char* objShapeName){
  uint i,j;
  ors::Shape *s;
  ors::Body *b;
  ors::Joint *l;

  //collect all bodies of the hand along the tree
  ors::Body *palm = all.getBodyByName(palmBodyName);
  ors::Shape *obj = all.getShapeByName(objShapeName);
  MT::Array<ors::Body*> handBodies;
  handBodies.append(palm);
  for_list(Type, b, handBodies) for_list(Type, l, b->outLinks) handBodies.append(l->to);
  cout <<"hand bodies:";  listWrite(handBodies, cout);
  cout <<endl;

  grasp.clear();
  
  //copy obj (always has index 0)
  ors::Body *objBody  = new ors::Body(grasp);
  objBody->X = obj->X;
  objBody->name = "object";
  ors::Shape *s_new;
  s_new = new ors::Shape(grasp, objBody, obj);
  s_new->rel.setZero();
  
  //copy hand
  MT::Array<ors::Body*> bodyMap(all.bodies.N);
  for_list(Type, b, handBodies){
    bodyMap(b->index) = new ors::Body(grasp, b);
    bodyMap(b->index)->type = ors::kinematicBT;
    for_list(Type, s, b->shapes){
      new ors::Shape(grasp, bodyMap(b->index), s);
    }
  }
  for_list(Type, b, handBodies){
    for_list(Type, l, b->outLinks){
      CHECK_EQ(l->from->index , b->index,"");
      new ors::Joint(grasp, bodyMap(l->from->index), bodyMap(l->to->index), l);
    }
  }

  grasp.calcBodyFramesFromJoints();
  init(grasp,gl,NULL);
  grasp >>FILE("grasp.ors");
}
  
void GraspEvaluation::simulateInPhysX(){
  s->physx = new PhysXInterface;
  s->glPhysx = new OpenGL("PhysX internal view");
  s->physx->G = &grasp;
  s->physx->create();
  s->glPhysx->add(glStandardScene, NULL);
  s->glPhysx->add(glPhysXInterface, s->physx);
  s->glPhysx->setClearColors(1.,1.,1.,1.);
  s->glPhysx->camera.setPosition(10.,-15.,8.);
  s->glPhysx->camera.focus(0,0,1.);
  s->glPhysx->camera.upright();

  for(uint t=0; t<1000; t++) {
    cout <<"\r t=" <<t <<std::flush;
    s->physx->step();
    gl.update();
    s->glPhysx->update();
    
    //check if object still in hand
    
    //jiggle at the grasp hand
    ors::Quaternion &rot(grasp.bodies(1)->X.rot);
    ors::Quaternion a,b,c;
    if(t>100){
      c.setDeg(3.,0,0,1);
      switch((t/100)%3){
	case 0: a.setDeg(2.,0,0,1); break;
	case 1: a.setDeg(2.,0,1,0); break;
	case 2: a.setDeg(2.,1,0,0); break;
      }
    }
    a = a*rot*c; //rotate
    b.setRandom();
    rot.setInterpolate(0.0, a, b); //add some noise
    grasp.calcBodyFramesFromJoints();
  }
}

#include <Core/array_t.h>
template MT::Array<ors::Body*>::Array(uint);
