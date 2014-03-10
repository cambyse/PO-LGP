#include <MT/soc.h> 
#include <MT/util.h>
#include <MT/opengl.h>
#include <MT/ors.h>     
#include <MT/plot.h>    
#include <Core/array.h> 
#include <MT/ors_physx.h>

#define DMITRY

//===========================================================================
// const double PI = 3.1415926535897932384626; 
// const double PI_OVER_2 = 0.5* PI;
 
void createOrs(ors::KinematicWorld& ors, OpenGL& gl) {
  ors.clear(); 
   
  for(uint k=0; k<3; k++) { 
    ors::Body *b = new ors::Body(ors);  
    b->X.setRandom(); 
    b->X.pos(2) += 1.;
    b->name <<"Box_" <<k;
    b->type = ors::staticBT;
    ors::Shape *s = new ors::Shape(ors, b);
    s->type=ors::boxST;
    s->size[0]=.1; s->size[1]=.1; s->size[2]=.1; s->size[3]=.1;
  }
  for(uint k=0; k<3; k++) {
    ors::Body *b = new ors::Body(ors);
    b->X.setRandom(); 
    b->X.pos(2) += 1.;
    b->name <<"Sphere_" <<k;
    ors::Shape *s = new ors::Shape(ors, b);
    s->type=ors::sphereST;
    s->size[0]=.1; s->size[1]=.1; s->size[2]=.1; s->size[3]=.1;
  }
   
  ors.calcShapeFramesFromBodies();
  
  gl.add(glStandardScene,NULL);
  gl.add(ors::glDrawGraph,&ors);
  gl.setClearColors(1.,1.,1.,1.);
  gl.camera.setPosition(10.,-15.,8.);
  gl.camera.focus(0,0,1.);
  gl.update();
}

void problem1(){   
  cout <<"\n= Sample physX simulation=\n" <<endl;
   ors::KinematicWorld ors;
   OpenGL gl, phys_gl; 
  
   createOrs(ors, gl);
   PhysXInterface physx;
   physx.G = &ors;
   physx.create();
  
   phys_gl.add(glStandardScene, NULL);
   phys_gl.add(glPhysXInterface, &physx);
   phys_gl.setClearColors(1.,1.,1.,1.);
   phys_gl.camera.setPosition(10.,-15.,8.);
   phys_gl.camera.focus(0,0,1.);
   phys_gl.watch();
  
  for(uint t=0; t<1000; t++) {
    physx.step();
    phys_gl.update();
    gl.update();
  }
  
}

void problem2(){   
  cout <<"\n= Sample physX simulation with joints=\n" <<endl;
   ors::KinematicWorld ors;
   OpenGL gl, phys_gl; 
  
   createOrs(ors, gl); 

   //! create joint
  
  ors.init("complex_writhe.ors");
 //  ors.init("box.ors");
  ors.getBodyByName("arm1")->type = ors::staticBT;
     //!
   
       
   PhysXInterface physx;    
   physx.G = &ors;
   physx.create();
   
  
   phys_gl.add(glStandardScene, NULL); 
   phys_gl.add(glPhysXInterface, &physx);
   phys_gl.setClearColors(1.,1.,1.,1.);
   phys_gl.camera.setPosition(10.,-15.,8.);
   phys_gl.camera.focus(0,0,1.);
   phys_gl.watch();
    
  for(uint t=0; t<1000; t++) {
   
    physx.step(); 
    gl.update();
    phys_gl.update();
  }
}

void problem3(){   
  cout <<"\n= Sample physX simulation with joints and kinematic objects=\n" <<endl;
   ors::KinematicWorld ors;
   OpenGL gl, phys_gl; 
   ors.init("complex_writhe.ors");
  
   ors.getBodyByName("center")->type = ors::kinematicBT;
    ors.getBodyByName("arm1")->type = ors::kinematicBT;
    
   gl.add(glStandardScene,NULL);
   gl.add(ors::glDrawGraph,&ors);
   gl.setClearColors(1.,1.,1.,1.);
   gl.camera.setPosition(10.,-15.,8.);
   gl.camera.focus(0,0,1.);
   gl.update();
          
   PhysXInterface physx;    
   physx.G = &ors;
   physx.create();
   
  
   phys_gl.add(glStandardScene, NULL); 
   phys_gl.add(glPhysXInterface, &physx);
   phys_gl.setClearColors(1.,1.,1.,1.);
   phys_gl.camera.setPosition(10.,-15.,8.);
   phys_gl.camera.focus(0,0,1.);
   phys_gl.watch();
    
  for(uint t=0; t<1000; t++) {
   
    physx.step(); 
    gl.update();
    phys_gl.update();
 
    ors.getBodyByName("center")->X.pos.p[2]= t*5e-3;
    ors.getBodyByName("arm1")->X.pos.p[0] = cos(t*1e-2);
    ors.getBodyByName("arm1")->X.pos.p[1] = sin(t*1e-2);
    //ors.getBodyByName("arm1")->X.pos.p[2] = 1.9+t*5e-3;
    
    //ors.calcShapeFramesFromBodies();
    //ors.calcBodyFramesFromJoints();   
  }
    
}

void problem4(){   
  cout <<"\n=  physX simulation with Schunk arm and hand=\n" <<endl;
   ors::KinematicWorld ors;
   OpenGL gl, phys_gl; 
   ors.init("schunk.ors");
   ors::Joint *jj;
   ors::Body *bb;
    uint i;
 /*
    for_list(Type, jj, ors.joints) { 
       jj->type  = ors::JT_fixed; 
  }
   */
    for_list(Type, bb, ors.bodies){
      if (bb->index>7) bb->type  = ors::dynamicBT; 
     else 
	bb->type = ors::kinematicBT; ;//ors::staticBT; 
  } 
    
   gl.add(glStandardScene,NULL);
   gl.add(ors::glDrawGraph,&ors);
   gl.setClearColors(1.,1.,1.,1.);   
   gl.camera.setPosition(10.,-15.,8.);
   gl.camera.focus(0,0,1.);
   gl.update(); 
           
   PhysXInterface physx;    
   physx.G = &ors;   
   physx.create();
   
   phys_gl.add(glStandardScene, NULL); 
   phys_gl.add(glPhysXInterface, &physx);
   phys_gl.setClearColors(1.,1.,1.,1.);
   phys_gl.camera.setPosition(10.,-15.,8.);
   phys_gl.camera.focus(0,0,1.); 
   phys_gl.watch();
    
  for(uint t=0; t<1000; t++) {
   
    physx.step(); 
    gl.update();
    phys_gl.update();
 
   //ors.getBodyByName("arm1")->X.pos.p[2] = 1.9+t*5e-3;
    
    //ors.calcShapeFramesFromBodies();
    //ors.calcBodyFramesFromJoints();   
  }
    
}


int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  int mode=MT::getParameter<int>("mode");
  switch(mode){
  case 1:  problem1();  break;
  case 2:  problem2();  break;
  case 3:  problem3();  break;
  case 4:  problem4();  break;
  default: NIY;
  }
  return 0; 
}
