#include <MT/soc.h>
#include <MT/ors.h>
#include <MT/socSystem_ors.h>
#include <MT/soc_inverseKinematics.h>
#include <MT/opengl.h>
#include <MT/util.h> 
#include <MT/aico.h>  
#include <MT/plot.h>     
#include <DZ/WritheMatrix.h>        
#include <DZ/aico_key_frames.h>    
#include <DZ/WritheTaskVariable.h>  
#include <MT/specialTaskVariables.h> 
#include <sstream>        
#include <z.cpp>           
  
         
            

int problem4(){      
  ors::Graph ors;  
  ors.init(MT::getParameter<MT::String>("orsfile",MT::String("rope_grasp_20.ors")));
  SwiftInterface swift;
  swift.init(ors,.5);   
  OpenGL gl;  
  gl.add(glStandardScene); 
  gl.add(ors::glDrawGraph,&ors); 
  gl.camera.setPosition(5,-10,10); 
  gl.camera.focus(0,0,1);
  gl.watch("loaded configuration - press ENTER");
  uint T=50; 
  soc::SocSystem_Ors soc;      
  soc.os=&std::cout;

 double eps=1e-1; //5e-3;  
 arr q,yy,x0;      
 int wrsize=20;//20;//11
 int jsize=ors.getJointStateDimension(); //20;//11
   
    
 yy = zeros(wrsize,wrsize);              
 
 arr ID; ID=ones(jsize,1);  
 for (int z=0;z<jsize;z++) ID(z,0)= z+1;//11-z;//z+1;
 soc.initBasics(&ors,&swift,&gl,T,2.0,true,&ones(jsize,1).reshape(jsize) ); //Fix time
 // soc.initBasics(&ors,&swift,&gl,T,2.,true,&ID.reshape(wsize)); //Fix time
         
  soc.getx0(x0);    
//   ifstream qitstr(ss.str().c_str()); yy.readRaw(qitstr); qitstr.close(); 
       
   WritheTaskVariable *wr = new WritheTaskVariable("writhe",ors,"rope",wrsize,1);
  wr->y_prec=1e-3;//1e-0;/ / 
  wr->y.reshape(wrsize,wrsize);  //for (int tp=0;tp< wrsize;tp++)  wr->y(tp, wrsize-1)=0;      
  wr->y_target =zeros(wrsize,wrsize);//wr->y;//zeros(10,10);//yy;   
  wr->setInterpolatedTargetsEndPrecisions(T,eps,eps,0.,eps);  
  MT::Array<TaskVariable*> Tlist;      
   
  Tlist.append(wr);
  soc.setTaskVariables(Tlist);   
   
    
 arr b,Binv,R,r;             
 int cnt;        
 soc.setx(x0);
 OneStepDynamicFull(b,Binv,cnt,soc,4.,5e-2,1e-5,1e-3,0,false);
// cout<<wr->J<<endl;       
 double tm;   
 soc.displayState(&b);    
//cout <<b;
soc.gl->watch();   
    soc.setx(x0);
    AICO aico(soc); 
    aico.iterate_to_convergence();  
    q = aico.q;   
    for (;;) soc.displayTrajectory(q,NULL,T,"AICO (planned trajectory)");
  soc.gl->watch();    
// plot_writhe(wr->y,wrsize);
 
}
 
void gradient_check()
{
  WritheGradientCheck(); 
}
//===========================================================================

int main(int argn,char **argv){
  MT::initCmdLine(argn,argv); 
 
  int mode=MT::getParameter<int>("mode");
  switch(mode){
    case 4:  problem4();  break;
    case 1:  gradient_check(); break;
  default: NIY;
  }
  return 0;
}

