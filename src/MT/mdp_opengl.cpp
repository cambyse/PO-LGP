#include "mdp.h"

#ifdef MT_FREEGLUT
#  include "opengl.h"
#  include "plot.h"

void mdp::showMaze(){
  byteA &maze=global_maze;
  byteA img(maze.d0,maze.d1,3);
  uint x,y,dx=maze.d1,dy=maze.d0;
  for(x=0;x<dx;x++) for(y=0;y<dy;y++){
    if(maze(y,x)==0){ img(y,x,0)=255; img(y,x,1)=255; img(y,x,2)=255; }
    else if(maze(y,x)==1){ img(y,x,0)=0;   img(y,x,1)=0;   img(y,x,2)=0;   }
    else if(maze(y,x)==2){ img(y,x,0)=255; img(y,x,1)=0;   img(y,x,2)=0;   }
    else if(maze(y,x)==3){ img(y,x,0)=0;   img(y,x,1)=255; img(y,x,2)=0;   }
    else if(maze(y,x)==4){ img(y,x,0)=0;   img(y,x,1)=0;   img(y,x,2)=255; }
    else HALT("strange global maze");
  }
  OpenGL gl;
  gl.watchImage(img,true,10);
}

void mix(byteA& A,const byteA& B,float f=.5){
  if(f>1.) f=1.; else if(f<0.) f=0.;
  A(0)=(1.-f)*A(0)+f*B(0);
  A(1)=(1.-f)*A(1)+f*B(1);
  A(2)=(1.-f)*A(2)+f*B(2);
}

void mdp::showAB(const arr& alpha,const arr& beta){
  byteA &maze=global_maze;
  byteA img(maze.d0,maze.d1,3);
  uint x,y,dx=maze.d1,dy=maze.d0;
  for(x=0;x<dx;x++) for(y=0;y<dy;y++){
    if(maze(y,x)==0){ img(y,x,0)=255; img(y,x,1)=255; img(y,x,2)=255; }
    else if(maze(y,x)==1){ img(y,x,0)=0;   img(y,x,1)=0;   img(y,x,2)=0;   }
    else if(maze(y,x)==2){ img(y,x,0)=0;   img(y,x,1)=0;   img(y,x,2)=255; }
    else if(maze(y,x)==3){ img(y,x,0)=255; img(y,x,1)=0;   img(y,x,2)=0;   }
    else if(maze(y,x)==4){ img(y,x,0)=0;   img(y,x,1)=255; img(y,x,2)=0;   }
    else HALT("strange global maze");
  }
  img.reshape(alpha.N,3);
  double aM=alpha.max(),bM=beta.max();
  for(x=0;x<alpha.N;x++) if(alpha(x)) mix(img[x](),ARRAY<byte>(0,0,255), alpha(x)/aM);
  for(x=0;x<alpha.N;x++) if(beta (x)) mix(img[x](),ARRAY<byte>(255,0,0), beta (x)/bM);
  img.reshape(maze.d0,maze.d1,3);
  static OpenGL *gl=NULL;
  if(!gl) gl=new OpenGL;
  gl->watchImage(img,false,10);
}

void mdp::plotPolicyAndValue(const arr& pi,const arr& V,const MDP& mdp,bool wait){
  plotOpengl();
  plotModule.colors=false;
  //plotModule.grid=true;
  
  //glDisplayRedBlue(V,global_maze.d0,global_maze.d1,false);
  arr tmp;
  uintA tmpI;
  plotClear();
  tmp=~V;
  tmp *= .8/tmp.max();
  tmp.reshape(global_maze.d0,global_maze.d1);
  plotSurface(tmp);
  
  getMaxPxxMap(tmpI,mdp.Pxax,pi);
  tmpI.reshape(global_maze.d0,global_maze.d1);
  plotMatrixFlow(tmpI,.25);
  plot(wait);
}

#else

void mdp::showMaze(){ MT_MSG("display only implemented when compiling with freeglut"); }
void mdp::showAB(const arr& alpha,const arr& beta){}
void mdp::plotPolicyAndValue(const arr& pi,const arr& V,const MDP& mdp,bool wait){}
void mdp::glDisplayGrey   (const arr &x,uint d0,uint d1,bool wait,uint win){}
void mdp::glDisplayRedBlue(const arr &x,uint d0,uint d1,bool wait,uint win){}

#endif
