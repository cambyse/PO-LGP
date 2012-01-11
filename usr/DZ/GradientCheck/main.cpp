#include <DZ/WritheMatrix.h>        
#include <DZ/aico_key_frames.h>    
#include <DZ/WritheTaskVariable.h>  

#include <z.cpp>           
    

void gradient_check()   
{     
  WritheGradientCheck();     
}

void gradient_scalar_check()   
{ 
  GradientScalarCheck();   
}
//===========================================================================
   
int main(int argn,char **argv){
  MT::initCmdLine(argn,argv);                  
  
  int mode=MT::getParameter<int>("mode");   
  switch(mode){
       case 1:  gradient_check(); break;
       case 2:  gradient_scalar_check(); break;
  default: NIY;
  }           
  return 0;
}
   
