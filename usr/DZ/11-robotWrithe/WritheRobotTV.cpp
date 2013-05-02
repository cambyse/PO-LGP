#include "WritheRobotTV.h"
#include "DZ/WritheMatrix.h"
#include <sstream>
#include "MT/plot.h"

WritheTaskVariable::WritheTaskVariable(const char* _name,
                                         ors::Graph& _ors,
                                         const char* _obj_name,
				         int _segments,
				         int _param){
  obj_name=_obj_name;
  param=_param;
  segments=_segments;
  set(_name, _ors, userTVT, -1, ors::Transformation(), -1, ors::Transformation(), ARR());
}


void plot_writhe(arr WM,int dim)
{
 WM.reshape(dim,dim);
 plotGnuplot(); 
 plotClear();
 plotSurface( WM );  
 plot(false);
}

void GetRopes(arr& r1,arr& r2,ors::Graph& _ors,int rope_points,const char* _name){
  //// TODO change it all!!!
  
  arr rope1=arr(rope_points,3); 
  arr rope2=arr(rope_points,3);
  arr ty;
  
  uint start_body = 2; 
  ors::Vector rel; rel.set(0.,0.,0.0);// rel.setZero();
  
  for (int i=0;i<rope_points;i++) {// start with second body part
      _ors.kinematicsPos(ty,i+start_body,  &rel); 
      rope1[i]() = ty;;    
  } 
   arr xtarget;  
  for (int i=0;i<rope_points;i++){
   std::stringstream ss;
   ss << "ring" << i;
   xtarget.setCarray(_ors.getShapeByName(ss.str().c_str())->X.pos.p,3);
   rope2[i]()= xtarget;
  }
  r1=rope1;
  r2=rope2;
 //cout<<rope1<<endl;
// cout<<rope2<<endl;
}
//! Matrix

void WritheTaskVariable::userUpdate(){
    arr rope1,rope2,yy,Jp,JM,points;
    GetRopes(rope1,rope2,*this->ors,segments+1,obj_name);
    GetWritheMatrix(yy,rope1,rope2,segments);
    y=zeros(segments,segments);
    y=yy;
 // cout<<yy<<endl; // TODO LAST zero!!!
 
    int wrsize = (segments)* (segments);
    y.reshape(wrsize); 
    
    ///////////Jacobian
      for (int k=0;k<segments;k++){
       this->ors->jacobianPos(Jp,k+2,&ors::Vector(0.,0.,.0)); // Zero jacobian? +1
	 points.append(Jp);
       } 
       
     //  cout <<"point jacobian" <<points<< endl;
     WritheJacobian(JM,rope1,rope2,points,segments);  
     
     //cout <<"final jacobian"<<JM<<endl;
       J = zeros(wrsize,ors->getJointStateDimension());
    //  for (int k=0;k<wrsize;k++)
	//for (int p=0;p<segments;p++)
	//for (int p=segments-16;p<segments;p++)
	//   J(k,p)=JM(k,p-1); // TODO check it!!
	//    J(k,p+2)=JM(k,p); 
 J=JM;
transpose(Jt,J);
}

//!end of matrix


//! Scalar
// void WritheTaskVariable::userUpdate(){
//     arr rope1,rope2,yy,Jp,JM,points;
//     GetRopes(rope1,rope2,*this->ors,segments+1,obj_name);
//     GetScalarWrithe(yy,rope1,rope2,segments);
//    // y=zeros(1,1);//zeros(segments,segments);
//     y=yy;
//    // int wrsize = (segments)* (segments);
//   //  y.reshape(wrsize); 
// //cout <<y<<endl;
//     ///////////Jacobian
//       for (int k=0;k<segments;k++){
//        this->ors->jacobianPos(Jp,k,&ors::Vector(0.,0.,.1)); // Zero jacobian? +1
// 	 points.append(Jp);
//        } 
//        ScalarJacobian(JM,rope1,rope2,points,segments);  
//       // cout<<JM<<endl; // TODO LAST zero!!!
//       
//     //   J = zeros(1,ors->getJointStateDimension());
//    //   for (int k=0;k<wrsize;k++)
// 	//for (int p=1;p<segments;p++)
// 	//for (int p=segments-16;p<segments;p++)
// 	//   J(0,p)=JM(0,p-1); // TODO check it!!
// 	  //  J(k,p)=JM(k,p-1); 
//    J=JM;
// transpose(Jt,J);
// }
//! End of scalar

void WritheTaskVariable::epsilon_check(arr& delta_q){
    arr rope1,rope2,yy,Jp,JM,points,y1,y2;
    GetRopes(rope1,rope2,*this->ors,segments+1,obj_name); 
    GetWritheMatrix(yy,rope1,rope2,segments);
     
//// HERE - small deformation
arr delta_y;
    delta_y=-0.01*ones(segments, segments);
  //  delta_y.reshape(segments-1,segments-1);
  //  for (int u=0;u<segments-1;u++)
   // delta_y(u,u) = 1.1*delta_y(u,u);
    delta_y.reshape(segments* segments);
  //  delta_y= delta_y-y;
   //   delta_y = (y1-y2);
   //  delta_y.reshape((segments-1)* (segments-1));
    
    ///////////Jacobian
  for (int k=0;k<segments;k++){
       this->ors->jacobianPos(Jp,k+1,&ors::Vector(0.,0.,.1)); // Zero jacobian? +1
	 points.append(Jp);
       } 
       WritheJacobian(JM,rope1,rope2,points,segments);  
    //  cout<<sum(JM)<<endl;
       J = zeros(segments*segments,ors->getJointStateDimension());
      for (int k=0;k<(segments-1)* (segments-1);k++){
      //   for (int k=(param)*10;k<(param+1)*10;k++){

	   for (int p=1;p<segments;p++){
	// for (int p=1;p<param+1;p++){
 	  // J(k,params(0)+1)=-JM(k,params(0)+1);
	  //  J(k,p)=JM(k,p);
	  
//J(k,p)=JM(k,p);
	 }
	 }
	//   cout<<J(param*10,param+1)<<endl;
//}
       
      J =JM ;
      // J=-ones(100,11);  

transpose(Jt,J);
///// HERE - pseudo inverse 
arr Jinv;
inverse(Jinv,J*Jt);
arr psJ = Jt*Jinv;
delta_q = psJ * delta_y;
cout<<"\nDelta_q"<<delta_q<<endl;
//cout<<"\nJ="<<J<<endl;

//cout<<"\nSum of Delta_q"<<sum(delta_q)<<endl;
}

void WritheTaskVariable::delta_check(arr& delta_q){
    arr rope1,rope2,yy,Jp,JM,points,y1,y2;
    GetRopes(rope1,rope2,*this->ors,segments+1,obj_name); 
    GetScalarWrithe(yy,rope1,rope2,segments);
//// HERE - small deformation
    arr delta_y;
    delta_y = ARR(-0.1);

    ///////////Jacobian
      for (int k=0;k<segments;k++){
       this->ors->jacobianPos(Jp,k,&ors::Vector(0.,0.,.1)); // Zero jacobian? +1
	 points.append(Jp);
       } 
       ScalarJacobian(J,rope1,rope2,points,segments);  
      transpose(Jt,J);
///// HERE - pseudo inverse 
arr Jinv;
inverse(Jinv,J*Jt);
arr psJ = Jt*Jinv;
delta_q = psJ * delta_y;
cout<<"\nyy"<<yy<<endl;
cout<<"\nDelta_y"<<delta_y<<endl;
cout<<"\nDelta_q"<<delta_q<<endl;
cout<<"\nJ="<<J<<endl;

//cout<<"\nSum of Delta_q"<<sum(delta_q)<<endl;
}

/*


void WritheTaskVariable::userUpdate(){
    arr rope1,rope2,yy,Jp,JM,points;
    GetRopes(rope1,rope2,*this->ors); 
    GetWritheMatrix(yy,rope1,rope2);
      y=zeros(10,10);
      y= y/(y.absMax());
      for (int k=0;k<10;k++){
	//for (int p=0;params(0)+p<10;p++){
	// y(k,param)=yy(k,param);
     //  }
      }

	 y(param,param)=yy(param,param);
       //y=yy;
      y.reshape(100); 

    ///////////Jacobian
      for (int k=0;k<11;k++){
	 //ors->jacobianPos(Jp,k+1,NULL);
         this->ors->jacobianPos(Jp,k+1);
	 points.append(Jp);
       }
       WritheJacobian(JM,rope1,rope2,points);  
    //  cout<<sum(JM)<<endl;
       J = zeros(100,ors->getJointStateDimension());
      for (int k=0;k<100;k++){
      //   for (int k=(param)*10;k<(param+1)*10;k++){

	  // for (int p=1;params(0)+p<11;p++){
	// for (int p=1;p<param+1;p++){
 	  // J(k,params(0)+1)=-JM(k,params(0)+1);
	    J(k,param+1)=JM(k,param+1);
	  
//J(k,p)=JM(k,p);
	 }
	   cout<<J(param*10,param+1)<<endl;
//}
       
      //J =-JM ;
      // J=-ones(100,11);  


transpose(Jt,J);
}
*/
