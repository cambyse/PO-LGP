#include <MT/ors.h>
#include <MT/array.h>


void vectorProduct(arr& x, const arr& y, const arr& z){
  x=ARRAY(0.,0.,0.);
   
  x(0)=y(1)*z(2)-y(2)*z(1);
  x(1)=y(2)*z(0)-y(0)*z(2);
  x(2)=y(0)*z(1)-y(1)*z(0);
}


void GetWritheSegment(double& writhe,const ors::Vector& A,const ors::Vector& B,const ors::Vector& C,const ors::Vector& D){
//  writhe between vectors AB and CD
ors::Vector r_ac = A-C;
ors::Vector r_ad = A-D;
ors::Vector r_bc = B-C;
ors::Vector r_bd = B-D;
//
ors::Vector n_a =  (r_ac^r_ad)/ pow( (r_ac^r_ad).lengthSqr(), 0.5);
ors::Vector n_b =  (r_ad^r_bd)/ pow( (r_ad^r_bd).lengthSqr(), 0.5);
ors::Vector n_c =  (r_bd^r_bc)/ pow( (r_bd^r_bc).lengthSqr(), 0.5);
ors::Vector n_d =  (r_bc^r_ac)/ pow( (r_bc^r_ac).lengthSqr(), 0.5);
//
 writhe = asin(n_a*n_b) + asin(n_b*n_c) + asin(n_c*n_d)+asin(n_d*n_a);
}

//column-wise cross-product
arr cwcp(const arr& J, const ors::Vector& v){
  arr z,Jt;
  Jt = ~J;
  ors::Vector temp;
  z.resizeAs(Jt);
  for(uint i=0;i<z.d0;i++){
    temp.set(Jt[i].p);
    z[i] = ARRAY(temp ^ v);
  }
  z = ~z;
  return z;
}

// returns 1D jacobian part 
arr Jr(ors::Vector& r1,ors::Vector& r2,ors::Vector& r3, uint d1, uint d2, uint d3, const arr& Jab){
double B,D;
arr A_prime, B_prime, C_prime, D_prime,A,C;
uint n=Jab.d2;
A = ARRAY(r1^r2); B=(r1^r2).length(); C=ARRAY(r2^r3); D=(r2^r3).length();
A_prime = cwcp(Jab[d1], r2) - cwcp(Jab[d2], r1);
B_prime = ~A*A_prime/B;
C_prime = cwcp(Jab[d2], r3) - cwcp(Jab[d3], r2);
D_prime = ~C*C_prime/D;
arr Jr = 1.0 / sqrt(1.0 - pow( (sum(~A*C)/(B*D)),2) ) * ( ~C * (A_prime*B - A*B_prime)/(pow(B,2)*D) + ~A*(C_prime*D - C*D_prime)/(pow(D,2)*B)  )  ;
CHECK(Jr.d0==1 && Jr.d1==n,"");
return Jr;
}

void GetWritheJacobianSegment(arr& JTscalar,const ors::Vector& p_a,const ors::Vector& p_b,const ors::Vector& p_c,const ors::Vector& p_d, const arr& Jab){
  CHECK(Jab.nd==3 && Jab.d0==2 && Jab.d1==3,"");
  uint n=Jab.d2;
  
  //  writhe between vectors AB and CD
ors::Vector r_ac = p_a-p_c;
ors::Vector r_ad = p_a-p_d;
ors::Vector r_bc = p_b-p_c;
ors::Vector r_bd = p_b-p_d;

//
ors::Vector n_a =  (r_ac^r_ad) / (r_ac^r_ad).length();
ors::Vector n_b =  (r_ad^r_bd) / (r_ad^r_bd).length();
ors::Vector n_c =  (r_bd^r_bc) / (r_bd^r_bc).length();
ors::Vector n_d =  (r_bc^r_ac) / (r_bc^r_ac).length();

//  
arr J1,J2,J3,J4;
//cout<<n_a<<"_"<<n_b<<"_"<<n_c<<"_"<<n_d<<"_"<<endl;
ors::Vector A,C;
double B,D;
arr A_prime, B_prime, C_prime, D_prime;
//double Jwrithe = 1.0 / sqrt(1.0 - pow((A/B *C /D ),2) )  *( (A_prime*B - A^B_prime)/pow(B,2)*(C/D) + (A/B)*(C_prime*D - C^D_prime)/pow(D,2)  )  ;

// J1 caculation n_a,n_b
// A = r_ac^r_ad; B=(r_ac^r_ad).length(); C=r_ad^r_bd; D=(r_ad^r_bd).length();
// A_prime = cwcp(Jab[0], r_ad) - cwcp(Jab[0], r_ac);
// B_prime = ARRAY(n_a)*A_prime;
// C_prime = cwcp(Jab[0], r_bd) - cwcp(Jab[1], r_ad);
// D_prime = ARRAY(n_b)*C_prime;
// J1 = 1.0 / sqrt(1.0 - pow(((A*C)/(B*C)),2) )  * ( ~ARRAY(C) * (A_prime*B - ARRAY(A)*B_prime))/(pow(B,2)*D) + ~ARRAY(A)*(C_prime*D - ARRAY(C)*D_prime)/(pow(D,2)*B)  )  ;
// CHECK(J1.d0==1 && J.d1==n,"");

J1 = Jr(r_ac, r_ad, r_bd, 0, 0, 1,Jab);
J2 = Jr(r_ad, r_bd, r_bc, 0, 1, 1,Jab);
J3 = Jr(r_bd, r_bc, r_ac, 1, 1, 0,Jab);
J4 = Jr(r_bc, r_ac, r_ad, 1, 0, 0,Jab);
 
// //
// A = r_ad^r_bd;B = pow( (r_ad^r_bd).lengthSqr(), 0.5); C=r_bd^r_bc; D=pow( (r_bd^r_bc).lengthSqr(), 0.5);
// A_prime= r_ad^pointJ; B_prime = n_b^A_prime; C_prime = pointJ^r_bc +r_bd^pointJ; D_prime = n_c^C_prime;
// J2 = 1.0 / sqrt(1.0 - pow((A/B *C /D ),2) )  *( (A_prime*B - A^B_prime)/pow(B,2)*(C/D) + (A/B)*(C_prime*D - C^D_prime)/pow(D,2)  )  ;
// //
// A = r_bd^r_bc; B= pow( (r_bd^r_bc).lengthSqr(), 0.5);C =r_bc^r_ac; D =  pow( (r_bc^r_ac).lengthSqr(), 0.5);
// A_prime=pointJ^r_bc +r_bd^pointJ; B_prime = n_c^A_prime; C_prime = pointJ^r_ac; D_prime = n_d^C_prime;
// J3 = 1.0 / sqrt(1.0 - pow((A/B *C /D ),2) )  *( (A_prime*B - A^B_prime)/pow(B,2)*(C/D) + (A/B)*(C_prime*D - C^D_prime)/pow(D,2)  )  ;
// //
// A = r_bc^r_ac; B= pow( (r_bc^r_ac).lengthSqr(), 0.5); C=r_ac^r_ad; D=pow( (r_ac^r_ad).lengthSqr(), 0.5);
// A_prime = pointJ^r_ac; B_prime=n_d^A_prime; C_prime.set(0,0,0); D_prime.set(0,0,0);
// J4 = 1.0 / sqrt(1.0 - pow((A/B *C /D ),2) )  *( (A_prime*B - A^B_prime)/pow(B,2)*(C/D) + (A/B)*(C_prime*D - C^D_prime)/pow(D,2)  )  ;

JTscalar=J1+J2+J3+J4;
}

void GetWritheMatrix(arr& WM, const arr& rope1, const arr& rope2,int dim){
 ors::Vector A,B,C,D;
  double writhe; 
   WM = zeros(dim,dim);
  for (int i=0;i<dim;i++) {
   A.set(rope1(i,0),rope1(i,1),rope1(i,2));
   B.set(rope1(i+1,0),rope1(i+1,1),rope1(i+1,2));
   for (int j=0;j<dim;j++) {
    C.set(rope2(j,0),rope2(j,1),rope2(j,2));
    D.set(rope2(j+1,0),rope2(j+1,1),rope2(j+1,2));
    GetWritheSegment(writhe, A,B,C,D);
    WM(i,j)=writhe;
 }} 
 
}

void WritheJacobian(arr& JM, const arr& rope1, const arr& rope2,arr& pointsJ,int dim){
  int total_joint_number =  pointsJ.d1;
  pointsJ.reshape(dim, 3, total_joint_number);
  
  ors::Vector A,B,C,D;
  arr Jab(2,3,total_joint_number);
  arr jj; 
  JM.resize(dim*dim,total_joint_number);
  int cnt=0;
  for (int i=0;i<dim;i++) {
    
    if (i>0) Jab[0] = pointsJ[i-1];
    else Jab[0]=zeros(3,total_joint_number);
    
    Jab[1] = pointsJ[i];
     A.set(rope1(i,0),rope1(i,1),rope1(i,2));
     B.set(rope1(i+1,0),rope1(i+1,1),rope1(i+1,2));
    for (int j=0;j<dim;j++) {
      C.set(rope2(j,0),rope2(j,1),rope2(j,2));
      D.set(rope2(j+1,0),rope2(j+1,1),rope2(j+1,2));
      GetWritheJacobianSegment(jj, A,B,C,D, Jab);  
      JM[cnt] = jj;
      cnt++;
  }} 
}

//! Scalar experiments
void GetScalarWrithe(arr& WS, const arr& rope1, const arr& rope2,int dim){
  arr Matr,vect;
  double scalar=0.;
   WS.resize(1);//=zeros(dim); //(1,1);
   WS.setZero();
  GetWritheMatrix(Matr,rope1,rope2,dim);
  //!diagonal
  Matr.reshape(dim,dim);
  for (int k=0;k<dim;k++){
    WS(0) += Matr(k,k);
  }

  //! all
//   Matr.reshape(dim*dim);
//   for (int k=0;k<dim*dim;k++){
//     WS(0) += Matr(k);
//   }
  //!diagonal
  //! diagonal vector
/*  Matr.reshape(dim,dim);
  for (int k=0;k<dim;k++){
     WS(k) = Matr(k,k);
  }*/
  //!end of  vector
// WS =vect;
}

void ScalarJacobian(arr& SJ, const arr& rope1, const arr& rope2,arr& pointsJ,int dim){
  int total_joint_number =  pointsJ.d1;
  arr MatrJ;
  WritheJacobian(MatrJ,rope1,rope2,pointsJ,dim);
  SJ.resize(1,total_joint_number);//SJ.resize(1,total_joint_number);
  SJ.setZero();
  //!diagonal
  MatrJ.reshape(dim,dim,total_joint_number);
  for (int i=0;i<dim;i++) {
    SJ[0]() += MatrJ[i][i];
  } 
  //!all
//   MatrJ.reshape(dim*dim,total_joint_number);
//   for (int i=0;i<dim*dim;i++) {
//     SJ[0]() += MatrJ[i]();
//   } 
    //!diagonal
  //! diagonal vector
/*   for (int i=0;i<dim;i++) {
     SJ[i]() = MatrJ[i][i];
   }*/
  //!end of vector
//  CHECK(SJ.nd==2 && SJ.d1==total_joint_number,"dimensions!");
}