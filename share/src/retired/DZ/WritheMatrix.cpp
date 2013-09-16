#include <Ors/ors.h>
#include <Core/array.h>

void GetWritheSegment(double& writhe,const ors::Vector& A,const ors::Vector& B,const ors::Vector& C,const ors::Vector& D){
//  writhe between vectors AB and CD
ors::Vector r_ac = A-C;
ors::Vector r_ad = A-D;
ors::Vector r_bc = B-C;
ors::Vector r_bd = B-D;
//
ors::Vector n_a =  (r_ac^r_ad)/ (r_ac^r_ad).length();
ors::Vector n_b =  (r_ad^r_bd)/ (r_ad^r_bd).length();
ors::Vector n_c =  (r_bd^r_bc)/ (r_bd^r_bc).length();
ors::Vector n_d =  (r_bc^r_ac)/ (r_bc^r_ac).length();
/// TRICK
double eps = 1e-10; n_a*=(1.0 - eps);n_b*=(1.0 - eps);n_c*=(1.0 - eps);n_d*=(1.0 - eps);/// trick against greater then 1.0 values

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
/// TRICK
B*=(1.0+1e-10); D*=(1.0+1e-10);/// trick against greater then 1.0 values

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

arr J1,J2,J3,J4;
J1 = Jr(r_ac, r_ad, r_bd, 0, 0, 1,Jab);
J2 = Jr(r_ad, r_bd, r_bc, 0, 1, 1,Jab);
J3 = Jr(r_bd, r_bc, r_ac, 1, 1, 0,Jab);
J4 = Jr(r_bc, r_ac, r_ad, 1, 0, 0,Jab);

JTscalar=J1+J2+J3+J4;
}

void GetWritheMatrix(arr& WM, const arr& rope1, const arr& rope2,int dim1,int dim2){
 ors::Vector A,B,C,D;
  double writhe; 
   WM = zeros(dim1,dim2);
  for (int i=0;i<dim1;i++) {
   A.set(rope1(i,0),rope1(i,1),rope1(i,2));
   B.set(rope1(i+1,0),rope1(i+1,1),rope1(i+1,2));
   for (int j=0;j<dim2;j++) {
    C.set(rope2(j,0),rope2(j,1),rope2(j,2));
    D.set(rope2(j+1,0),rope2(j+1,1),rope2(j+1,2));
    GetWritheSegment(writhe, A,B,C,D);
    WM(i,j)=writhe;
 }} 
 
}

void WritheJacobian(arr& JM, const arr& rope1, const arr& rope2,arr& pointsJ,int dim1,int dim2){
  int total_joint_number =  pointsJ.d1;
  pointsJ.reshape(dim1, 3, total_joint_number);
  
  ors::Vector A,B,C,D;
  arr Jab(2,3,total_joint_number);
  arr jj; 
  JM.resize(dim1*dim2,total_joint_number);
  int cnt=0;
  for (int i=0;i<dim1;i++) {
    /// TODO check it!
    if (i>0) Jab[0] = pointsJ[i-1];
    else Jab[0]=zeros(3,total_joint_number);
    
    Jab[1] = pointsJ[i];
     A.set(rope1(i,0),rope1(i,1),rope1(i,2));
     B.set(rope1(i+1,0),rope1(i+1,1),rope1(i+1,2));
    for (int j=0;j<dim2;j++) {
      C.set(rope2(j,0),rope2(j,1),rope2(j,2));
      D.set(rope2(j+1,0),rope2(j+1,1),rope2(j+1,2));
      GetWritheJacobianSegment(jj, A,B,C,D, Jab);  
      JM[cnt] = jj;
      cnt++;
  }} 
}

/// Scalar experiments
void GetScalarWrithe(arr& WS, const arr& rope1, const arr& rope2,int dim){
  arr Matr,vect;
  double scalar=0.;
   WS.resize(1);//=zeros(dim); //(1,1);
   WS.setZero();
  GetWritheMatrix(Matr,rope1,rope2,dim,dim);
  ///diagonal
//   Matr.reshape(dim,dim);
//   for (int k=0;k<dim;k++){
//     WS(0) += Matr(k,k);
//   }

  /// all
  Matr.reshape(dim*dim);
  for (int k=0;k<dim*dim;k++){
    WS(0) += Matr(k);
  }
  ///diagonal
  /// diagonal vector
/*  Matr.reshape(dim,dim);
  for (int k=0;k<dim;k++){
     WS(k) = Matr(k,k);
  }*/
  ///end of  vector
// WS =vect;
}

void ScalarJacobian(arr& SJ, const arr& rope1, const arr& rope2,arr& pointsJ,int dim){
  int total_joint_number =  pointsJ.d1;
  arr MatrJ;
  WritheJacobian(MatrJ,rope1,rope2,pointsJ,dim,dim);
  SJ.resize(1,total_joint_number);//SJ.resize(1,total_joint_number);
  SJ.setZero();
  ///diagonal
//   MatrJ.reshape(dim,dim,total_joint_number);
//   for (int i=0;i<dim;i++) {
//     SJ[0]() += MatrJ[i][i];
//   } 
  ///all
  MatrJ.reshape(dim*dim,total_joint_number);
  for (int i=0;i<dim*dim;i++) {
    SJ[0]() += MatrJ[i]();
  } 
    ///diagonal
  /// diagonal vector
/*   for (int i=0;i<dim;i++) {
     SJ[i]() = MatrJ[i][i];
   }*/
  ///end of vector
//  CHECK(SJ.nd==2 && SJ.d1==total_joint_number,"dimensions!");
}