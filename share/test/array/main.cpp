#include<algorithm>
#include<MT/array.h>
#include<MT/util.h>
//#include<MT/algos.h>

using namespace std;

bool DoubleComp(const double& a,const double& b){ return a<b; }

void testBasics(){
  cout <<"\n*** basic manipulations\n";
  arr a;     //'arr' is a macro for MT::Array<double>
  intA ints; //a macro for MT::Array<int>

  a.resize(7,10);
  double* i;
  for(i=a.p;i!=a.pstop;i++) *i=i-a.p; //assign pointer offsets to entries
  cout <<"\narray filled with pointer offsets (-> memory is linear):\n" <<a;
  cout <<"\nsubarray (of the original) [2:4,:] (in MATLAB notation)\n" <<a.sub(2,4,0,-1);

  //fancy writing:
  cout <<"\nfancy writing with dimensionality tag:" <<endl;
  a.write(cout,"-"," NEWLINE\n","{}",true);
  
  //deleting rows/columns
  a.delRows(1);
  cout <<"\nrow 1 deleted:\n" <<a;
  a.delColumns(1,2);
  cout <<"\n2 columns deleted at 1:\n" <<a;
  a.insColumns(1,3);
  cout <<"\n3 columns inserted at 1:\n" <<a;

  //access:
  cout <<"\n3rd line:\n" <<a[2] <<endl; //gets a const-version of the []-subarray
  a[2](1)=7.; //same as a(2,1)=7 (but much slower)
  a[3]()+=1.; //use operator() to get a non-const &-version of the []-subarray 
  a[1]()=a[2];
  cout <<"\nrows manipulated:\n" <<a;

  //setting arrays ``by hand''
  a = ARR(0, 1, 2, 3, 4); //ARR() is equivalent to ARRAY<double>()
  cout <<"\nset by hand:\n" <<a <<endl;
  ints = ARRAY<int>(0, -1, -2, -3, -4);
  cout <<"set by hand:\n" <<ints <<endl;

  //TRY DEBUGGING with GDB:
  //set a breakpoint here
  //in gdb (or a watch console of your IDE) type 'print gdb(a)' and 'print gdb(ints)'
  
  //randomization
  rndUniform(a,-1.,1,false); //same as   forall(i,a) *i=rnd.uni(-1,1);
  cout <<"\nrandom double array:\n" <<a <<endl;
  cout <<"multiplied by 2:\n" <<2.*a <<endl;

  //sorting
  a.sort(DoubleComp);
  //std::sort(a.p,a.pstop,DoubleComp);
  cout <<"\n sorting: sorted double array:\n" <<a <<endl;
  a.insertInSorted(.01,DoubleComp);
  cout <<"\n sorted insert: .01 added:\n" <<a <<endl;

  //commuting I/O-operators:
  a.resize(3,7);
  arr b;
  rndInteger(a,1,9,false);
  cout <<"\nbefore save/load:\n" <<a;

  ofstream of("z.tmp");
  of <<a;
  of.close();

  ifstream inf("z.tmp");
  inf >>b;
  inf.close();

  cout <<"\nafter saved and loaded from a file:\n" <<b <<endl;
  CHECK(a==b,"save-load failed");
}

void testMatlab(){
  arr x = randn(5);
  cout <<"randn(5)" <<x <<endl;

  x = eye(5);
  cout <<"eye(5)" <<x <<endl;

  uintA p = randperm(5);
  cout <<"randperm(5)" <<p <<endl;

  arr A = ARR(1,2,3,4);  A.reshape(2,2);
  arr B = repmat(A,2,3);
  cout <<"A=" <<A <<endl;
  cout <<"repmat(A,2,3)" <<B <<endl;
}

void testException(){
  cout <<"\n*** exception handling\n";
  arr A;
  A.append(10);
  cout <<"accessing our of range..." <<endl;
  try{
    cout <<A(2);
  }catch(const char *e){
    cout <<"exception caught `" <<e <<"'" <<endl;
  }
}

void testMemoryBound(){
  cout <<"\n*** memory bound\n";
  MT::globalMemoryBound=1ull<<20;
  MT::globalMemoryStrict=true;
  arr A;
  try{
    A.resize(1000,1000,1000);
  }catch(...){
    cout <<"caught memory restriction exception..." <<endl;
  }
  A.resize(1000);
  cout <<"total memory allocated = " <<MT::globalMemoryTotal <<endl;
  MT::globalMemoryBound=1ull<<30;
}

void testBinaryIO(){
  cout <<"\n*** acsii and binary IO\n";
  arr a,b; a.resize(10000,100); rndUniform(a,0.,1.,false);

  ofstream fout("z.ascii"),bout("z.bin",ios::binary);
  ifstream fin("z.ascii") ,bin("z.bin",ios::binary);

  MT::timerStart();
  a.write(fout," ","\n","[]",true,false);
  cout <<"ascii save time: " <<MT::timerRead() <<"sec" <<endl;
  fout.close();

  MT::timerStart();
  b.read(fin);
  cout <<"ascii load time: " <<MT::timerRead() <<"sec" <<endl;
  fin.close();

  //CHECK(a==b) would fail because ascii numbers are not exact!

  MT::timerStart();
  a.write(bout,NULL,NULL,NULL,true,true);
  cout <<"binary save time: " <<MT::timerRead() <<"sec" <<endl;
  bout.close();

  MT::timerStart();
  b.read(bin);
  cout <<"binary load time: " <<MT::timerRead() <<"sec" <<endl;
  bin.close();

  CHECK(a==b,"binary IO failed!");
  cout <<"binary IO exactly restores double array and is much faster" <<endl;
}

//#include"expressions.cpp"

void testExpression(){
  cout <<"\n*** matrix expressions\n";
  arr a(2,3),b(3,2),c(3),d;
  rndInteger(a,-5,5,false);
  rndInteger(b,-5,5,false);
  rndInteger(c,-5,5,false);
  cout <<"\nmatrix A\n" <<a;
  cout <<"\nmatrix B\n" <<b;
  cout <<"\nmatrix product A*B\n" <<a*b;
  cout <<"\nvector c\n" <<c <<endl;
  cout <<"\ntranspose of c\n" <<~c;
  cout <<"\ntensor product c^c\n" <<(c^c);
  cout <<"\ncoupled unitary\n" <<1. + .5 * (2.*a) - 1.;
  cout <<"\nlonger expression\n" <<2.*a + 3.*a;
  cout <<"\nlonger expression\n" <<2.*a + ~b;
}

void testPermutation(){
  cout <<"\n*** permutation\n";
  uintA p;
  rnd.seed(3);

  p.setStraightPerm(10);
  cout <<"\ninit:\n" <<p;

  p.permute(2,5);
  cout <<"\npermute(2,5):\n" <<p;

  p.setRandomPerm();
  cout <<"\nrandom:\n" <<p <<endl;

  for(uint i=0;i<p.N;i++) cout <<i <<":" <<p(i) <<"\n";
}

void testGnuplot(){
  cout <<"\n*** gnuplot\n";
  uint i,j;
  arr X(30,30);
  for(i=0;i<X.d0;i++) for(j=0;j<X.d1;j++) X(i,j)=sin(.2*i)*sin(.1*j);
  gnuplot(X);

  X.resize(100);
  for(i=0;i<X.d0;i++) X(i)=sin(.3*i);
  gnuplot(X);

  X.resize(100,2);
  for(i=0;i<X.d0;i++){ X(i,0)=MT_PI*(2./(X.d0-1)*i-1.); X(i,1)=sin(X(i,0)); }
  gnuplot(X);
}

void testDeterminant(){
  cout <<"\n*** determinant computation\n";
  //double A[4]={1.,2.,-2.,3.};
  double B[9]={1,1,2,1,1,0,0,-2,3};
  //arr a; a.copy(A,4); a.resizeCopy(2,2);
  arr a; a.setCarray(B,9); a.reshape(3,3);
  cout <<a <<"det=" <<determinant(a) <<std::endl;
  cout <<"co00=" <<cofactor(a,0,0) <<std::endl;
  cout <<"co10=" <<cofactor(a,1,0) <<std::endl;
}

void testMM(){
  cout <<"\n*** matrix multiplication speeds\n";
  uint M=10000,N=100,O=100;
  arr A(M,N),B(N,O),C,D;
  rndUniform(A,-1,1,false);
  rndUniform(B,-1,1,false);

  cout <<"speed test: " <<M <<'x' <<N <<'x' <<O <<" matrix multiplication..." <<std::endl;

  MT::useLapack=false; 
  MT::timerStart();
  innerProduct(D,A,B);
  cout <<"native time = " <<MT::timerRead() <<std::endl;

  if(MT::lapackSupported){
    MT::useLapack=true;
    MT::timerStart();
    blas_MM(C,A,B);
    cout <<"blas time = " <<MT::timerRead() <<std::endl;
    cout <<"error = " <<sqrDistance(C,D) <<std::endl;
  }else{
    cout <<"LAPACK not installed - only native algorithms" <<std::endl;
  }
}

void testSVD(){
  cout <<"\n*** singular value decomposition\n";
  uint m=1000,n=500,r=2,svdr;
  arr L(m,r),R(r,n),A,U,d,V,D;
  rndUniform(L,-1,1,false);
  rndUniform(R,-1,1,false);
  A=L*R;
  
  cout <<"speed test: " <<m <<'x' <<n <<" (rank=" <<r <<") SVD decomposition..." <<std::endl;

  MT::useLapack=false;
  MT::timerStart();
  svdr=svd(U,d,V,A);
  cout <<"native SVD time = " <<MT::timerRead(); cout.flush();
  D.setDiag(d);
  cout <<" error = " <<norm(A - U*D*~V) <<" rank = " <<svdr <<"("<<r<<")"<<std::endl;

  if(MT::lapackSupported){
    MT::useLapack=true;
    MT::timerStart();
    svdr=svd(U,d,V,A);
    cout <<"lapack SVD time = " <<MT::timerRead(); cout.flush();
    D.setDiag(d);
    cout <<" error = " <<norm(A - U*D*~V) <<" rank = " <<svdr <<"("<<r<<")" <<std::endl;
  }else{
    cout <<"LAPACK not installed - only native algorithms" <<std::endl;
  }
}

void testInverse(){
  cout <<"\n*** matrix inverse\n";
  uint m=500,n=500,svdr;
  arr A(m,n),invA,I;
  rndUniform(A,-1,1,false);
  I.setId(m);
  
  cout <<"speed test: " <<m <<'x' <<n <<" inversion..." <<std::endl;

  MT::useLapack=false;
  MT::timerStart();
  svdr=inverse_SVD(invA,A);
  cout <<"native SVD inverse time = " <<MT::timerRead(); cout.flush();
  uint mi;
  cout <<" error = " <<maxDiff(A*invA,I,&mi) <<" rank = " <<svdr <<std::endl;
  
  /*MT::timerStart();
  MT::inverse_LU(invA,A);
  cout <<"native LU  inverse time = " <<MT::timerRead(); cout.flush();
  cout <<" error = " <<maxDiff(invA*A,I) <<std::endl;*/
  
  if(MT::lapackSupported){
    MT::useLapack=true;
    MT::timerStart();
    svdr=inverse_SVD(invA,A);
    cout <<"lapack SVD inverse time = " <<MT::timerRead(); cout.flush();
    cout <<" error = " <<maxDiff(A*invA,I,NULL) <<" rank = " <<svdr <<std::endl;

    /*MT::timerStart();
    MT::inverse_LU(invA,A);
    cout <<"lapack LU  inverse time = " <<MT::timerRead(); cout.flush();
    cout <<" error = " <<norm(invA*A - I) <<std::endl;*/
  }else{
    cout <<"LAPACK not installed - only native algorithms" <<std::endl;
  }
  
  cout <<"\n*** symmetric matrix inverse\n";
  A.resize(m,m);
  rndUniform(A,-1,1,false);
  A=A*~A;
  I.setId(m);
  
  cout <<"speed test: " <<m <<'x' <<m <<" symmetric inversion..." <<std::endl;

  if(MT::lapackSupported){
    MT::timerStart();
    lapack_inverseSymPosDef(invA,A);
    cout <<"lapack SymDefPos inverse time = " <<MT::timerRead(); cout.flush();
    cout <<" error = " <<maxDiff(A*invA,I,&mi) <<std::endl;
  }else{
    cout <<"LAPACK not installed - only native algorithms" <<std::endl;
  }
}

void testGaussElimintation() {
  cout << "\n*** Gaussian elimination with partial pivoting \n";
  if (MT::lapackSupported) {
    arr A;
    A.append(ARR(7., 2., 4., 3.));
    A.append(ARR(3., 2., 6., 5.));
    A.append(ARR(7., 5., 3., 7.));
    A.reshape(4,3);
    cout << "A = " << A << endl;

    arr b = ARR(9., 5., 2.);
    cout << "b = " << b << endl;

    arr X;
    lapack_mldivide(X, A, b);
    cout << "X = " << endl << X << endl;
  }
}

//--------------------------------------------------------------------------------
//
// alternative operator notation
//

enum ArrayOpType { exProduct, inProduct, elemProduct };
class ArrayOp{
public:
  ArrayOpType type;
  const arr *left,*right;
  ArrayOp(ArrayOpType typ){ type=typ; left=right=0; }
  void assign(arr &x){
    CHECK(left && right,"mist");
    switch(type){
    case inProduct:
      cout <<"inner Product between " <<*left <<" and " <<*right <<endl;
      innerProduct(x,*left,*right);
      return;
    case exProduct:
      cout <<"outer Product between " <<*left <<" and " <<*right <<endl;
      outerProduct(x,*left,*right);
      return;
    case elemProduct:
      cout <<"element-wise Product between " <<*left <<" and " <<*right <<endl;
      mult(x,*left,*right);
      return;
    }
    left=right=0;
    HALT("something went wrong");
    return;
  }
};
ArrayOp &operator%(const arr &z,ArrayOp &op){ op.left=&z; return op; }
ArrayOp &operator%(ArrayOp &op,const arr &z){ op.right=&z; return op; }
ostream &operator<<(ostream &os,ArrayOp &op){ arr x; op.assign(x); os <<x; return os; }
//arr &operator=(arr &x,ArrayOp &op){ op.assign(x); return x; }

#define PROD % ArrayOp(inProduct) %
#define PROD % ArrayOp(inProduct) %
#define MUL  % ArrayOp(elemProduct) %

/*void testNewOp(){
  arr x(2,3),y(3,4);
  rndInt(x,0,5);
  rndInt(y,0,5);
  cout <<x <<y <<x PROD y <<x MUL x;
  //x = x MUL x;
  }*/


void testTensor(){
  cout <<"\n*** tensor manipulations\n";

  arr A,B,C,D;
  uint k;
  for(k=0;k<100;k++){
    //element-wise multiplication
    A.resize(TUP(rnd(3)+1,rnd(3)+1));
    B.resize(A.d1,A.d0);
    rndUniform(A,0.,1.,false);
    rndUniform(B,0.,1.,false);
    C=A;
    tensorMultiply(C,B,TUP(1,0));
    CHECK(C==A%~B,"");
    C=A;
    tensorMultiply_old(C,B,TUP(C.d0,C.d1),TUP(1,0));
    CHECK(C==A%~B,"");
    tensorEquation(C,A,TUP(0,1),B,TUP(1,0),0);
    CHECK(C==A%~B,"");

    //matrix product
    C.resize(A.d0,A.d0);
    tensorEquation(C,A,TUP(0,2),B,TUP(2,1),1);
    CHECK(C==A*B,"");

    C.resize(A.d1,A.d1);
    tensorEquation(C,A,TUP(2,0),B,TUP(1,2),1);
    CHECK(C==~A*~B,"");
  }
  cout <<"\n... tensor tests successful\n";

  //test permutations:
  A.resize(2,3,4);
  rndInteger(A,0.,1.,false);
  tensorPermutation(B,A,TUP(0,1,2));
  cout <<A <<endl <<B <<endl;
}

//--------------------------------------------------------------------------------

char* gdb(MT::Array<double>& a){
  static MT::String buf;
  buf.clear();
  a.writeDim(buf);
  a.writeRaw(buf);
  return buf.p;
}
char* gdb(MT::Array<int>& a){
  static MT::String buf;
  buf.clear();
  a.writeDim(buf);
  a.writeRaw(buf);
  return buf.p;
}

int main(int argc, char *argv[]){
  
  testBasics();
  testMatlab();
//   return 0;
  testException();
  testMemoryBound();
  testBinaryIO();
  testExpression();
  testPermutation();
  testGnuplot();
  testDeterminant();
  testInverse();
  testMM();
  testSVD();
  testTensor();
  testGaussElimintation();
  
  cout <<"\n ** total memory still allocated = " <<MT::globalMemoryTotal <<endl;
  
  return 0;
}

