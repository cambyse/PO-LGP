#include <algorithm>
#include <Core/util.h>
#include <Core/array.h>
#include "array_cheatsheet.h"
#include <Core/array-vector.h>

using namespace std;

bool DoubleComp(const double& a,const double& b){ return a<b; }

void TEST(Basics){
  cout <<"\n*** basic manipulations\n";
  arr a;     //'arr' is a macro for MT::Array<double>
  intA ints; //a macro for MT::Array<int>

  a.resize(7,5);
  double *ap=a.p, *astop=ap+a.N;
  for(; ap!=astop; ap++) *ap=ap-a.p; //assign pointer offsets to entries
  cout <<"\narray filled with pointer offsets (-> memory is linear):\n" <<a <<endl;
  cout <<"\nsubarray (of the original) [2:4,:] (in MATLAB notation)\n" <<a.sub(2,4,0,-1) <<endl;
  CHECK(a.last()==a.N-1,"");

  //easier looping:
  cout <<"\neasier looping:\n";
  for(double& e: a) e++;
  cout <<a <<endl;

  //fancy writing:
  cout <<"\nfancy writing with dimensionality tag:" <<endl;
  a.write(cout,"-"," NEWLINE\n","{}",true);
  
  //deleting rows/columns
  a.delRows(1);
  cout <<"\n\nrow 1 deleted:\n" <<a <<endl;
  a.delColumns(1,2);
  cout <<"\n2 columns deleted at 1:\n" <<a <<endl;
  a.insColumns(1,3);
  cout <<"\n3 columns inserted at 1:\n" <<a <<endl;
  CHECK(a.d1==6,"");
  CHECK(a(0,1)==0,"non-zeros inserted");

  //access:
  cout <<"\n3rd line:\n" <<a[2] <<endl; //gets a const-version of the []-subarray
  a[2](1)=7.; //same as a(2,1)=7 (but much slower)
  a[3]()+=1.; //use operator() to get a non-const &-version of the []-subarray 
  a[1]()=a[2];
  cout <<"\nrows manipulated:\n" <<a <<endl;
  CHECK(a(2,1)==7.,"");
  CHECK(a[1]==a[2],"");

  //setting arrays ``by hand''
  a = ARR(0, 1, 2, 3, 4); //ARR() is equivalent to ARRAY<double>()
  cout <<"\nset by hand:\n" <<a <<endl;
  ints = ARRAY<int>(0, -1, -2, -3, -4);
  cout <<"\nset by hand:\n" <<ints <<endl;
  copy(a, ints); //copying between different types
  CHECK(a(2)==-2,"");

  //TRY DEBUGGING with GDB:
  //set a breakpoint here
  //in gdb (or a watch console of your IDE) type 'print gdb(a)' and 'print gdb(ints)'
  
  //randomization
  rndUniform(a,-1.,1,false); //same as   forall(i,a) *i=rnd.uni(-1,1);
  cout <<"\nrandom double array:\n" <<a <<endl;

  //sorting
  a.sort(DoubleComp);
  //std::sort(a.p,a.pstop,DoubleComp);
  cout <<"\n sorting: sorted double array:\n" <<a <<endl;
  a.insertInSorted(.01,DoubleComp);
  cout <<"\n sorted insert: .01 added:\n" <<a <<endl;
  for(uint i=0;i<a.N-1;i++) CHECK(a(i)<=a(i+1),"not sorted!");

  //commuting I/O-operators:
  a.resize(3,7,2);
  arr b;
  rndInteger(a,1,9,false);
  cout <<"\nbefore save/load:\n " <<a <<endl;

  FILE("z.tmp") <<a;

  FILE("z.tmp") >>b;

  cout <<"\nafter saved and loaded from a file: " <<b <<endl;
  CHECK_ZERO(maxDiff(a,b), 1e-4, "non-exact save load");
}

void TEST(StdVectorCompat) {
  std::vector<double> x(3);
  x[0]=1.;
  cout <<"std::vector to arr:" <<ARRAY(x) <<endl;
  arr y(10);
  y.setStraightPerm(10);
  x=VECTOR(y);
  cout <<"arr -> std::vector -> arr = " <<ARRAY(x) <<endl;
}

void TEST(SimpleIterators) {
  // This test shows how to use the iterators

  cout << "*** Iterate linearly through the memory of an array (1D) - const" << endl;
  arr A = randn(9, 1);
  // notice: we're using const here
  for (const auto& elem : A) {
    cout << elem << endl;
  }

  cout << "*** increment each element" << endl;
  // notice: we DON'T use const here
  for (auto& elem : A) { elem += 1; }

  cout << "*** Iterate linearly through the memory of an array (2D)" << endl;
  A.reshape(3, 3);
  for (const auto& elem : A) {
    cout << elem << endl;
  }

  cout << "*** Iterate linearly through the memory of an array (3D)" << endl;
  arr C = randn(1, 8);
  C.reshape(TUP(2, 2, 2));
  for (const auto& elem : C) {
    cout << elem << endl;
  }
}

void TEST(InitializationList) {
  cout << "Use c++11 initialization list to initialize arrays by hand" << endl;
  arr a = {1, 3, 2, 5};
  cout << a << endl;
}

void TEST(RowsAndColumsAccess) {
  // access rows and columns easily
  arr A = eye(3);

  cout << "\nAccessing single rows" << endl;
  cout << A.row(0) << endl;
  cout << A.row(1) << endl;
  cout << A.row(2) << endl;

  cout << "\nAccessing single columns" << endl;
  cout << A.col(0) << endl;
  cout << A.col(1) << endl;
  cout << A.col(2) << endl;

  cout << "\nAccessing multiple rows" << endl;
  cout << A.rows(0, 2) << endl;
  cout << A.rows(1, 3) << endl;

  cout << "\nAccessing multiple columns" << endl;
  cout << A.cols(0, 2) << endl;
  cout << A.cols(1, 3) << endl;
}

void TEST(Matlab){
  arr x = randn(5);
  cout <<"\nrandn(5)" <<x <<endl;

  x = eye(5);
  cout <<"\neye(5)" <<x <<endl;
  for(uint i=0;i<x.d0;i++) CHECK(x(i,i)==1.,"is not eye");

  uintA p = randperm(5);
  cout <<"\nrandperm(5)" <<p <<endl;

  arr A = ARR(1,2,3,4);  A.reshape(2,2);
  arr B = repmat(A,2,3);
  cout <<"\nA=" <<A <<endl;
  cout <<"\nrepmat(A,2,3)" <<B <<endl;
}

void TEST(Exception){
  cout <<"\n*** exception handling\n";
  arr A;
  A.append(10);
  cout <<"accessing our of range..." <<endl;
  bool caught=false;
  try{
    cout <<A(2);
  }catch(const char *e){
    caught=true;
    cout <<"exception caught `" <<e <<"'" <<endl;
  }
  CHECK(caught,"exception not caught");
}

void TEST(MemoryBound){
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

void TEST(BinaryIO){
  cout <<"\n*** acsii and binary IO\n";
  arr a,b; a.resize(10000,100); rndUniform(a,0.,1.,false);

  ofstream fout("z.ascii"),bout("z.bin",ios::binary);
  ifstream fin("z.ascii") ,bin("z.bin",ios::binary);

  MT::timerStart();
  a.write(fout," ","\n","[]",true,false);
  cout <<"ascii write time: " <<MT::timerRead() <<"sec" <<endl;
  fout.close();

  MT::timerStart();
  b.read(fin);
  cout <<"ascii read time: " <<MT::timerRead() <<"sec" <<endl;
  fin.close();

  CHECK_ZERO(maxDiff(a,b), 1e-4, "ascii write-read error");

  MT::timerStart();
  a.write(bout,NULL,NULL,NULL,true,true);
  cout <<"binary write time: " <<MT::timerRead() <<"sec" <<endl;
  bout.close();

  MT::timerStart();
  b.read(bin);
  cout <<"binary read time: " <<MT::timerRead() <<"sec" <<endl;
  bin.close();

  CHECK(a==b,"binary IO failed!");
  cout <<"binary IO exactly restores double array and is much faster" <<endl;
}

void TEST(Expression){
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

void TEST(Permutation){
  cout <<"\n*** permutation\n";
  uintA p;
  rnd.seed(3);

  p.setStraightPerm(10);
  cout <<"\ninit:\n" <<p;
  CHECK(p(2)==2 && p(5)==5,"");

  p.permute(2,5);
  cout <<"\npermute(2,5):\n" <<p;
  CHECK(p(2)==5 && p(5)==2,"");

  p.setRandomPerm();
  cout <<"\nrandom:\n" <<p <<endl;
  for(uint i=0;i<p.N;i++) cout <<i <<":" <<p(i) <<"\n";
}

void TEST(Gnuplot){
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
  MT::wait(1.);
}

void TEST(Determinant){
  cout <<"\n*** determinant computation\n";
  arr a = ARR(1,1,2,1,1,0,0,-2,3);
  a.reshape(3,3);
  double d=determinant(a);
  double c00=cofactor(a,0,0);
  double c11=cofactor(a,1,1);
  double c22=cofactor(a,2,2);
  cout <<a <<"det=" <<d <<std::endl;
  cout <<"co00=" <<c00 <<std::endl;
  cout <<"co11=" <<c11 <<std::endl;
  cout <<"co22=" <<c22 <<std::endl;
  //  CHECK(fabs(d-c00*a(0,0))<1e-10,"");
  //  CHECK(fabs(d-c11*a(1,0))<1e-10,"");
}

void TEST(MM){
  cout <<"\n*** matrix multiplication speeds\n";
  uint M=10000,N=100,O=100;
  arr A(M,N),B(N,O),C,D;
  rndUniform(A,-1,1,false);
  rndUniform(B,-1,1,false);

  cout <<"speed test: " <<M <<'x' <<N <<'x' <<O <<" matrix multiplication..." <<std::endl;

  MT::useLapack=false; 
  MT::timerStart();
  innerProduct(D,A,B);
  double t_native=MT::timerRead();
  cout <<"native time = " <<t_native <<std::endl;

  if(!MT::lapackSupported){
    cout <<"LAPACK not installed - only native algorithms" <<std::endl;
    return;
  }

  MT::useLapack=true;
  MT::timerStart();
  blas_MM(C,A,B);
  double t_blas=MT::timerRead();
  cout <<"blas time = " <<MT::timerRead() <<std::endl;

  CHECK_ZERO(maxDiff(C,D), 1e-10, "blas MM is not equivalent to native matrix multiplication");
  CHECK(t_blas < t_native,"blas MM is slower than native");
}

void TEST(SVD){
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
  double t_native = MT::timerRead();
  cout <<"native SVD time = " <<t_native <<flush;
  D.setDiag(d);
  cout <<" error = " <<maxDiff(A, U*D*~V) <<" rank = " <<svdr <<"("<<r<<")"<<std::endl;
  CHECK_ZERO(maxDiff(A, U*D*~V), 1e-10, "native SVD failed");

  if(!MT::lapackSupported){
    cout <<"LAPACK not installed - only native algorithms" <<std::endl;
    return;
  }

  MT::useLapack=true;
  MT::timerStart();
  svdr=svd(U,d,V,A);
  double t_lapack = MT::timerRead();
  cout <<"lapack SVD time = " <<t_lapack <<flush;
  D.setDiag(d);
  cout <<" error = " <<maxDiff(A, U*D*~V) <<" rank = " <<svdr <<"("<<r<<")" <<std::endl;
  CHECK_ZERO(maxDiff(A, U*D*~V), 1e-10, "Lapack SVD failed");
}

void TEST(PCA) {
  // TODO: not really checking values automatically, just visualizing them
  // (and they are ok).

  cout <<"\n*** principal component analysis\n";
  arr x = { 1, -2, 1, -1, 1, 1, 1, 2 };
  x.reshape(4, 2);
  cout << "x = " << x << endl;

  arr xp, v, w;
  pca(xp, v, w, x, 1);

  cout << "xp = " << xp << endl;
  cout << "v = " << v << endl;
  cout << "w = " << w << endl;

  arr y = { 1, 1, -1, 1, -1, -1, 1, -1 };
  y.reshape(4, 2);
  cout << "y = " << y << endl;

  arr yp = y * w;
  cout << "yp = " << yp << endl;
}

void TEST(Inverse){
  cout <<"\n*** matrix inverse\n";
  uint m=500,n=500,svdr;
  arr A(m,n),invA,I;
  rndUniform(A,-1,1,false);
  I.setId(m);
  
  cout <<"speed test: " <<m <<'x' <<n <<" inversion..." <<std::endl;

  MT::useLapack=false;
  MT::timerStart();
  svdr=inverse_SVD(invA,A);
  double t_native = MT::timerRead();
  cout <<"native SVD inverse time = " <<t_native <<flush;
  cout <<" error = " <<maxDiff(A*invA,I) <<" rank = " <<svdr <<std::endl;
  CHECK_ZERO(maxDiff(A*invA,I), 1e-10, "native matrix inverse failed");

  /*MT::timerStart();
  MT::inverse_LU(invA,A);
  cout <<"native LU  inverse time = " <<MT::timerRead(); cout.flush();
  cout <<" error = " <<maxDiff(invA*A,I) <<std::endl;*/
  
  if(!MT::lapackSupported){
    cout <<"LAPACK not installed - only native algorithms" <<std::endl;
    return;
  }

  MT::useLapack=true;
  MT::timerStart();
  svdr=inverse_SVD(invA,A);
  double t_lapack = MT::timerRead();
  cout <<"lapack SVD inverse time = " <<t_lapack <<flush;
  cout <<" error = " <<maxDiff(A*invA, I) <<" rank = " <<svdr <<std::endl;
  CHECK_ZERO(maxDiff(A*invA, I), 1e-10, "lapack matrix inverse failed");

  /*MT::timerStart();
    MT::inverse_LU(invA,A);
    cout <<"lapack LU  inverse time = " <<MT::timerRead(); cout.flush();
    cout <<" error = " <<length(invA*A - I) <<std::endl;*/
  
  cout <<"\n*** symmetric matrix inverse\n";
  A.resize(m,m);
  rndUniform(A,-1,1,false);
  A=A*~A;
  I.setId(m);
  
  cout <<"speed test: " <<m <<'x' <<m <<" symmetric inversion..." <<std::endl;

  MT::timerStart();
  lapack_inverseSymPosDef(invA,A);
  double t_symPosDef = MT::timerRead();
  cout <<"lapack SymDefPos inverse time = " <<t_symPosDef <<flush;
  cout <<" error = " <<maxDiff(A*invA, I) <<std::endl;
  CHECK_ZERO(maxDiff(A*invA, I), 1e-6, "lapack SymDefPos inverse failed");

  CHECK(t_lapack < t_native, "lapack matrix inverse slower than native");
  CHECK(t_symPosDef < t_lapack, "symposdef matrix inverse slower than general");
}

void TEST(GaussElimintation) {
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

//------------------------------------------------------------------------------
//
// alternative operator notation -- TRASH THIS?
//

/*enum ArrayOpType { exProduct, inProduct, elemProduct };
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
      x = *left % *right;
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
*/

/*void TEST(NewOp){
  arr x(2,3),y(3,4);
  rndInt(x,0,5);
  rndInt(y,0,5);
  cout <<x <<y <<x PROD y <<x MUL x;
  //x = x MUL x;
  }*/


//------------------------------------------------------------------------------

void TEST(Tensor){
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
  rndInteger(A,0,1,false);
  tensorPermutation(B, A, TUP(0,1,2));
  cout <<A <<endl <<B <<endl;
}

//------------------------------------------------------------------------------

void write(RowShiftedPackedMatrix& PM){
  cout <<"RowShiftedPackedMatrix: real:" <<PM.Z.d0 <<'x' <<PM.real_d1 <<"  packed:" <<PM.Z.d0 <<'x' <<PM.Z.d1 <<endl;
  cout <<"\npacked numbers =" <<PM.Z
      <<"\nrowShifts=" <<PM.rowShift
     <<"\ncolPaches=" <<PM.colPatches
    <<"\nunpacked =" <<unpack(PM.Z) <<endl;
}

void TEST(RowShiftedPackedMatrix){
  cout <<"\n*** RowShiftedPackedMatrix\n";
  
  arr J;
  RowShiftedPackedMatrix *Jaux = auxRowShifted(J,10,4,12);
  rndInteger(J,0,9);
  for(uint i=0;i<J.d0;i++) Jaux->rowShift(i) = i/3;
  Jaux->computeColPatches(false);
  write(*Jaux);

  //constructor compressing an array
  arr K =  packRowShifted(unpack(J));
  write(*((RowShiftedPackedMatrix*)K.aux));
  
  cout <<"-----------------------" <<endl;

  //--randomized check
  for(uint k=0;k<100;k++){
    arr X(1+rnd(5),1+rnd(5));
    rndInteger(X,0,1);
    arr Y = packRowShifted(X);
//    write(castRowShiftedPackedMatrix(Y));
    arr x(X.d0);   rndInteger(x,0,9);
    arr x2(X.d1);  rndInteger(x2,0,9);
    cout <<"unpacking errors = " <<maxDiff(X,unpack(Y))
        <<' ' <<maxDiff(~X*X,unpack(comp_At_A(Y)))
       <<' ' <<maxDiff(X*~X,unpack(comp_A_At(Y)))
      <<' ' <<maxDiff(~X*x,comp_At_x(Y,x)) <<endl;
    CHECK_ZERO(maxDiff(X, unpack(Y)), 1e-10, "");
    CHECK_ZERO(maxDiff(~X*X, unpack(comp_At_A(Y))), 1e-10, "");
//    arr tmp =comp_A_At(Y);
//    //write(*((RowShiftedPackedMatrix*)tmp.aux));
//    cout <<X*~X <<endl <<unpack(comp_A_At(Y)) <<endl;
    CHECK_ZERO(maxDiff(X*~X, unpack(comp_A_At(Y))), 1e-10, "");
    CHECK_ZERO(maxDiff(~X*x, comp_At_x(Y,x)), 1e-10, "");
    CHECK_ZERO(maxDiff(X*x2, comp_A_x(Y,x2)), 1e-10, "");
  }
}

//------------------------------------------------------------------------------

int MAIN(int argc, char *argv[]){

  testBasics();
  testCheatSheet();
  testInitializationList();
  testSimpleIterators();
  testRowsAndColumsAccess();
  testStdVectorCompat();
  testMatlab();
  testException();
  testMemoryBound();
  testBinaryIO();
  testExpression();
  testPermutation();
  testGnuplot();
  testDeterminant();
  testRowShiftedPackedMatrix();
  testInverse();
  testMM();
  testSVD();
  testPCA();
  testTensor();
  testGaussElimintation();
  
  cout <<"\n ** total memory still allocated = " <<MT::globalMemoryTotal <<endl;
  CHECK_ZERO(MT::globalMemoryTotal, 0, "memory not released");
  
  return 0;
}

