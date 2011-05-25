
void decomposeCameraProjectionMatrix(arr& K, arr& R, arr& t, const arr& P, bool verbose){
  //We decompose P to get K,R,t (calibration, rotation, translation)
  //get the square part
  arr KR(3,3),KRt(3,1);
  P.getMatrixBlock(KR ,0,0);
  P.getMatrixBlock(KRt,0,3);
  lapack_RQ(K,R,KR);
  t = -inverse(KR)*KRt;
  t.reshape(3);
  K /= K(2,2);   //the scaling of K is arbitrary, convention: $K_{3,3}=1$
  //R[2]() *= -1.; //OpenGL is flipping the z-axis...
  //ors::Quaternion r;  r.setMatrix(R.p);
  
  arr PP=~(K*R);  PP.append(-K*R*t);  PP=~PP;  PP/=PP.elem(0);  cout <<PP <<P <<endl;
  
  if(verbose){
    cout <<"\nProjection Matrix:"
         <<"\nP=" <<P
         <<"\nK=" <<K
         <<"\nR=" <<R
         //<<"\nr=" <<r
         <<"\nt=" <<t
         //<<"\nposition error = " <<norm(t-S.getCameraTranslation())
         <<endl;
  }
}

double projectionError(const arr& P, const arr& x, const arr& X){
  uint N=x.d0;
  arr x2 = X*~P;
  for(uint i=0;i<N;i++) x2[i]() /= x2(i,2);
  //cout <<x.sub(0,10,0,-1) <<x2.sub(0,10,0,-1) <<endl;
  return sqrt(sumOfSqr(x-x2)/N);
}

void estimateCameraProjectionMatrix(arr& P, const arr& x, const arr& X){
  //construct the matrix A:
  uint N=x.d0;
  arr A,zero(4);
  zero.setZero();
  for(uint i=0;i<N;i++){
    //COMPARE slide 24 lecture 8!
    A.append(zero);         A.append(-x(i,2)*X[i]); A.append(x(i,1)*X[i]);
    A.append(x(i,2)*X[i]);  A.append(zero);         A.append(-x(i,0)*X[i]);
  }
  A.reshape(2*N,12); //each point contributes two equations (two rows to A)

  //now we need to find p that minimizes |Ap| (subject to |p|=1 because the scaling doesn't matter)
  arr U,w,V;
  svd(U,w,V,~A*A); //Singular Value Decomposition of A^T A
  cout <<"Algebraic projection error = " <<w(11) <<endl;
  P = (~V)[11];
  P.reshape(3,4);
  P /= P(0,0);
  cout <<"image error=" <<projectionError(P,x,X) <<endl;
}

void stereoTriangulation(arr& X, const arr& xL,const arr& xR, const arr& PL, const arr& PR){
  arr B;
  B = skew(xL)*PL;
  B.append(skew(xR)*PR);
  B.reshape(6,4);
  //now we need to find p that minimizes |Ap| (subject to |p|=1 because the scaling doesn't matter)
  arr U,w,V;
  svd(U,w,V,~B*B); //Singular Value Decomposition of A^T A
  cout <<"Algebraic triangulation error = " <<w(3) <<endl;
  X = (~V)[3];
  X.reshape(4);
  X /= X(3);
}

