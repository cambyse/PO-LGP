#include <Ors/roboticsCourse.h>

void inferCameraProjectionMatrix(){
  uint N = 100;
  double pixelNoise = .0; //5;
  VisionSimulator S;
  arr  X;
  S.getRandomWorldPoints(X,N);

  for(uint k=0;k<100;k++){
    cout <<"\n\n*** iteration " <<k <<endl;
    //the real world projects these points onto the camera (here: simulated)
    arr  x;
    S.projectWorldPointsToImagePoints(x,X,pixelNoise);
    // the i-th world point is X[i] (4D because in homogeneous coordinates)
    // the i-th image point is x[i] (3D because in homogeneous coordinates)
  

    //** now you have to infer the project matrix P  (see slide 24 lecture 8)
    //construct the matrix A:
    arr A,Arow(3,4);
    for(uint i=0;i<N;i++){
      //COMPARE slide 24 lecture 8!
      Arow[0]=0.;           Arow[1]=-x(i,2)*X[i];  Arow[2]= x(i,1)*X[i];
      A.append(Arow);
      Arow[0]=x(i,2)*X[i];  Arow[1]=0.;            Arow[2]=-x(i,0)*X[i];
      A.append(Arow);
    }
    A.reshape(2*N,12); //each point contributes two equations (two rows to A)

    //now we need to find p that minimizes |Ap| (subject to |p|=1 because the scaling doesn't matter)
    arr P,U,w,V;
    svd(U,w,V,~A*A); //Singular Value Decomposition of A^T A
    P = (~V)[11];
    P.reshape(3,4);
    P /= P(0,0);

    //** What follows is code to extract human-readable information from P (see slide 32!)
    //We decompose P to get K,R,t (calibration, rotation, translation)
    //get the square part
    arr KR(3,3),K,R,KRt(3,1);
    P.getMatrixBlock(KR ,0,0);
    P.getMatrixBlock(KRt,0,3);
    arr t = -inverse(KR)*KRt;  t.reshape(3);
    lapack_RQ(K,R,KR);
    if(determinant(R)<1.){ //account it to the calibration matrix, that z-axis is flipped! (OpenGL: glFrustrum flips z-axis)
      R[2]()*=-1.; //flip z-axis
      for(uint i=0;i<3;i++) K(i,2) *=-1.; //flip z-axis
      K /= -K(2,2); //the scaling of K is arbitrary, convention: $K_{3,3}=-1$
    }else{
      K /= K(2,2); //the scaling of K is arbitrary, convention: $K_{3,3}=1$
    }
    R = ~R; //in my convention this R is the inverse rotation!
    ors::Quaternion r;  r.setMatrix(R.p);

    cout <<"\nProjection Matrix computed from noise points:"
	 <<"\nP=" <<P
	 <<"\nK=" <<K
	 <<"\nR=" <<R
	 <<"\nr=" <<r
	 <<"\nt=" <<t
	 <<"\nposition error = " <<length(t-S.getCameraTranslation()) <<endl;
    S.watch();
  }
  
}

int main(int argc, char **argv){
  inferCameraProjectionMatrix();
}
