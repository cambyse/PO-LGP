
void decomposeCameraProjectionMatrix(arr& K, arr& R, arr& t, const arr& P,bool verbose);
double projectionError(const arr& P, const arr& x, const arr& X);
void estimateCameraProjectionMatrix(arr& P, const arr& x, const arr& X);
void stereoTriangulation(arr& X, const arr& xL,const arr& xR, const arr& PL, const arr& PR);

#include "calibration.cpp"