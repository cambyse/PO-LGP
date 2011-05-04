#ifndef MT_calibration_h
#define MT_calibration_h

#include "array.h"

void decomposeCameraProjectionMatrix(arr& K, arr& R, arr& t, const arr& P,bool verbose);
double projectionError(const arr& P, const arr& x, const arr& X);
void estimateCameraProjectionMatrix(arr& P, const arr& x, const arr& X);
void stereoTriangulation(arr& X, const arr& xL,const arr& xR, const arr& PL, const arr& PR);


void stereoTriangulation_nonhom(arr& X_3d, const arr& x_4d, const arr&PL, const arr& PR);
void stereoTriangulation_nonhom(arr& X, const arr& x);

#ifdef MT_IMPLEMENTATION
#  include "calibration.cpp"
#endif

#endif