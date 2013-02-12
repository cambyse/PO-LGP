/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */


#ifdef MT_OPENCV
#undef COUNT
#include <opencv2/opencv.hpp>
#undef MIN
#undef MAX
#endif

#include "vision.h"
#include "vision_cuda.h"
#include "BinaryBP.h"
#ifdef MT_NILS
#  include <NP/transformations.h>
#endif

arr camera_calibration;

//student-t distribution with different degrees
float student1(float x){
  x=1.f+x;
  return 1.f/x;
}
float student3(float x){
  x=1.f+x/3.f;
  return 1.f/(x*x);
}
float student7(float x){
  x=1.f+x/7.f;
  x=x*x;
  return 1.f/(x*x);
}
float student(float x, float nu){
  return pow(1.f+x/nu, -(nu+1.f)/2.f);
}

#ifdef MT_OPENCV

CvMatDonor::CvMatDonor(){ i=0; mat=new CvMat[10]; }
CvMat* CvMatDonor::get(){ i++; if(i>=10) i=0; return &mat[i]; }

CvMat* arr2cvmat(CvMat *mat, const byteA& img){
  if(img.nd==3){
    cvInitMatHeader(mat, img.d0, img.d1, CV_8UC3, img.p);
  }else{
    cvInitMatHeader(mat, img.d0, img.d1, CV_8UC1, img.p);
  }
  return mat;
}

CvMat* arr2cvmat(CvMat *mat, const floatA& img){
  if(img.nd==1) cvInitMatHeader(mat, 1, 1, CV_32FC3, img.p);
  if(img.nd==2) cvInitMatHeader(mat, img.d0, img.d1, CV_32FC1, img.p);
  if(img.nd==3) cvInitMatHeader(mat, img.d0, img.d1, CV_32FC3, img.p);
  return mat;
}

CvMat* arr2cvmat(CvMat *mat, const doubleA& img){
  if(img.nd==3){
    NIY;
  }else{
    cvInitMatHeader(mat, img.d0, img.d1, CV_64FC1, img.p);
  }
  return mat;
}

char cvShow(const byteA& img, const char *window, bool wait){
  ENABLE_CVMAT
  CHECK(img.nd==2 || (img.nd==3 && img.d2==3), "img has improper dimensionalities");
  cvNamedWindow(window, CV_WINDOW_AUTOSIZE);
  if(img.nd==3){
    byteA imgBGR; resizeAs(imgBGR, img);
    cvCvtColor(CVMAT(img), CVMAT(imgBGR), CV_RGB2BGR);
    cvShowImage(window, CVMAT(imgBGR));
  }else{
    cvShowImage(window, CVMAT(img));
  }
  if(wait) return cvWaitKey();
  return cvWaitKey(2);
}

char cvShow(const floatA& img, const char *window, bool wait){
  ENABLE_CVMAT
  CHECK(img.nd==2 || (img.nd==3 && img.d2==3), "");
  cvNamedWindow(window, CV_WINDOW_AUTOSIZE);
  cvShowImage(window, CVMAT(img));
  if(wait) return cvWaitKey();
  return cvWaitKey(2);
}

char cvShowEvidence(const floatA& phi, const char *window){
  byteA tmp(phi.d0, phi.d1);
  for(uint i=0; i<tmp.N; i++) tmp.elem(i) = 255.*ratio_to_p(phi.elem(i));
  return cvShow(tmp, window);
}

//===========================================================================
//
// draw helper routinges
//

void cvDrawGraph(byteA& img, doubleA& V, uintA& E){
  ENABLE_CVMAT
  uint i;
  CvMat *mat = CVMAT(img);
  for(i=0; i<V.d0; i++){ cvCircle(mat, cvPoint(V(i, 0), V(i, 1)), 1, cvScalar(0, 0, 0)); }
  for(i=0; i<E.d0; i++){
    cvLine(mat,
           cvPoint(V(E(i, 0), 0), V(E(i, 0), 1)),
           cvPoint(V(E(i, 1), 0), V(E(i, 1), 1)),
           cvScalar(0, 0, 0));
  }
}

void cvDrawBox(byteA& img, const floatA& box){
  ENABLE_CVMAT
  cvRectangle(CVMAT(img), cvPoint(box(0), box(1)), cvPoint(box(2), box(3)), cvScalar(1, 0, 0));
}

void cvDrawPoints(byteA& img, const arr& points){
  ENABLE_CVMAT
  for(uint i=0; i<points.d0; i++){
    cvCircle(CVMAT(img), cvPoint(points(i, 0), points(i, 1)), 2, cvScalar(128., 0, 0));
  }
}
#else
#include "util.h"
CvMatDonor::CvMatDonor(){ NIY; }
CvMat* CvMatDonor::get(){ NIY; }
CvMat* arr2cvmat(CvMat *mat, const byteA& img){ NIY; return NULL; }
CvMat* arr2cvmat(CvMat *mat, const floatA& img){ NIY; return NULL; }
CvMat* arr2cvmat(CvMat *mat, const doubleA& img){ NIY; return NULL; }
char cvShow(const byteA& img, const char *window,bool wait){ NIY; return 0; }
char cvShow(const floatA& img, const char *window){ NIY; return 0; }
char cvShowEvidence(const floatA& phi, const char *window){ NIY; return 0; }
void cvDrawGraph(byteA& img, doubleA& V, uintA& E){ NIY; }
void cvDrawBox(byteA& img, const floatA& box){ NIY; }
void cvDrawPoints(byteA& img, const arr& points){ NIY; }
#endif

void boxConvolution(arr& out, const arr& in, uint width){
  uint K=in.N;
  uint w_half=width/2;
  if(width!=2*w_half+1) HALT("use odd box width please");
  double sum=0.;
  out.resize(K);
  out.setZero();
  double *inp=in.p, *outp=out.p;
  uint i=0;
  for(; i<w_half; i++){   sum+=inp[i];  }
  for(; i<width; i++){    sum+=inp[i];                        outp[i-w_half]=sum/(i+1);  }
  for(; i<K; i++){        sum+=inp[i];  sum-=inp[i-width];    outp[i-w_half]=sum/width;  }
  for(; i<K+w_half; i++){               sum-=inp[i-width];    outp[i-w_half]=sum/(K+w_half-i+1);  }
}

void gaussConvolution(arr& out, const arr& in, uint width, double eps){
  arr tmp=in;
  boxConvolution(out, tmp, width); memmove(tmp.p, out.p, out.N*out.sizeT);
  boxConvolution(out, tmp, width); memmove(tmp.p, out.p, out.N*out.sizeT);
  boxConvolution(out, tmp, width);
  //tmp=out; boxConvolution(out, width, tmp);
  normalizeDist(out);
  out += eps;
}

void mrf_BP(BP_data& msg, void (*conv)(arr&, const arr&), uint iter, byteA *max){
  CHECK(msg.phi.nd==3, "");
  uint Y=msg.phi.d0, X=msg.phi.d1, K=msg.phi.d2;
  msg.phi.reshape(Y*X, K);
  
  //-- fwd & bwd messages
  if(msg.u.N==Y*X*K){
    msg.u.reshape(Y*X, K);
    msg.d.reshape(Y*X, K);
    msg.l.reshape(Y*X, K);
    msg.r.reshape(Y*X, K);
  }else{
    msg.u.resize(Y*X, K);
    msg.d.resize(Y*X, K);
    msg.l.resize(Y*X, K);
    msg.r.resize(Y*X, K);
    msg.u=1.; msg.d=1.; msg.l=1.; msg.r=1.;
  }
  arr ud(Y*X, K), lr(Y*X, K);
  
  //#define MOD if((x&1)^(k&1))
#define MOD
  
  uint k, y, x, i, j;
  for(k=0; k<iter; k++){
    cout <<'.' <<std::flush;
    //recompute all messages
    for(y=0; y<Y; y++){
      for(x=0; x<X; x++) MOD { i=y*X+x;  ud[i]()=msg.phi[i]%msg.u[i]%msg.d[i]; }
        for(x=1; x<X; x++) MOD { i=y*X+x; j=i-1;  conv(msg.r[i](), msg.r[j]%ud[j]);  }
          for(x=X-1; x--;)  MOD { i=y*X+x; j=i+1;  conv(msg.l[i](), msg.l[j]%ud[j]);  }
          }
    for(x=0; x<X; x++){
      for(y=0; y<Y; y++) MOD { i=y*X+x;  lr[i]()=msg.phi[i]%msg.l[i]%msg.r[i]; }
        for(y=1; y<Y; y++) MOD { i=y*X+x; j=i-X;  conv(msg.d[i](), msg.d[j]%lr[j]);  }
          for(y=Y-1; y--;)  MOD { i=y*X+x; j=i+X;  conv(msg.u[i](), msg.u[j]%lr[j]);  }
          }
  }
  cout <<"done" <<endl;
  
  //-- posterior beliefs
  msg.b.resize(Y*X, K);
  if(max) max->resize(Y*X);
  for(i=0; i<X*Y; i++){
    msg.b[i]() = msg.phi[i]%msg.u[i]%msg.d[i]%msg.l[i]%msg.r[i];
    if(max)(*max)(i) = msg.b[i].maxIndex();
  }
  msg.phi.reshape(Y, X, K);
}



void getMaskedStats(floatA& mean, floatA& sdv, byteA& img, floatA& mask){
  mean.resize(img.d2); mean.setZero();
  sdv .resize(img.d2); sdv .setZero();
  uint I=img.d0, J=img.d1;
  img.reshape(I*J, img.d2);
  mask.reshape(I*J);
  float w=0.f, p;
  for(uint x=0; x<I*J; x++){
    p = mask(x); //ratio_to_p(mask(x));
    for(uint i=0; i<3; i++){
      mean(i) += p*float(img(x, i));
      sdv(i) += p*MT::sqr(float(img(x, i)));
    }
    w    += p;
  }
  mean /= w;
  sdv  /= w;
  sdv -= sqr(mean);
  sdv = sqrt(sdv);
  img.reshape(I, J, img.d1);
  mask.reshape(I, J);
}

void getCenter(arr &cen, floatA& b){
  uint x, y, Y=b.d0, X=b.d1;
  float px=0, py=0, p, P=0.;
  for(y=0; y<Y; y++) for(x=0; x<X; x++){
      p = 1.+tanh(b(y, x));
      px += p*x;
      py += p*y;
      P += p;
    }
  px/=P; py/=P;
  cen.resize(2);
  cen(0)=px; cen(1)=py;
}


#undef MIN
#undef MAX

void byte2float(floatA& f, const byteA& b){
  resizeAs(f, b);
  for(uint i=0; i<b.N; i++) f.elem(i) = ((float)b.elem(i))/255.f;
}

void rgb2hsv(floatA& hsv, floatA& rgb){
  uint W=0, H=0, N=0;
  if(rgb.nd==3){ H=rgb.d0; W=rgb.d1; N=W*H; rgb.reshape(N, 3); } else N=rgb.d0;
  CHECK(rgb.d1==3, "");
  uint i;
  float r, g, b, h, s, v, min;
  hsv.resizeAs(rgb);
  for(i=0; i<N; i++){
    r=rgb(i, 0);  g=rgb(i, 1);  b=rgb(i, 2);
    v  =MT::MAX(MT::MAX(r, g), b);
    min=MT::MIN(MT::MIN(r, g), b);
    CHECK(v<=1.f && min>=0.f, "require [0, 1] floats");
    if(v>0) s=(v-min)/v; else s=0.f;
    if(v==min)    h = 0.;
    else if(v==r) h = 1.f/6.f*(0.f+(g-b)/(v-min));
    else if(v==g) h = 1.f/6.f*(2.f+(b-r)/(v-min));
    else if(v==b) h = 1.f/6.f*(4.f+(r-g)/(v-min));
    else HALT("");
    if(h<0) h+=1.;
    hsv(i, 0)=h;  hsv(i, 1)=s;  hsv(i, 2)=v;
  }
  if(H){
    rgb.reshape(H, W, 3);
    hsv.reshape(H, W, 3);
  }
}

void hsv2rgb(byteA& rgb, const intA& hsv){
  float h=hsv(0)/60.f, s=hsv(1)/255.f, v=hsv(2)/255.f;
  h=(float)fmod(h, 6.f);
  float r, g, b;
  r=g=b=0.;
  if(h<=1.)        { r=v; g=v*h; }
  if(h>1. && h<=2.){ g=v; r=v*(2.f-h); }
  if(h>2. && h<=3.){ g=v; b=v*(h-2.f); }
  if(h>3. && h<=4.){ b=v; g=v*(4.f-h); }
  if(h>4. && h<=5.){ b=v; r=v*(h-4.f); }
  if(h>5. && h<=6.){ r=v; b=v*(6.f-h); }
  rgb(0)=255*(s*r+(1.f-s)*v);
  rgb(1)=255*(s*g+(1.f-s)*v);
  rgb(2)=255*(s*b+(1.f-s)*v);
}

void gnuplotHistogram(floatA &data, float min, float max, uint bins){
  uint W=0, H=0, N=0, K, i, k;
  if(data.nd==3){ H=data.d0; W=data.d1; data.reshape(W*H, data.d2); }
  if(data.nd==1){ data.reshape(data.N, 1); }
  N=data.d0;
  K=data.d1;
  if(min==max){  min=data.min();  max=data.max();  }
  arr hist(K, bins);
  hist.setZero();
  for(i=0; i<N; i++) for(k=0; k<K; k++){
      uint b=((data(i, k)-min)/(max-min))*(bins-1);
      if(b<bins) hist(k, b)+=1.;
    }
  hist/=(double)N;
  ofstream z("z.hist");
  for(i=0; i<bins; i++){ z <<min+(max-min)*i/(bins-1) <<' '; for(k=0; k<K; k++) z <<hist(k, i) <<' '; z <<endl; }
  z.close();
  MT::String cmd("plot 'z.hist' us 1:2");
  for(k=1; k<K; k++) cmd <<", 'z.hist' us 1:" <<k+2;
  cout <<"plotting " <<cmd <<endl;
  gnuplot(cmd);
  if(H){
    data.reshape(H, W, 3);
  }
}

float hsv_diff(const floatA& a, const floatA& b, const floatA& tol){
  float h = sin((a(0)-b(0))*MT_PI)/MT_PI/tol(0);
  float s = (a(1)-b(1))/tol(1);
  float v = (a(2)-b(2))/tol(2);
  return h*h+s*s+v*v;
}

void getHsvEvidences(floatA &phi, floatA &hsv, const floatA& hsvTarget, const floatA& hsvTol){
  uint i, X=0, Y=0, N=0;
  if(hsv.nd==3){ Y=hsv.d0; X=hsv.d1; hsv.reshape(Y*X, 3); }
  N=hsv.d0;
  hsv.reshape(N, 3);
  phi.resize(N);
  for(i=0; i<N; i++){ //for each pixel
    //double p=exp(-.5*hsv_diff(hsv[i], hsvTarget, hsvTol));
    //phi(i)=.5*log(p/(1.-p));
    double d = hsv_diff(hsv[i], hsvTarget, hsvTol);
    phi(i) = -log(d);
  }
  if(Y){
    phi.reshape(Y, X);
    hsv.reshape(Y, X, 3);
  }
}

#ifdef MT_OPENCV
void getDiffProb(floatA& diff, const byteA& img0, const byteA& img1, float pixSdv, uint range){
  diff.resize(img0.d0, img0.d1);
  diff.setZero();
  float d, *p=diff.p;
  byte *p0=img0.p, *p1=img1.p;
  for(uint i=0; i<diff.N; i++){
    d = float(*p0)-float(*p1); *p += d*d;    p0++; p1++;
    d = float(*p0)-float(*p1); *p += d*d;    p0++; p1++;
    d = float(*p0)-float(*p1); *p += d*d;    p0++; p1++;
    *p /= 3*(pixSdv*pixSdv);
    p++;
  }
  floatA smoothed(diff);
  ENABLE_CVMAT
  cvSmooth(CVMAT(diff), CVMAT(smoothed), CV_BLUR, range, range);
  cvSmooth(CVMAT(diff), CVMAT(smoothed), CV_BLUR, range, range);
  for(uint i=0; i<smoothed.N; i++) smoothed.p[i] = student3(smoothed.p[i]);
  diff = smoothed;
}
#endif

void getHsvCenter(arr& cen, byteA &img, uint iter, const floatA& hsvTarget, const floatA& hsvTol){
  floatA rgb, hsv;
  BinaryBPGrid bp;
  
  byte2float(rgb, img);
  rgb2hsv(hsv, rgb);
  getHsvEvidences(bp.phi, hsv, hsvTarget, hsvTol); //ARRAY<float>(.0, 1., 1.), ARRAY<float>(.2, .5, .5));
  
  byteA tmp;
  resizeAs(tmp, bp.phi);
  for(uint i=0; i<tmp.N; i++) tmp.elem(i) = 128.*(1.+tanh(bp.phi.elem(i)));
  cvShow(tmp, "evid");
  
  bp.tanh_J = .5;
  bp.b=bp.phi;
  for(uint k=0; k<iter; k++) bp.step();
  
  getCenter(cen, bp.b);
  
  //byteA tmp;
  resizeAs(tmp, bp.b);
  for(uint i=0; i<tmp.N; i++) tmp.elem(i) = 128.*(1.+tanh(bp.b.elem(i)));
  cvShow(tmp, "post");
}

void imagePointPair2WorldPoint(arr& worldPoint, const arr& left, const arr& right){
  arr image_points(right);
  image_points.reshape(1, 2);
  arr disparity(1);
  disparity(0) = left(0)-right(0);
#ifdef MT_NILS
  vision::image2world(worldPoint, camera_calibration, image_points, disparity);
#else
  NIY;
#endif
  worldPoint.reshape(3);
  worldPoint(0) *= -1.;
  worldPoint    *= .01;
  worldPoint(0) += .06;
  cout <<"localized Point = " <<worldPoint <<endl;
}

/*
  void localizeHsv(arr& worldPoint, byteA& left, byteA& right, const floatA& hsvTarget, const floatA& hsvTol, int smaller){
  if(smaller!=1){
  cvResize(left , smaller);
  cvResize(right, smaller);
  }

  arr cenL, cenR;
  getHsvCenter(cenL, left , 2, hsvTarget, hsvTol);
  getHsvCenter(cenR, right, 2, hsvTarget, hsvTol);

  cvCircle(CVMAT(left) , cvPoint(cenL(0), cenL(1)), 5, cvScalar(0, 255, 0));
  cvCircle(CVMAT(right), cvPoint(cenR(0), cenR(1)), 5, cvScalar(0, 255, 0));
  cvShow(left, "left");
  cvShow(right, "right");

  //cout <<"disparity = " <<cenL(0)-cenR(0) <<endl;

  if(smaller!=1){
  cenL*=(double)smaller;
  cenR*=(double)smaller;
  }
  imagePointPair2WorldPoint(worldPoint, cenL, cenR);
  }
*/




//===========================================================================
//
// parameters, initialization, precomputations
//

#if 0 //STUFF FROM THE FLOW ESTIMATION PAPER...

//constants
#define V 13
#define W 5
#define R 25
uint I, J, X, S; //height, width, V-range, I-comparison-range, I-range, segments

bool invertV=false;

//range definitions
int Vdj[V] = {  0, -1,  0,  1, -2, -1, 0, 1, 2, -1, 0, 1, 0 };
int Vdi[V] = { -2, -1, -1, -1,  0,  0, 0, 0, 0,  1, 1, 1, 2 };

int Rdj[R] = { -2, -1,  0,  1,  2,  -2, -1,  0,  1,  2,  -2, -1, 0, 1, 2,  -2, -1, 0, 1, 2,  -2, -1, 0, 1, 2 };
int Rdi[R] = { -2, -2, -2, -2, -2,  -1, -1, -1, -1, -1,   0,  0, 0, 0, 0,   1,  1, 1, 1, 1,   2,  2, 2, 2, 2 };

//parameters
double rho_I, rho_V, rho_S;
double sig_I, sig_V;
double nu_I, nu_V, nu_S;
arr A_q, t_q, sig_q;

//conversions between linear and pixel coordinates
void getIJ(uint& i, uint& j, uint x){ i=x/J; j=x-J*i; }
//void getRdidj(int& di, int& dj, uint r){ di=Rdi[r]; dj=Rdj[r]; }

//mean of segments' flow patterns
void q_mean(float& qx, float& qy, uint x, uint s){
  arr xvec(2);
  uint i, j;
  getIJ(i, j, x);
  float px, py;
  px = 4.*((double)j-J/2)/(I+J);
  py = 4.*((double)i-I/2)/(I+J);
  qx = A_q(s, 0, 0)*px + A_q(s, 0, 1)*py + t_q(s, 0);
  qy = A_q(s, 1, 0)*px + A_q(s, 1, 1)*py + t_q(s, 1);
  if(invertV){
    qx *= -1.f;
    qy *= -1.f;
  }
}


static floatA I_student;
static floatA V_student, V_range;
static floatA S_noise;
static floatA w_ell, w_V;
static intA Qdx;
static floatA Qprob;
static int Rdx[R], Vdx[V];

void compute_basics(){
  uint r, v, w, s, t, x;
  int c, d;
  int dx, dy;
  double var;
  float qx, qy, qvx, qvy;
  
  for(r=0; r<R; r++) Rdx[r] = Rdj[r] + J*Rdi[r];
  for(v=0; v<V; v++) Vdx[v] = Vdj[v] + J*Vdi[v];
  
  //I-patch metric
  rho_I = 2.5;
  sig_I = 1.;
  I_student.resize(R, 256, 256);
  for(r=0; r<R; r++){
    dx=Rdj[r]; dy=Rdi[r];
    var = sig_I*sig_I / ::exp(-.5 * (dx*dx+dy*dy)/(rho_I*rho_I));
    for(c=0; c<256; c++) for(d=0; d<256; d++)
        //I_student(r, c, d) = ::exp(-.5 * (c-d)*(c-d)/var);
        I_student(r, c, d) = student3((c-d)*(c-d)/var);
  }
  //cout <<"I_student=\n" <<I_student <<endl;
  
  //V-noise
  sig_V = .1;
  var = sig_V*sig_V;
  V_student.resize(V, V);
  for(v=0; v<V; v++) for(w=0; w<V; w++){
      dx = Vdj[v] - Vdj[w];
      dy = Vdi[v] - Vdi[w];
      d = dx*dx + dy*dy;
      V_student(v, w) = student3(d/var);
    }
  tensorCondNormalize(V_student, 1);
  //cout <<"V_student=\n" <<V_student <<endl;
  
  //V-range
  rho_V = 2.5;
  V_range.resize(R);
  for(r=0; r<R; r++){
    dx = Rdj[r];
    dy = Rdi[r];
    V_range(r) = ::exp(-.5 * (dx*dx+dy*dy)/(rho_V*rho_V));
  }
  cout <<"V_range=\n" <<V_range <<endl;
  
  //range filters
  w_ell.resize(W);
  for(d=-W/2; d<=W/2; d++)
    w_ell(d+W/2) = ::exp(-.5 * (d*d)/(rho_I*rho_I));
  w_ell /= sum(w_ell);
  cout <<"w_ell=\n" <<w_ell <<endl; // <<w_ell * ~w_ell <<endl;
  
  w_V.resize(W);
  for(d=-W/2; d<=W/2; d++)
    w_V(d+W/2) = ::exp(-.5 * (d*d)/(rho_V*rho_V));
  w_V /= sum(w_V);
  cout <<"w_V=\n" <<w_V <<endl; // <<w_V * ~w_V <<endl;
  
  //S-noise
  nu_S = .1;
  S_noise.resize(S, S);
  for(s=0; s<S; s++) for(t=0; t<S; t++){
      S_noise(s, t) = s==t?1.-nu_S*(S-1):nu_S;
    }
  checkNormalization(S_noise, 1e-6);
  cout <<"S_noise=\n" <<S_noise <<endl;
  
  //initialize A_q aned t_q
  A_q.resize(S, 2, 2);
  A_q.setZero();
  t_q.resize(S, 2);
  t_q.setZero();
  for(s=0; s<S; s++) t_q(s, 0) = 4.*s/(S-1) - 2.;
  cout <<"A_q=\n" <<A_q <<"\nt_q=\n" <<t_q <<endl;
  
  //discretized mean segmentation displacement
  Qdx.resize(X, S);
  for(x=0; x<X; x++) for(s=0; s<S; s++){
      q_mean(qx, qy, x, s);
      dx = (int)floor(qx+.5);
      dy = (int)floor(qy+.5);
      Qdx(x, s) = dx + J*dy;
    }
    
  //q-prior
  sig_q.resize(S);
  sig_q = 1.;
  Qprob.resize(X, S, V);
  for(x=0; x<X; x++) for(s=0; s<S; s++){
      q_mean(qx, qy, x, s);
      for(v=0; v<V; v++){
        qvx = qx - Vdj[v];
        qvy = qy - Vdi[v];
        Qprob(x, s, v) = ::exp(-.5 * (qvx*qvx+qvy*qvy)/(sig_q(s)*sig_q(s)));
      }
    }
}


//===========================================================================
//
// convolution
//

void symmetricConvolution(floatA& output, const floatA& input, const floatA& wi, const floatA& wj){
  CHECK(input.nd==2, "");
  int I=input.d0, J=input.d1;
  int DI=(wi.N-1)/2, DJ=(wj.N-1)/2;
  int x, di, dj, X=I*J;
  //first convolute horizontally
  floatA mid(input.N);
  mid.setZero();
  output.resizeAs(input);
  output.setZero();
#if 0 //readable version
  for(x=0; x<X; x++){
    for(dj=-DJ; dj<=DJ; dj++) mid((X+x+dj)%X) += wj(DJ+dj) * input.elem(x);
  }
  //now vertically
  for(x=0; x<X; x++){
    for(di=-DI; di<=DI; di++) output.elem((X+x+di*J)%X) += wi(DI+di) * mid(x);
  }
#else //optimized version
  float input_x, *midp, *wjp, *wjstop;
  for(x=0; x<X; x++){
    input_x = input.p[x];
    midp=mid.p+x-DJ;
    wjp=wj.p;
    wjstop=wjp+2*DJ+1;
    if(x>DJ && x<X-DJ){ //inner image: no mod necessary
      for(; wjp!=wjstop; midp++, wjp++)(*midp) += (*wjp) * input_x;
    }else{
      for(dj=-DJ; wjp!=wjstop; dj++, wjp++) mid.p[(X+x+dj)%X] += (*wjp) * input_x;
    }
  }
  //now vertically
  float mid_x, *outp, *wip, *wistop;
  for(x=0; x<X; x++){
    mid_x = mid.p[x];
    outp=output.p+x-DI*J;
    wip=wi.p;
    wistop=wip+2*DI+1;
    if(x>DI*J && x<X-DI*J){ //inner image: no mod necessary
      for(; wip!=wistop; wip++, outp+=J)(*outp) += (*wip) * mid_x;
    }else{
      for(di=-DI; wip!=wistop; di++, wip++) output.p[(X+x+di*J)%X] += (*wip) * mid_x;
    }
  }
#endif
}



//===========================================================================
//
// messages
//

void evidence_diff(floatA& diff, const byteA& img0, const byteA& img1, float pixSdv){
  uint X=img0.d0*img0.d1;
  diff.resize(img0.d0, img0.d1);
  float d;
  for(uint x=0; x<X; x++){
    d = ((float)img1.p[3*x  ] - (float)img0.p[3*x  ]);    diff.p[x]  = d*d;
    d = ((float)img1.p[3*x+1] - (float)img0.p[3*x+1]);    diff.p[x] += d*d;
    d = ((float)img1.p[3*x+2] - (float)img0.p[3*x+2]);    diff.p[x] += d*d;
    diff.p[x] /= 3*(pixSdv*pixSdv);
  }
  floatA diff2=diff;
  symmetricConvolution(diff, diff2, w_ell, w_ell);
  for(uint x=0; x<X; x++) diff.p[x] = p_to_ratio(1.-student3(diff.p[x]));
}

void compute_mu_IV(floatA& mu_IV, const byteA& img0, const byteA& img1, float pixSdv){
  uint X=img0.N;
  mu_IV.resize(V, X);
  float d;
  floatA diff;
  diff.resize(img0.d0, img0.d1);
  uint v;
  byteA It_trans;
  for(v=0; v<V; v++){
#if 0
    for(uint x=0; x<X; x++){
      d = ((float)img1.p[x] - (float)img0.p[(X+x-Vdx[v])%X])/pixSdv;
      diff.p[x] = d*d;
    }
#else //faster
    It_trans = img0;
    if(!invertV) It_trans.shiftPerm(Vdx[v]); else It_trans.shiftPerm(-Vdx[v]);
    float *diffp=diff.p, *diffstop=diffp+diff.N;
    byte *Itp=It_trans.p, *Ittp=img1.p;
    for(; diffp!=diffstop; diffp++, Itp++, Ittp++){ d=(float(*Ittp)-float(*Itp))/pixSdv; (*diffp) = d*d; }
#endif
    symmetricConvolution(mu_IV[v](), diff, w_ell, w_ell);
    //mu_IV[v]()=diff;
  }
  float *mu_IVp=mu_IV.p, *mu_IVstop=mu_IVp+mu_IV.N;
  //for(;mu_IVp!=mu_IVstop;mu_IVp++){ (*mu_IVp) = ::exp(-.5*(*mu_IVp)); }       //gauss
  for(; mu_IVp!=mu_IVstop; mu_IVp++){(*mu_IVp) = student3(*mu_IVp); }  //gauss
  mu_IV = ~mu_IV;
}

/*void compute_mu_VV(floatA& mu_VV, const floatA& alphaV){
  uint v;

  floatA alphaV_T, noised_alphaV;
  transpose(alphaV_T, alphaV);
  innerProduct(noised_alphaV, V_student, alphaV_T);

  floatA translated_alpha;
  mu_VV.resize(V, X);
  for(v=0;v<V;v++){
  translated_alpha=noised_alphaV[v];
  if(!invertV) translated_alpha.shiftPerm(Vdx[v]); else translated_alpha.shiftPerm(-Vdx[v]);
  symmetricConvolution(mu_VV[v](), translated_alpha, w_V, w_V);
  }
  mu_VV=~mu_VV;
  }*/

void compute_meanV(arr& mean, const floatA& alphaV){
  uint x, v;
  double Z, w;
  mean.resize(X, 2);
  mean.setZero();
  for(x=0; x<X; x++){
    Z=0.;
    for(v=0; v<V; v++){
      w=alphaV(x, v);
      Z += w;
      mean(x, 0) += w * Vdj[v];
      mean(x, 1) += w * Vdi[v];
    }
    mean[x]() /= Z;
  }
}

void compute_hueMap(byteA& hue, const floatA& alphaV){
  arr mean;
  compute_meanV(mean, alphaV);
  //cout <<mean.sub(0, 500, 0, -1) <<endl;
  //MT::wait();
  hue.resize(X, 3);
  MT::Color col;
  uint x, h;
  float s;
  for(x=0; x<X; x++){
    h=(uint)((MT::phi(mean(x, 0), mean(x, 1))+MT_PI)*180./MT_PI);
    s = norm(mean[x]) * 100; if(s>255) s=255;
    hsv2rgb(hue[x](), ARRAY<int>(h, (byte)s, 255));
  }
  hue.reshape(I, J, 3);
}

//#undef R
//#undef W
//#undef V

#endif




#ifdef MT_OPENCV
void smooth(floatA& theta, uint size){
  ENABLE_CVMAT
  floatA tmp=theta;
  cvSmooth(CVMAT(tmp), CVMAT(theta), CV_BLUR, size, size);
}

void findMaxRegionInEvidence(uintA& box, floatA *center, floatA *axis,
                             const floatA& theta, double threshold){
  ENABLE_CVMAT
  uint W=theta.d1;
  uint i=theta.maxIndex(); float max1 = theta.elem(i); if(max1 < 0.5) return;
  CvConnectedComp component;
  byteA mask(theta.d0+2, theta.d1+2); mask.setZero();
  cvFloodFill(CVMAT(theta), cvPoint(i%W, i/W), cvScalar(1),
              cvScalar(threshold), cvScalar(threshold),
              &component, CV_FLOODFILL_FIXED_RANGE|CV_FLOODFILL_MASK_ONLY, CVMAT(mask));
              
  box=ARRAY<uint>(component.rect.x, component.rect.y,
                  component.rect.x+component.rect.width, component.rect.y+component.rect.height);
                  
  //if(box(2)-box(0)>theta.d0*0.95 || box(3)-box(1)>theta.d0*0.95) return;
  
  float Z=0, M20=0, M02=0, M11=0, z;
  
  if(!center) return;
  
  center->resize(2);
  center->setZero();
  
  for(uint x=box(0); x<=box(2) && x<theta.d1; x++)
    for(uint y=box(1); y<=box(3) && y<theta.d0; y++)
      if(mask(y+1, x+1)){
        z=theta(y, x);
        (*center)(0) += z*x;
        (*center)(1) += z*y;
        Z += z;
        if(axis){
          M20 += z*x*x;
          M02 += z*y*y;
          M11 += z*x*y;
        }
      }
  (*center)/=Z;
  if(!axis) return;
  axis->resize(4);
  axis->setZero();
  M20/=Z;
  M02/=Z;
  M11/=Z;
  floatA C(2, 2);//image covariance
  C(0, 0) = M20 - (*center)(0)*(*center)(0);
  C(1, 1) = M02 - (*center)(1)*(*center)(1);
  C(0, 1) = M11 - (*center)(1)*(*center)(0);
  C(1, 0) = C(0, 1);
  float T = C(0, 0) + C(1, 1);
  float D = C(0, 0)*C(1, 1) - C(0, 1)*C(1, 0);
  float l1 = T/2 + sqrt(T*T/4-D);
  floatA ax(2);
  ax(0) = l1 - C(1, 1); ax(1) = C(1, 0);
  ax/=norm(ax);
  ax *= (float)sqrt(l1);
  (*axis)(0) = ax(0)+(*center)(0);
  (*axis)(1) = ax(1)+(*center)(1);
  (*axis)(2) = -1*ax(0)+(*center)(0);
  (*axis)(3) = -1*ax(1)+(*center)(1);
}
#endif

byteA evi2rgb(const floatA& theta){
  byteA tmp;
  tmp.resize(theta.d0*theta.d1, 3);
  for(uint i=0; i<theta.N; i++) tmp(i, 0)=tmp(i, 1)=tmp(i, 2)=255.f*theta.elem(i);
  tmp.reshape(theta.d0, theta.d1, 3);
  return tmp;
}

floatA rgb2evi(const byteA& theta){
  floatA tmp;
  tmp.resize(theta.d0*theta.d1);
  for(uint i=0; i<tmp.N; i++) tmp.elem(i) = float(theta.elem(3*i)+theta.elem(3*i+1)+theta.elem(3*i+2))/(3.f*255.f);
  tmp.reshape(theta.d0, theta.d1);
  return tmp;
}


