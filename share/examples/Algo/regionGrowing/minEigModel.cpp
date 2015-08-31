#include <Gui/opengl.h>
#include "minEigModel.h"


void MinEigModel::setPoints(const uintA& points){
  pts = points;
  fringe = pts;
  included.resize(data.n());
  included.setZero();
  for(uint i:pts) included(i)=true;
  weights.resize(data.n());
  weights.setZero();
  setWeightsToOne();
}

void MinEigModel::setWeightsToOne(){
  setWeightsToZero();
  for(uint i:pts) weights(i)=1.;
  addStatistics(pts);
}

void MinEigModel::setWeightsToZero(){
  weights.setZero();
  stat_n=0;
  stat_x = zeros(data.d());
  stat_xx = zeros(data.d(), data.d());
}

void MinEigModel::addStatistics(const uintA& points, bool minus){
  for(uint i:points){
    double w=weights(i);
    if(!w) continue;
    if(!minus){
      stat_n += w;
      stat_x += w*data.X[i];
      stat_xx += w*(data.X[i]^data.X[i]);
    }else{
      stat_n -= w;
      stat_x -= w*data.X[i];
      stat_xx -= w*(data.X[i]^data.X[i]);
    }
  }
}

void MinEigModel::calc(bool update){
#if 0
  resetStatistics();
  addStatistics(pts);
#endif
  mean = stat_x/stat_n;
  eig.A = stat_xx/stat_n - (mean^mean);
  if(bias_xx.N) eig.A += bias_xx;
  if(!update){
    eig.computeExact();
  }else{
    eig.stepPowerMethod(3);
  }
}

void MinEigModel::expand(uint steps){
  for(uint s=0;s<steps;s++){
    data.expandFringe(fringe, pts, included);
    reweightWithError(fringe, 0.01);
  }
}

void MinEigModel::reweightWithError(uintA& pts, double margin){
  addStatistics(pts, true);
  if(margin<0.) margin = ::sqrt(eig.lambda_lo);
  for(uint j=pts.N;j--;){
    uint i=pts(j);
    double coeff = -.1 * MT::sqr(scalarProduct(data.X[i]-mean, eig.x_lo)/margin);
    weights(i) = ::exp(coeff);
    if(coeff<-5. || weights(i)<.1){
      pts.remove(j);
      included(j)=false;
      weights(i)=0.;
    }
  }
  addStatistics(pts, false);
}

void MinEigModel::computeConvexHull(){
  for(uint i:pts) if(weights(i)>.5) convexHull.V.append(data.X[i]);
  convexHull.V.reshape(convexHull.V.N/3,3);
  convexHull.makeConvexHull();
}

void MinEigModel::calcDensity(){
  computeConvexHull();
  density = stat_n * MT::sqr(mean(2)) / convexHull.getArea();
}

void MinEigModel::glDraw(){
  if(eig.x_lo.N!=3) return;
  if(!convexHull.V.N){
    ors::Quaternion rot;
    rot.setDiff(Vector_z, ors::Vector(eig.x_lo));
    glPushMatrix();
    glTranslatef(mean(0), mean(1), mean(2));
    glRotate(rot);
    glDrawBox(.5, .5, .01);
    glBegin(GL_LINES);
    glVertex3d(0.,0.,0.);
    glVertex3d(0., 0., .1);
    glEnd();
    glPopMatrix();
  }else{
    convexHull.glDraw();
  }
}

void MinEigModel::report(ostream& os, bool mini){
  if(mini){
    os <<pts.N <<' ' <<density <<' '<<mean <<"  " <<eig.lambda_lo <<' ' <<eig.x_lo <<"  " <<label <<endl;
  }else{
    os <<"model-report" <<endl;
    os <<"  #pts=" <<pts.N <<" #fringe=" <<fringe.N <<endl;
    os <<"  STATS: n=" <<stat_n <<" <x>=" <<stat_x/stat_n /*<<" <xx>=" <<stat_xx/stat_n*/ <<endl;
    os <<"  EIG: lambda_lo=" <<eig.lambda_lo <<" x_lo=" <<eig.x_lo <<endl;
    os <<"  QUALITY: density=" <<density <<" area=" <<convexHull.getArea() <<endl;
  }
}

void MinEigModel::colorPixelsWithWeights(arr& cols){
  for(uint i:pts){
    cols(data.idx2pixel(i), 0) = weights(i);
    cols(data.idx2pixel(i), 2) = data.weights(i);
  }
}
