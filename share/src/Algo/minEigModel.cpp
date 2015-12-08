#include <Gui/opengl.h>
#include <GL/gl.h>
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
  stat_n = 0;
  stat_x = zeros(data.d());
  stat_xx = zeros(data.d(), data.d());
}

void MinEigModel::addStatistics(const uintA& points, bool minus){
  arr wXi, wXiXi;
  for(uint i:points){
    double w=weights(i);
    if(!w) continue;
    const arr& Xi=data.X[i];
    wXi=Xi;  wXi*=w;
    outerProduct(wXiXi, wXi, Xi);
    if(!minus){
      stat_n += w;
      stat_x += wXi;
      stat_xx += wXiXi;
    }else{
      stat_n -= w;
      stat_x -= wXi;
      stat_xx -= wXiXi;
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
  arr Xi;
  for(uint j=pts.N;j--;){
    uint i=pts(j);
    Xi = data.X[i];
    Xi -= mean;
    double coeff = -.1 * mlr::sqr(scalarProduct(Xi, eig.x_lo)/margin);
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

double MinEigModel::coveredData(bool novelDataOnly){
  double coveredData=0.;
  if(!novelDataOnly)
    for(uint i:pts) coveredData += weights(i)*data.costs(i);
  else
    for(uint i:pts) coveredData += weights(i)*data.costs(i)*(1.-data.weights(i));
  return coveredData;
}

void MinEigModel::calcDensity(){
  computeConvexHull();
//  density = stat_n * mlr::sqr(mean(2)) / convexHull.getArea();
  density = coveredData() / convexHull.getArea();
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
    os <<pts.N <<' ' <<density <<' ' <<coveredData() <<' '  <<mean <<"  " <<eig.lambda_lo <<' ' <<eig.x_lo <<"  " <<label <<endl;
  }else{
    os <<"model-report" <<endl;
    os <<"  #pts=" <<pts.N <<" #fringe=" <<fringe.N <<endl;
    os <<"  STATS: n=" <<stat_n <<" <x>=" <<stat_x/stat_n /*<<" <xx>=" <<stat_xx/stat_n*/ <<endl;
    os <<"  EIG: lambda_lo=" <<eig.lambda_lo <<" x_lo=" <<eig.x_lo <<endl;
    os <<"  QUALITY: density=" <<density <<" coveredData=" <<coveredData() <<" area=" <<convexHull.getArea() <<endl;
  }
}

void MinEigModel::colorPixelsWithWeights(arr& cols){
  for(uint i:pts){
    cols(data.idx2pixel(i), 0) = weights(i);
    cols(data.idx2pixel(i), 2) = data.weights(i);
  }
}
