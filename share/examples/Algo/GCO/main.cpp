#include <extern/GCO/GCoptimization.h>
#include <Core/array.h>
#include <Perception/kinect2pointCloud.h>
#include <Geo/geo.h>
#include <GL/gl.h>

const double scale=10000.;

static const double colorsTab[6][3] = {
  {0.2, 0.2, 1.0}, // blue
  {1.0, 0.8, 0.0}, // gold
  {1.0, 0.0, 0.0}, // red
  {0.7, 0.7, 0.7}, // gray
  {0.0, 1.0, 1.0}, // white
  {0.2, 1.0, 0.2}
};

struct MinEigModel{
  //statistics
  double n;
  arr X, mu;
  //minimal eigenvector
  arr beta;
  double beta_len;

  void resetStatistics(uint dim){
    double reg=1e-6;
    n  = reg;
    mu = zeros(dim);
    X  = reg * eye(dim);
  }

  void addStatistics(const arr& phi){
    n +=1.;
    mu+=phi;
    X +=phi*~phi;
  }

  void comBeta(){
    arr Sig, Beta;
    lapack_EigenDecomp(X - mu*~(mu/n), Sig, Beta);
    beta_len = Sig(0);
    if(beta_len>1e-10) beta = Beta[0];
    else{ beta = zeros(mu.N); beta.last()=1.; }
          cout <<"compEig: " <<Sig <<':' <<Beta <<endl;
    if(beta_len>1e-2) CHECK_ZERO(beta_len - length((X - mu*~(mu/n))*beta), 1e-6, "");
  }

  double cost(const arr& phi){
    return mlr::sqr(scalarProduct(beta, phi-mu/n));
  }

  double f(const arr& phi){
    if(!beta.N) return 0.;
//    double y_pred = -(~phi.sub(0,-2)*s->beta.sub(0,-2)).scalar()/s->beta.last();
    arr mean=mu/n;
    double y_pred = mean.last();
    if(mu.N>1 && beta_len>1e-10 && beta.last()>1e-10) y_pred -= (~(phi.sub(0,-2)-mean.sub(0,-2))*beta.sub(0,-2))/beta.last();
    return y_pred;
  }
};

struct ModelDrawer:GLDrawer{
  mlr::Array<MinEigModel>& M;
  ModelDrawer(mlr::Array<MinEigModel>& M):M(M){}
  void glDraw(OpenGL &){
    uint c=0;
    for(MinEigModel &m:M) if(m.beta.N==3){
      ors::Quaternion rot;
      rot.setDiff(Vector_z, ors::Vector(m.beta));
      arr mean=m.mu/m.n;
      glColor(c++);
      glPushMatrix();
      glTranslatef(mean(0), mean(1), mean(2));
      glRotate(rot);
      glDrawBox(.5, .5, .01);
      glBegin(GL_LINES);
      glVertex3d(0.,0.,0.);
      glVertex3d(0., 0., .1);
      glEnd();
      glPopMatrix();
    }
  }
};


void displayData(){
  uint16A kinect_depth(FILE("z.kinect_depth"));
  arr pts;
  arr cols;
  MLR::depthData2pointCloud(pts, kinect_depth);
  cols.resizeAs(pts);

  int width=kinect_depth.d1;
  int height=kinect_depth.d0;
  int num_pixels=width*height;
  int num_labels=6;

  intA labels(num_pixels);
  for(uint i=0;i<labels.N;i++) labels.elem(i) = rnd(num_labels);

  for(uint i=0;i<labels.N;i++) cols[i].setCarray(colorsTab[labels(i)%6], 3);

  boolA ok(num_pixels);
  for(uint i=0;i<ok.N;i++) if(pts(i,2)>=0) ok(i)=true; else ok(i)=false;

  //-- generate features
  arr phi(num_pixels, 3);
  phi = pts;
//  for(uint i=0;i<phi.d0;i++) phi(i,0) = pts(i,2); //depth only

  //-- models
  mlr::Array<MinEigModel> M(num_labels);

  ModelDrawer D(M);
  OpenGL gl;
//  gl.add(glDrawAxes);
  gl.add(glDrawPointCloud, &pts);
  gl.addDrawer(&D);
  gl.camera.setKinect();
  gl.camera.setPosition(0., 0., -1.);


  for(uint k=0;k<10;k++){
    for(MinEigModel &m:M) m.resetStatistics(phi.d1);

    //-- assign data
    for(int i=0;i<num_pixels;i++){
      if(ok(i)) M(labels(i)).addStatistics(phi[i]);
    }
    for(MinEigModel &m:M) m.comBeta();


    //-- compute costs
    double C=0.;
    for(int i=0; i<num_pixels;i++) if(ok(i)) C += M(labels(i)).cost(phi[i]);
    cout <<"cost = " <<C <<endl;
    C=0.;
    for(MinEigModel &m:M) C += m.beta_len;
    cout <<"cost = " <<C <<endl;

    gl.watch();


    intA data(num_pixels,num_labels);
    data.setZero();
    for(uint i=0; i<data.d0;i++) if(ok(i))
      for(uint l=0; l<data.d1;l++)
        data(i,l) = scale*M(l).cost(phi[i]);

#if 1
    struct MySmoothCostFunctor:GCoptimization::SmoothCostFunctor {
      boolA& ok;
      MySmoothCostFunctor(boolA& ok):ok(ok){}
      virtual GCoptimization::EnergyTermType compute(GCoptimization::SiteID s1, GCoptimization::SiteID s2, GCoptimization::LabelID l1, GCoptimization::LabelID l2){
        if(!ok(s1) || !ok(s2)) return 0;
        else return (l1==l2) ? 0 : 10;
      }
    } SCF(ok);
#else
    intA smooth(num_labels,num_labels);
    for(uint l1=0; l1<smooth.d0; l1++)
      for(uint l2=0; l2 <smooth.d1; l2++)
        smooth(l1,l2) = (l1==l2) ? 0 : 10;
#endif

    GCoptimizationGridGraph *gc = new GCoptimizationGridGraph(width,height,num_labels);
    gc->setDataCost(data.p);
//    gc->setSmoothCost(smooth.p);
    gc->setSmoothCostFunctor(&SCF);
    for(int i=0; i<num_pixels;i++) gc->setLabel(i, labels(i));
    for(uint k=0;k<1;k++){
      cout <<"k=" <<k <<" energy= " <<(double)gc->compute_energy()/scale <<endl;
      gc->expansion(1);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
      cout <<"k=" <<k <<" energy= " <<(double)gc->compute_energy()/scale <<endl;
      gc->swap(1);
    }
    //    cout <<"\nAfter optimization energy is " <<gc->compute_energy() <<endl;

    for(uint i=0; i<labels.N; i++) labels(i) = gc->whatLabel(i);

    for(uint i=0;i<labels.N;i++) cols[i].setCarray(colorsTab[labels(i)%6], 3);
//    gl.watch();
  }
}

void TEST(GCO) {
  

  int width=100;
  int height=100;
  int num_pixels=width*height;
  int num_labels=10;

  intA result(num_pixels);   // stores result of optimization

  // first set up the array for data costs
  intA data(num_pixels,num_labels);
  for(uint i=0; i<data.d0;i++)
    for(uint l=0; l<data.d1;l++)
      if(i<25){
	if(l==0) data(i,l)=0; else data(i,l)=10;
      }else{
	if(l==5) data(i,l)=0; else data(i,l)=10;
      }
  // next set up the array for smooth costs
  intA smooth(num_labels,num_labels);
  for(uint l1=0; l1<smooth.d0; l1++)
    for(uint l2=0; l2 <smooth.d1; l2++)
      smooth(l1,l2) = ((l1-l2)*(l1-l2) <= 4) ? (l1-l2)*(l1-l2) : 4;


  try{
    GCoptimizationGridGraph *gc = new GCoptimizationGridGraph(width,height,num_labels);
    gc->setDataCost(data.p);
    gc->setSmoothCost(smooth.p);
    for(uint k=0;k<1;k++){
      cout <<"k=" <<k <<" energy= " <<gc->compute_energy() <<endl;
      gc->expansion(1);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
      cout <<"k=" <<k <<" energy= " <<gc->compute_energy() <<endl;
      gc->swap(1);
    }
    //    cout <<"\nAfter optimization energy is " <<gc->compute_energy() <<endl;

    for(uint i=0; i<result.N; i++) result(i) = gc->whatLabel(i);
    
    delete gc;
  }catch (GCException e){
    e.Report();
  }

}


int MAIN(int argc,char** argv){

  displayData();
//  testGCO();

  return 0;
}
