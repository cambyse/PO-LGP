#include <Ors/ors.h>
#include <Ors/ors_swift.h>
#include <Ors/ors_ode.h>
#include <Algo/spline.h>
#include <Algo/algos.h>
#include <Gui/opengl.h>
#include <Gui/plot.h>
#include <GL/gl.h>
#include <Optim/optimization.h>
#include <Hardware/kinect/kinect.h>
#include <Perception/kinect2pointCloud.h>

void TEST(CapturePointcloud) {
  /// initialize world
  double w = 640;
  double h = 480;
  ors::KinematicWorld world("toy.ors");
  OpenGL gl("background", w,h);
  gl.add(glStandardScene, 0);
  gl.add(world);

  /// set camera position
  gl.camera.setKinect();
  gl.camera.setPosition(0., -.5, .5);
  gl.camera.focus(0.,0.,0.);

  /// add simple box to scene
  ors::Body *boxBody = new ors::Body(world);
  boxBody->name = STRING("box");
  boxBody->type = ors::BodyType::dynamicBT;
  boxBody->X.pos = ors::Vector(0.,0.,.05);
  ors::Shape *boxShape = new ors::Shape(world, *boxBody);
  boxShape->type = ors::boxST;
  boxShape->name = STRING("box");
  arr size = ARRAY(0.2,0.2,0.1, 0.0);
  memmove(boxShape->size, size.p, 4*sizeof(double));
  arr color = ARRAY(0.,0.,0.);
  memmove(boxShape->color, color.p, 3*sizeof(double));

  world.calc_fwdPropagateFrames();

  /// render scene
  gl.update("", true, true, true);
//  gl.renderInBack(true, true, w, h);

  /// get image and depth image
  ors::Mesh mesh;
  arr pts, cols;
  floatA captureDepth = gl.captureDepth;
  byteA captureImage = gl.captureImage;

  /// convert byteA to uint16A
  uint16A depth_image;
  depth_image.resize(captureDepth.d0,captureDepth.d1);
  for (uint i=0;i<depth_image.N;i++) {
    double d = (double) captureDepth.elem(i);
    gl.camera.glConvertToTrueDepth(d);
    depth_image.elem(i) = (uint16_t) (d*1000.); // conv. from [m] -> [mm]
  }
  if (captureDepth.min()==0) {
    cout <<"max D=" <<captureDepth.max() <<" min d=" <<captureDepth.min() <<endl;
    MLR_MSG("image not completely rendered!"); return;
  }
  cout <<"max D=" <<depth_image.max() <<"mm, min d=" <<depth_image.min() <<"mm" <<endl;

  //  flip_image(captureImage);

  /// convert depth image to pointcloud
  images2pointcloud(pts,cols,captureImage,depth_image);
  //depthData2pointCloud(pts,depth_image);

  /// transform depth image into world frame
  for(uint i=0;i<pts.d0;i++) pts(i,2) *= -1.; // switch z dimension of points
  ors::Transformation T = gl.camera.X;
  T.applyOnPointArray(pts);

  /// visualize pointcloud
  mesh.V = pts*1.;
  mesh.C = cols;
  gl.add(mesh);
  gl.watch();

  /// save image and depth_image to file
  write_ppm(gl.captureImage, "z.ppm", true);
  byteA D = convert<byte, float>(captureDepth*255.f);
  write_ppm(D, "z.d.ppm", true);
  pts >> FILE("pts.dat");
  cols >> FILE("cols.dat");
}


void TEST(GenerateDataset) {
  /// initialize world
  double w = 640;
  double h = 480;
  ors::KinematicWorld world("toy.ors");
  OpenGL gl("background", w,h);

  gl.add(glStandardScene, 0);
  gl.add(world);

  /// set camera position
  gl.camera.setKinect();
  gl.camera.setPosition(0., -.5, .5);
  gl.camera.focus(0.,0.,0.);


  arr theta(1);
  arr limits = ARR(1.,2.);
  uint N = 20;

  ors::Mesh mesh;
  gl.add(mesh);
  for (uint n=0;n<N;n++) {
    theta(0) = limits(0)+n/(double)(N-1)*(limits(1)-limits(0));

    /// add simple box to scene
    ors::Body *boxBody = new ors::Body(world);
    boxBody->name = STRING("box");
    boxBody->type = ors::BodyType::dynamicBT;
    boxBody->X.pos = ors::Vector(0.,0.,.05);
    ors::Shape *boxShape = new ors::Shape(world, *boxBody);
    boxShape->type = ors::boxST;
    boxShape->name = STRING("box");
    arr size = ARRAY(theta(0)*0.2,theta(0)*0.2,0.1, 0.0);
    memmove(boxShape->size, size.p, 4*sizeof(double));
    arr color = ARRAY(0.,0.,0.);
    memmove(boxShape->color, color.p, 3*sizeof(double));
    world.calc_fwdPropagateFrames();

    /// render scene
    gl.update("", true, true, true);
  //  gl.renderInBack(true, true, w, h);

    /// get image and depth image
    arr pts, cols;
    floatA captureDepth = gl.captureDepth;
    byteA captureImage = gl.captureImage;

    /// convert byteA to uint16A
    uint16A depth_image;
    depth_image.resize(captureDepth.d0,captureDepth.d1);
    for (uint i=0;i<depth_image.N;i++) {
      double d = (double) captureDepth.elem(i);
      gl.camera.glConvertToTrueDepth(d);
      depth_image.elem(i) = (uint16_t) (d*1000.); // conv. from [m] -> [mm]
    }

    if (captureDepth.min()==0) {
      cout <<"max D=" <<captureDepth.max() <<" min d=" <<captureDepth.min() <<endl;
      MLR_MSG("image not completely rendered!"); return;
    }

    /// convert depth image to pointcloud
    images2pointcloud(pts,cols,captureImage,depth_image);
    //depthData2pointCloud(pts,depth_image);

    /// transform depth image into world frame
    for(uint i=0;i<pts.d0;i++) pts(i,2) *= -1.; // switch z dimension of points
    ors::Transformation T = gl.camera.X;
    T.applyOnPointArray(pts);

    /// visualize pointcloud
//    mesh.V = pts*1.;
//    mesh.C = cols;
//    gl.watch();

    /// save image and depth_image to file
    mlr::String folder("dataset1/");
    write_ppm(gl.captureImage, STRING(folder<<n<<"_z.ppm"), true);
    byteA D = convert<byte, float>(captureDepth*255.f);
    write_ppm(D, STRING(folder<<n<<"_d.ppm"), true);
    pts >> FILE(STRING(folder<<n<<"_pts.dat"));
    cols >> FILE(STRING(folder<<n<<"_cols.dat"));
    theta >> FILE(STRING(folder<<n<<"_theta.dat"));

    delete boxBody;
    mesh.V.clear();
    mesh.C.clear();
  }

  return;
}


int main(int argc,char **argv){
  testCapturePointcloud();
//  testGenerateDataset();
  return 0;
}
