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

int main(int argc,char **argv){
  double width = 640;
  double height = 480;

  // initialize world and set camera
  ors::KinematicWorld world("toy.ors");
  world.gl().resize(width,height);

  world.gl().camera.setKinect();
  world.gl().camera.setPosition(0., -.5, .5);
  world.gl().camera.focus(0.,0.,0.);

  world.getBodyByName("box")->X.addRelativeRotationDeg(30,0.,0.,1.);
  world.calc_fwdPropagateFrames();
  world.gl().update("simulator",true,true);

  world.gl().renderInBack(true, true, width,height);

  // get depth image and compute pointcloud
  ors::Mesh mesh;
  arr& pts = mesh.V;
  arr& cols = mesh.C;
  byteA captureDepth = world.gl().captureDepth;
  byteA captureImage = world.gl().captureImage;

  // convert byteA to uint16A
  uint16A depth_image((const uint16_t*)captureDepth.p, captureDepth.N);
  depth_image.reshape(captureDepth.d0,captureDepth.d1);

  images2pointcloud(pts,cols,captureImage,depth_image);
//  depthData2pointCloud(pts,depth_image);

//  ors::Transformation T;
//  T = world.gl().camera.X;
//  T.setInverse(world.gl().camera.X);
//  T.applyOnPointArray(pts);

  world.gl().add(mesh);
  world.watch(true);

  write_ppm(world.gl().captureImage, "z.ppm", true);
  write_ppm(world.gl().captureDepth, "z.d.ppm", true);

  return 0;
}
