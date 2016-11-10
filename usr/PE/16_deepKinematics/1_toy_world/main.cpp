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
  ors::KinematicWorld world("toy.ors");
  double width = 640;
  double height = 480;
  world.gl().resize(width,height);
  world.gl().camera.setKinect();
  world.gl().camera.setPosition(0., -.5, .5);
  world.gl().camera.focus(0.,0.,0.);

  world.getBodyByName("box")->X.addRelativeRotationDeg(30,0.,0.,1.);
  world.calc_fwdPropagateFrames();

  world.gl().renderInBack(true, true,width,height);

  world.watch(true);

  ors::Mesh mesh;
  arr& pts = mesh.V;
  arr& cols = mesh.C;

  byteA captureDepth = world.gl().captureDepth; // unsigned char byte
  byteA captureImage = world.gl().captureImage; // unsigned char byte

  // convert byteA to uint16A
  uint16A depth_image((const uint16_t*)captureDepth.p, captureDepth.N);
  depth_image.reshape(captureDepth.d0,captureDepth.d1);

  images2pointcloud(pts,cols,captureImage,depth_image);
  //  depthData2pointCloud(pts,depth_image);


//  pts = pts*world.gl().camera.X.rot.getArr();
//  for (uint i = 0;i<pts.d0;i++) {
//    double x,y,z;
//    x = pts(i,0);
//    y = pts(i,0);
//    z = pts(i,0);
//    world.gl().unproject(x,y,z);
//    pts[i] = ARR(x,y,z);
//  }

  world.gl().add(mesh);
  world.watch(true);
//  world.gl().watchImage(world.gl().captureDepth,true,1);

  write_ppm(world.gl().captureImage, "z.ppm", true);
  write_ppm(world.gl().captureDepth, "z.d.ppm", true);
  pts >> FILE("pts.dat");
//  write_ppm(depth_image, "z.d2.ppm", true);


  return 0;
}
