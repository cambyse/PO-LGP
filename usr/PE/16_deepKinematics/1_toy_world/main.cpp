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

  world.getBodyByName("box")->X.addRelativeRotationDeg(15,0.,0.,1.);
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
    MLR_MSG("image not completely rendered!"); return 0;
  }
  cout <<"max D=" <<depth_image.max() <<" min d=" <<depth_image.min() <<endl;

  //  flip_image(captureImage);

  /// convert depth image to pointcloud
  images2pointcloud(pts,cols,captureImage,depth_image);
  //depthData2pointCloud(pts,depth_image);

  for(uint i=0;i<pts.d0;i++) pts(i,2) *= -1.; // switch z dimension of points
  ors::Transformation T = gl.camera.X;
  T.applyOnPointArray(pts);
  cout << T.rot.getArr() << endl;

  /// visualize pointcloud
  mesh.V = pts;
  mesh.C = cols;
  gl.add(mesh);
  gl.watch();

  /// save image and depth_image to file
  write_ppm(gl.captureImage, "z.ppm", true);
  byteA D = convert<byte, float>(captureDepth*255.f);
  write_ppm(D, "z.d.ppm", true);

  return 0;
}
