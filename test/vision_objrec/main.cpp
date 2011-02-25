#include <MT/array.h>
#include <MT/algos.h>
#include <MT/ors.h>
#include <MT/ors_opengl.h>
#include <MT/opengl.h>
#include <MT/ors_qhull.h>
#include <MT/BinaryBP.h>
#include <MT/vision.h>

#include <NP_2Drec/common.h>
#include <NP_2Drec/image.h>
#include <NP_2Drec/vision_module.h>
#include <NP_2Drec/camera/bumblebee2.h>
#include <NP_2Drec/vision_low/tracking.h>
#include <NP_2Drec/ransac.h>
#include <NP_2Drec/extern/opensurf.h>

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <vector>

extern "C" {
#include <getopt.h>
#include <sys/time.h>
}

#include <opencv/highgui.h>
#include <opencv/cv.h>
void draw_points(byteA& frame, doubleA& points, int r = 255, int g = 255, int b = 255, int s = 3)
{
  int num_points = points.d0;
  if (num_points <= 0)
    ERROR("no points to draw");

  IplImage* frame_cv = cvCreateImageHeader(cvSize(frame.d1, frame.d0), IPL_DEPTH_8U, (frame.d2 == 3 ? 3 : 1));
  frame_cv->origin = IPL_ORIGIN_TL;             // image origin (top-left)
  frame_cv->widthStep = (frame.d2 == 3 ? 3 : 1) * frame.d1;
  frame_cv->imageData = (char*) frame.p;

  for (int i = 0; i < num_points; i++)
  {
    cvCircle(frame_cv, cvPoint(points(i,0),points(i,1)), s, cvScalar(b,g,r),-1);
  }

  cvReleaseImageHeader(&frame_cv);
}

void test_object_recording()
{
  //vision::record_object_data("reiniger", "./objects_db");
//   vision::record_object_data("schwamm", "./objects_db");
// vision::record_object_data("muesli", "./objects_db");
//    vision::record_object_data("erdnuss", "./objects_db");
//    vision::record_object_data("tee", "./objects_db");
//    vision::record_object_data("bild1", "./objects_db");
//    vision::record_object_data("bild2", "./objects_db");
//   vision::record_object_data("dose1", "./objects_db");
//  vision::record_object_data("dose2", "./objects_db");
//  vision::record_object_data("dose3", "./objects_db");
//  vision::record_object_data("blatt", "./objects_db");

//  vision::record("test", "./objects_db");
//   vision::record("teekanne", "./objects_db");
}

void test()
{
  // left_object database
  vision::ObjectList ol;
  vision::load_object_library(ol, "./objects_db");

  // camera
  camera::Bumblebee2 camera;
  camera.init(49712223529894949LL);
  byteA left, right, stereo;

  // transformation matrices
  doubleA q;                                          // 2D to 3D
  camera.get_matrix_q(q);

  // feature matching

  // OpenGL/ORS stuff
  ors::Graph ors;
  ors::Body *n = new ors::Body;
  ors.bodies.append(n);
  ors::Shape *s = new ors::Shape;
  n->shapes.append(s);
  s->body=n;
  s->type = BCCYLINDER;
  s->size[2]=.05; s->size[3]=.1;
  s->color[0]=1.;
  OpenGL gl;
  gl.add(glStandardScene);
  gl.add(ors::glDrawGraph,&ors);

  // OpenCV
  cvNamedWindow("Detection", CV_WINDOW_AUTOSIZE);
  std::ostringstream text;
  vision::Detection2DList D;
  uint t_det = 10; double t_outlier = 100.;
  uint num_nn = 25; double t_nn =  0.0575;
  for (int i = 0;;i++)
  {
    // capture stereo frame and switch from RGB to BGR
    camera.capture(left, right);
    resize(left, left, 0.5);
    resize(right, right, 0.5);

    // skip the first couple frames allowing camera sensors settle
    if (i < 10)
      continue;

    vision::detect(D, left, right, ol, t_det, t_outlier, t_nn, num_nn, q);

    // display detected object on screen
    for (uint obi = 0; obi < D.N; obi++)
    {
      vision::Detection2D *o = D(obi);

      vision::draw_interest_points(left, o->F_left.keypoints, 0);
//       vision::draw_object(left, o->C_left2, o->c_left2, "", 0, 0, 255);
      vision::draw_object(left, o->C_left2, o->c_left2, o->identifier.c_str(), 255, 0, 0);
      vision::draw_interest_points(right, o->F_right.keypoints, 0);
//       vision::draw_object(right, o->C_right2, o->c_right2, "", 0, 0, 255);
      vision::draw_object(right, o->C_right2, o->c_right2, o->identifier.c_str(), 255, 0, 0);
      std::cout << "c3d = " << o->c_3D << std::endl;
//       n->X.p = ors::Vector(o->c_3D(0,0), o->c_3D(0,1), o->c_3D(0,2));
//       gl.update();
//       std::cout << o->identifier.c_str() << ": c_left = " << o->c_left << ", c_right = " << o->c_right << ", c_3D = " << o->c_3D << std::endl;
    }

    vision::to_stereo(stereo, left, right);

    /*for (uint obi = 0; obi < D.N; obi++)
    {
      vision::Detection2D *o = D(obi);
      if (o->E_lr.N == 0)
        std::cout << "o->E_lr.N == 0" << std::endl;
      
      vision::drawEdges(stereo, o->E_lr, o->F_left, o->F_right);
    }*/

    
    
    vision::cvdisplay(stereo, "Detection");

    // check user input
    char c = cvWaitKey(15);
    switch (c)
    {
        case 'p':
          c = 0;
          std::cout << "Camera paused. Press 'p' to continue." << std::endl;
          while(c != 'p' && c != 27)
              c = cvWaitKey(250);
          std::cout << "Camera is capturing again." << std::endl;
          break;
        case 's':
          break;
        default:
          break;
    }
    if(c == 27)
        break;
  }

  cvDestroyAllWindows();
  camera.deinit();
}

void test_new()
{
  // left_object database
  vision::ObjectList objects;
  vision::load_object_library(objects, "./objects_db");

  // camera
  camera::Bumblebee2 camera;
  camera.init(49712223529894949LL);
  byteA left_frame, right_frame;

  // matching etc.
  vision::FramesMatch2DA left_matches, right_matches;

  // transformation matrices
  doubleA q;                                          // 2D to 3D
  camera.get_matrix_q(q);

  // feature matching
  visionlow::FeatureTracker left_ft, right_ft;       // stereo matching
  vision::LocalizedObjectList detected_objects;
  int num_detected_objects;

  // OpenCV
  cvNamedWindow("LEFT", CV_WINDOW_AUTOSIZE);
  cvMoveWindow("LEFT", 700, 100);
  cvNamedWindow("RIGHT", CV_WINDOW_AUTOSIZE);
  cvMoveWindow("RIGHT", 1222, 100);
  std::ostringstream text;

  // OpenGL/ORS
  ors::Graph ors;
  ors::Body *n = new ors::Body;
  ors.bodies.append(n);

  ors::Shape *s = new ors::Shape;
  n->shapes.append(s);
  s->body=n;
  s->type = BCCYLINDER;
  s->size[2]=.05; s->size[3]=.1;
  s->color[1]=1.;

  OpenGL gl;
  gl.add(glStandardScene);
  gl.add(ors::glDrawGraph,&ors);

  for (int i = 0;;i++)
  {
    vision::StereoFrameData sfd;
    camera.capture(left_frame, right_frame);
    if (i < 5)
      continue;
    resize(left_frame, left_frame, 0.5);
    resize(right_frame, right_frame, 0.5);

    // compute interest points
    opensurf::compute_pois_and_descriptors(sfd.left_keypoints, sfd.left_descriptors, left_frame, false, 4, 6, 1, 0.001f );
    opensurf::compute_pois_and_descriptors(sfd.right_keypoints, sfd.right_descriptors, right_frame, false, 4, 6, 1, 0.001f);

    // detect objects in the scene and display findings
    vision::StereoFrameData final_features;
    num_detected_objects = vision::detect_objects(detected_objects, final_features, sfd, objects, q);
    for (int obi = 0; obi < num_detected_objects; obi++)
    {
      std::cout << "obi = " << obi << ", name = " << detected_objects(obi)->name << std::endl;
      vision::draw_object(left_frame, detected_objects(obi)->left_contour_2D, detected_objects(obi)->left_center_2D, detected_objects(obi)->name.c_str(), 255, 255, 0);
      vision::draw_object(right_frame, detected_objects(obi)->right_contour_2D, detected_objects(obi)->right_center_2D, detected_objects(obi)->name.c_str(), 255, 255, 0);
      n->X.p = ors::Vector(detected_objects(obi)->center_3D(0,0),detected_objects(obi)->center_3D(0,1),detected_objects(obi)->center_3D(0,2));
      //n->X.r = ors::Quaternion(worldP(0),worldP(1),worldP(2));
      gl.update();
    }
    if (num_detected_objects <= 0)
      n->X.p = ors::Vector(-100.,-100.,-100.);

    // display keypoints
//     vision::draw_interest_points(left_frame, sfd.left_keypoints, 0);
//     vision::draw_interest_points(right_frame, sfd.right_keypoints, 0);
    vision::draw_interest_points(left_frame, final_features.left_keypoints, 0);
    vision::draw_interest_points(right_frame, final_features.right_keypoints, 0);

    // display frames
    vision::cvdisplay(left_frame,"LEFT");
    vision::cvdisplay(right_frame, "RIGHT");

    // check user input
    char c = cvWaitKey(15);
    switch (c)
    {
      case 'p':
        c = 0;
        std::cout << "Camera paused. Press 'p' to continue." << std::endl;
        while(c != 'p' && c != 27)
          c = cvWaitKey(250);
        std::cout << "Camera is capturing again." << std::endl;
        break;
      case 's':
        break;
      default:
        break;
    }
    if(c == 27)
      break;
  }

  cvDestroyAllWindows();
  camera.deinit();
}

void temp()
{
}

int main(int argc, char** argv)
{
  MT::initCmdLine(argc,argv);

  switch (MT::getParameter<uint>("mode",2))
  {
    case 0:  vision::record(MT::getParameter<MT::String>("name"), "./objects_db");  break;
    case 1: test(); break;
    case 2: test_new(); break;
    default: test(); break;
  }

   return 0;
}
