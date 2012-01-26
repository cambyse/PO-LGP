
void patch_network_structure(uintA& centroids_x, uintA& centroids_y, boolA& edges, const uintA& segmentation) {
  uint HEIGHT = segmentation.d0;
  uint WIDTH = segmentation.d1;
  // determine number of patches and centroids
  uintA patches(HEIGHT, WIDTH); // uses my own patch ids
  uintA patches_colors;
  uintA patches_sizes;
  uintA patches_Xs;
  uintA patches_Ys;
  MT::Array< uintA > patches_neighbors;
  int idx;
  uint y, x;
  FOR2D(segmentation, y, x) {
    idx = patches_colors.findValue(segmentation(y, x)); if (idx < 0) {
      idx = patches_colors.N;
      patches_sizes.append(0);
      patches_Xs.append(0);
      patches_Ys.append(0);
      patches_colors.append(segmentation(y,x));
    }
    patches(y,x) = idx;
    patches_sizes(idx)++;
    patches_Xs(idx) += x;
    patches_Ys(idx) += y;
  }
  
  uint num_patches = patches_sizes.N;
  uint k;
  FOR1D(patches_sizes, k) {
    centroids_x.append((1.0 * patches_Xs(k)) / patches_sizes(k));
    centroids_y.append((1.0 * patches_Ys(k)) / patches_sizes(k));
  }
  
  // determine neighbors
  edges.resize(num_patches, num_patches);
  edges.setUni(false);
  FOR2D(patches, y, x) {
    // upper neighbor
    if (y > 0) {
      if (patches(y,x) != patches(y-1,x)) {
        edges(patches(y,x), patches(y-1,x)) = true;
        edges(patches(y-1,x), patches(y,x)) = true;
      }
    }
    // left neighbor
    if (x > 0) {
      if (patches(y,x) != patches(y,x-1)) {
        edges(patches(y,x), patches(y,x-1)) = true;
        edges(patches(y,x-1), patches(y,x)) = true;
      }
    }
    // right neighbor
    if (x < WIDTH-1) {
      if (patches(y,x) != patches(y,x+1)) {
        edges(patches(y,x), patches(y,x+1)) = true;
        edges(patches(y,x+1), patches(y,x)) = true;
      }
    }
    // lower neighbor
    if (y < HEIGHT-1) {
      if (patches(y,x) != patches(y+1,x)) {
        edges(patches(y,x), patches(y+1,x)) = true;
        edges(patches(y+1,x), patches(y,x)) = true;
      }
    }
  }
}



void patch_colors(byteA& patches_colors, const byteA& image, uint patches_num, const uintA& image_patches) {
  patches_colors.resize(patches_num, 3);
  uintA patch_sizes(patches_num);
  patch_sizes.setUni(0);
  uint y, x, k;
  FOR2D(image, y, x) {
    patches_colors(image_patches(y, x), 0) += image(y,x,0);
    patches_colors(image_patches(y, x), 1) += image(y,x,1);
    patches_colors(image_patches(y, x), 2) += image(y,x,2);
  }
  FOR1D(patches_colors, k) {
    if (patches_colors(k, 0) > 0) patches_colors(k, 0) /= patch_sizes(k);
    if (patches_colors(k, 1) > 0) patches_colors(k, 1) /= patch_sizes(k);
    if (patches_colors(k, 2) > 0) patches_colors(k, 2) /= patch_sizes(k);
  }
}

  
void draw_dot(uintA& image_net, uint x, uint y, uint width, uint symbol) {
  
  int left=x;
  while (left-1 >= 0 && (x-(left-1))<=width)
    left--;
  
  int right=x;
  while (right+1 < image_net.d1 && (x+right+1)<=width)
    right++;
  
  int top=y;
  while (top-1 >= 0 && (y-(top-1))<=width)
    top--;
  
  int bottom=y;
  while (bottom+1 < image_net.d0 && (y+bottom+1)<=width)
    bottom++;
  
  int x1, y1;
  for (x1=left; x1<=right; x1++) {
    for (y1=top; y1<=bottom; y1++) {
      image_net(y1,x1) = symbol;
    }
  }
}


void draw_patch_net(uintA& image_net, uintA& centroids_x, uintA& centroids_y, boolA& edges, uint width, uint height) {
  image_net.resize(width, height);
  image_net.setUni(0);
  uint i, k;
  // centroids
  FOR1D(centroids_x, i) {
    draw_dot(image_net, centroids_x(i), centroids_y(i), 5, 1);
  }
  // edges
  int x_diff, y_diff;
  int x_diff_abs, y_diff_abs;
  int line_x, line_y;
  for (i=0; i<edges.d0; i++) {
    for (k=i+1; k<edges.d1; k++) {
      if (edges(i,k)) {
        x_diff_abs = fabs(centroids_x(k) - centroids_x(i));
        y_diff_abs = fabs(centroids_y(k) - centroids_y(i));
        if (x_diff_abs > y_diff_abs) {
          if (centroids_x(k) > centroids_x(i)) {
            x_diff = centroids_x(k) - centroids_x(i);
            y_diff = centroids_y(k) - centroids_y(i);
            for (line_x = centroids_x(i); line_x<=centroids_x(k); line_x++) {
              line_y = ((line_x - centroids_x(i)) * 1.0 / fabs(x_diff)) * y_diff +  centroids_y(i);
              draw_dot(image_net, line_x, line_y, 0, 2);
            }
          }
          else {
            x_diff = centroids_x(i) - centroids_x(k);
            y_diff = centroids_y(i) - centroids_y(k);
            for (line_x = centroids_x(k); line_x<=centroids_x(i); line_x++) {
              line_y = ((line_x - centroids_x(k)) * 1.0 / fabs(x_diff)) * y_diff +  centroids_y(k);
              draw_dot(image_net, line_x, line_y, 0, 2);
            }
          }
        }
        else {
          if (centroids_y(k) == centroids_y(i)) {
            line_y = centroids_y(k);
            for (line_x = centroids_x(i); line_x<=centroids_x(k); line_x++) {
              draw_dot(image_net, line_x, line_y, 0, 2);
            }
          }
          if (centroids_y(k) > centroids_y(i)) {
            x_diff = centroids_x(k) - centroids_x(i);
            y_diff = centroids_y(k) - centroids_y(i);
            for (line_y = centroids_y(i); line_y<=centroids_y(k); line_y++) {
              line_x = ((line_y - centroids_y(i)) * 1.0 / fabs(y_diff)) * x_diff +  centroids_x(i);
              draw_dot(image_net, line_x, line_y, 0, 2);
            }
          }
          else {
            x_diff = centroids_x(i) - centroids_x(k);
            y_diff = centroids_y(i) - centroids_y(k);
            for (line_y = centroids_y(k); line_y<=centroids_y(i); line_y++) {
              line_x = ((line_y - centroids_y(k)) * 1.0 / fabs(y_diff)) * x_diff +  centroids_x(k);
              draw_dot(image_net, line_x, line_y, 0, 2);
            }
          }
        }
      }
    }
  }
}







#if 0

void testFelzNet(){
  camera::UVCCamera uvc;
  uvc.init("/dev/video0");

               byteA img;
               uintA img_segmentation_original;
               cvNamedWindow("cv window", CV_WINDOW_AUTOSIZE);
               for(;;){
               uvc.capture_frame(img);
    
               byteA img_hsv(img.d0,img.d1,3);
               CvMat frame1 = cvMat(img.d0, img.d1, CV_8UC3, img.p);
               CvMat cv_img_hsv = cvMat(img_hsv.d0, img_hsv.d1, CV_8UC3, img_hsv.p);
               cvCvtColor(&frame1, &cv_img_hsv, CV_RGB2HSV);
    
               vision_low::get_single_color_segmentation(
               img_segmentation_original,  // segmented image
               img,           // input image
               1.5,          // (Gaussian!?) blurring factor
               500,           // similarity threshold
               200            // min. no. of pixels per segment
                                             );
    // patches id: start from 0,...,N
               uintA img_segmentation_processed;
               uint patch_number;
               rename_ids(img_segmentation_processed, patch_number, img_segmentation_original);
    
    // process segmentation image into RGB
               byteA img_segmentation_bgr;
               array2array(img_segmentation_bgr, img_segmentation_original);
               CvMat frame_cv = cvMat(img_segmentation_bgr.d0, img_segmentation_bgr.d1, CV_8UC1, img_segmentation_bgr.p);
               byteA canvas(img.d0, img.d1, 3);
               CvMat cv_img_segmentation_bgr = cvMat(canvas.d0, canvas.d1, CV_8UC3, canvas.p);
               cvCvtColor(&frame_cv, &cv_img_segmentation_bgr, CV_GRAY2BGR);

    // determine patch network structure
               uintA centroids_x, centroids_y;
               boolA edges;
               patch_network_structure(centroids_x, centroids_y, edges, img_segmentation_processed);

    // calculate patches net visualization
               uintA img_patches_net;
               draw_patch_net(img_patches_net, centroids_x, centroids_y, edges, img_segmentation_processed.d0, img_segmentation_processed.d1);
               ofstream out("z.arr");
               img_patches_net.write(out);
    
    // add patches net visualization to overall image
               uint y,x;
               FOR2D(img_patches_net, y, x) {
               if (img_patches_net(y, x) == 1) {
               canvas(y, x, 0) = 0;
               canvas(y, x, 1) = 0;
               canvas(y, x, 2) = 250;
}
               else if (img_patches_net(y, x) == 2) {
               canvas(y, x, 0) = 0;
               canvas(y, x, 1) = 250;
               canvas(y, x, 2) = 0;
}
}
               cvShowImage("cv window", &cv_img_segmentation_bgr);
               char c = cvWaitKey(0);
    
    // determine colors of patch nodes
               byteA patch_colors_hsv;
               patch_colors(patch_colors_hsv, img_hsv, patch_number, img_segmentation_processed);
    
    // put into MRF
               byteA target_hsv(3);
               target_hsv(0)=0;
               target_hsv(1)=200;
               target_hsv(2)=200;
  
               binarySegmentation(img_hsv, target_hsv);
    // TODO
    
               exit(0);
}
               cvDestroyWindow("cv window");
}

#endif
