#include <MT/array.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <opencv/cv.h>

#include <common.h>
#include <opencv/highgui.h>
#include <visualize.h>
#include <wrappers/libcolorseg.h>
#include <wrappers/opensurf.h>
#include <voc/voc.h>
#include <voc/vocfeatures.h>
#include <image.h>
#include <kmeans.h>

extern "C" {
#include <pthread.h>
}

void group_features()
{
  std::cout << "writing features belonging to the same object class to one file" << std::endl;
  // group features belonging to the same object class
  stringA classes;
  voc::load_file_list(classes, "data/classes");
  std::ostringstream css;

  for (uint cli = 1 /* skip 0 - background */; cli < classes.d0; cli++)
  {
    css.str("");
    css << "./data/file_lists/train/" << classes(cli, 0) << ".files";
    voc::group_raw_features(css.str().c_str(), "/home/data/voc2009/features/", classes);
  }
};

void clusters2codebook(doubleA codebook, int size)
{
  doubleA clusters;
  int num_dim = -1;
  int num_codewords = 0;
  stringA classes(20);
  voc::load_file_list(classes, "data/classes");
  std::ostringstream out, in;
  int num_codewords_class = size;

  // count code words
  for (uint cli = 1 /* skip 0 - background */; cli < classes.d0; cli++)
  {
    in.str("");
    in << "/home/data/voc2009/features/" << classes(cli, 0) << "." << num_codewords_class << ".clusters";
    std::cout << in.str() << std::endl;

    load_single_array(clusters, in.str().c_str(), "doubleA_class_clusters");
    num_codewords += clusters.d0;
    if (num_dim == -1)
      num_dim = clusters.d1;

    if (num_dim != clusters.d1)
      ERROR("num_dim != clusters.d1");
  }

  // create codebook
  int cnt = 0;
  codebook.resize(num_codewords, num_dim);
  for (uint cli = 1 /* skip 0 - background */; cli < classes.d0; cli++)
  {
    in.str("");
    in << "/home/data/voc2009/features/" << classes(cli, 0) << "." << num_codewords << ".clusters";

    load_single_array(clusters, in.str().c_str(), "doubleA_class_clusters");
    for (int cwi = 0; cwi < clusters.d0; cwi++)
      codebook[cnt++] = clusters[cwi];
  }

  out.str("");
  out << "/home/data/voc2009/codebooks/" << num_codewords << ".codebook";
  std::cout << out.str() << std::endl;
  save_single_array(codebook, out.str().c_str(), "doubleA_codebook");
};

void generate_visual_codebook(doubleA codebook, int num_clusters_per_class = 100, int max_iter = 200, double threshold = 0.001, int num_threads = 8)
{
//   group_features();
  doubleA clusters, descriptors;

  stringA classes;
  voc::load_file_list(classes, "data/classes");
  std::ostringstream out, in;
  for (uint cli = 1 /* skip 0 - background */; cli < 18/*classes.d0*/; cli++)
  {
    in.str("");
    in << "/home/data/voc2009/features/" << classes(cli, 0) << ".surfRGB";
    out.str("");
    out << "/home/data/voc2009/features/" << classes(cli, 0) << "." << num_clusters_per_class << ".clusters";
    std::cout << out.str() << std::endl;

    load_single_array(descriptors, in.str().c_str(), "doubleA_class_descriptors");
    voc::gen_clusters(clusters, descriptors, num_clusters_per_class, max_iter, threshold, num_threads);
    save_single_array(clusters, out.str().c_str(), "doubleA_class_clusters");
  }

  clusters2codebook(codebook, num_clusters_per_class);
};

int main(int argc, char** argv)
{
  int num_threads = 8;

  // load file list, parameters, etc.
  stringA list, classes;
  voc::load_file_list(list,    "data/file_lists/test.files");
  voc::load_file_list(classes, "data/classes");

  // compute raw image features
  doubleA raw_feature_params;
  load_single_array(raw_feature_params, "data/raw_feature.parameters", "doubleA_raw_features");
  voc::raw_features(list, raw_feature_params, "voc2009_temp/features/", num_threads);

  // generate visual codebooks
//   doubleA codebook;
//   generate_visual_codebook(codebook, 5, 200, 0.001, num_threads);
//   generate_visual_codebook(codebook, 10, 200, 0.001, num_threads);
//   generate_visual_codebook(codebook,  25, 200, 0.001, num_threads);
//   generate_visual_codebook(codebook,  50, 200, 0.001, num_threads);
//   generate_visual_codebook(codebook,  75, 200, 0.001, num_threads);
//   generate_visual_codebook(codebook, 100, 200, 0.001, num_threads);
//   load_single_array(codebook, "/home/data/voc2009/codebooks/2000.codebook", "doubleA_codebook");

  // quantize features
//   load_single_array(codebook, "/home/data/voc2009/codebooks/500.codebook", "doubleA_codebook");
//   voc::features2xysc(list, codebook, num_threads);
//   load_single_array(codebook, "/home/data/voc2009/codebooks/1000.codebook", "doubleA_codebook");
//   voc::features2xysc(list, codebook, num_threads);
//   load_single_array(codebook, "/home/data/voc2009/codebooks/1500.codebook", "doubleA_codebook");
//   voc::features2xysc(list, codebook, num_threads);
//   load_single_array(codebook, "/home/data/voc2009/codebooks/2000.codebook", "doubleA_codebook");
//   voc::features2xysc(list, codebook, num_threads);

  // global image histogram

  // patch histograms

  // train (image+patch) classifiers

  // construct trees

  // do inference

  // save final segmentation
  
//   raw_image_features("data/file_lists/train.files", 8);
//   group_features();
//   generate_visual_codebook();
//   clusters2codebook();
//   voc::quantize_features("/home/data/voc2009/codebooks/2000.codebook", "data/file_lists/train.files");

  
//   byteA image, image2;
//   doubleA image_t;
//   vision::load_image(image, "image.jpg");
//   doubleA descriptors, keypoints, grid;
//   float scale = 1.2;
//   voc::regular_grid(grid, image.d1, image.d0, image.d1/4, 10);
//   opensurf::compute_descriptors_rgb(descriptors, keypoints, image, grid, scale);
// 
//   std::cout << "grid " << std::endl << grid << std::endl;
//   std::cout << "keypoints " << std::endl << keypoints << std::endl;
//   std::cout << "descriptors " << std::endl << descriptors << std::endl;



//   voc::Parameters p;
//   voc::ImageFeatures imf;
//   voc::FileList fl;
//   int num_pix = voc::load_file_list(fl, "test.txt");
//   byteA image;
//   voc::load_parameters(p, "parameters.array");
// 
//   std::cout << "num pix " << num_pix << std::endl;
//   for (int i = 0; i < 1/*num_pix*/; i++)
//   {
//     std::cout << "img " << std::setw(2) << i << std::flush;
//     vision::load_image(image, fl.p[i]);
//     std::cout << " done" << std::endl;
//   }
// 
//   byteA cf;
//   doubleA centers, grid, descriptors, keypoints, color_descriptors;
//   std::ostringstream oss;
//   for (int i = 0; i < 1/*imf.pyramid.d0*/; i++)
//   {
//     oss.str("");
//     oss << "image_l" << i << ".png";
//     copy(cf, image);
// //     colorize_patches(cf, imf.pyramid.p[i].patches, imf.color.p[i].descriptors);
//     voc::regular_grid(grid, image.d1, image.d0, 6, 5);
//     opensurf::compute_descriptors(descriptors, keypoints, image, grid, 1.2);
//     voc::color_tiles(color_descriptors, image, grid);
//     vision::draw_points2D(image, keypoints, 1.);
// //     vision::draw_points2D(cf, imf.pyramid.p[i].centers, 2.);
//     vision::save_image(oss.str(), image);
//   }
//   std::cout << color_descriptors << std::endl;
// 
//   

  return 0;
}
