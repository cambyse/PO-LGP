#include <iostream>
#include <ctime>
#include <cstdio>
#include <limits>

extern "C" {
  #include <getopt.h>
}

#include <NP/nputils.h>
#include <NP/desc.h>
#include <NP/vocutils.h>
#include <NP/rtree.h>
#include <NP/cvutils.h>

#include <opencv/ml.h>

static char shortopts[] = "hd:l:t:i:r:n:f:";
static struct option longopts[] = {
  { "help",            no_argument,            NULL,              'h' },
  /* working mode */
  { "data",            required_argument,      NULL,              'd' },
  { "labels",          required_argument,      NULL,              'l' },
  { "type",            required_argument,      NULL,              't' },
  { "labelidx",        required_argument,      NULL,              'i' },
  { "rtree",           required_argument,      NULL,              'r' },
  { "numtrees",        required_argument,      NULL,              'n' },
  { "fraction",        required_argument,      NULL,              'f' },
  { NULL,              0,                      NULL,              0   }
};

int args(charA& data, charA& labels, uint& type, uint& label_idx, charA& rtrees, uint& num_trees, float& fraction, int argc, char** argv);

int main(int argc, char** argv)
{
  charA labels_file, data_file, rtree_file;
  floatA data, labels, labels_type;
  CvRTrees *rtrees = NULL;
  uint type, num_trees, label_idx;
  float fraction;

  // parse command line arguments
  args(data_file, labels_file, type, label_idx, rtree_file, num_trees, fraction, argc, argv);

  // load data
  np::ucm_final_features_load(data, data_file.p);
//  np::labels_load(labels, labels_file.p);
  np::labels_load(labels, labels_file.p);
  std::cout << labels.d0 << " " << labels.d1 << " " << labels.d2 << std::endl; // TODO debug
  labels_type.resize(labels.d1);
  for (uint i = 0; i < labels_type.d0; i++)
    labels_type(i) = labels(type, i, label_idx);
  std::cout << labels_type.d0 << " " << labels_type.d1 << " " << labels_type.d2 << std::endl; // TODO debug

  std::cout << "starting" << std::endl
  << "fraction  = " << fraction << std::endl
  << "label_idx   = " << label_idx << std::endl
  << "type        = " << type << std::endl
  << "num_trees   = " << num_trees << std::endl
  << "labels_file = " << labels_file.p << std::endl
  << "data_file   = " << data_file.p << std::endl
  << "rtree_file  = " << rtree_file.p << std::endl;

  rtrees = new CvRTrees();
  rtrees->load("zzz.rtree.xml");
//   rtrees = np::rtree_train(data, labels_type, fraction, num_trees, true);
//   if (rtrees != NULL)
//     rtrees->save(rtree_file.p);
//   else
//     std::cout << "rtrees = NULL" << std::endl;
//   std::cout << "done!" << std::endl;

  // testing
  std::cout << "Testing Random Trees ... " << std::endl;
  floatA tlabels, tfeatures, tlabels_temp, predictions;
  np::labels_load(tlabels, "voc2009/features/labels/testsamples.aeroplane.labels.array");
  np::ucm_final_features_load(tfeatures, "voc2009/features/final/testsamples.aeroplane.features.array");

  tlabels_temp.resize(tlabels.d1);
  for (uint i = 0; i < tlabels_temp.d0; i++)
    tlabels_temp(i) = tlabels(type,i,1);

  np::rtree_predict(predictions, rtrees, tfeatures);

  float error=0, terror=0;
  for (uint i = 0; i < tlabels_temp.d0; i++)
  {
    terror = tlabels_temp(i) - predictions(i);
    error += terror*terror;
  }

//   std::cout << predictions << std::endl;
  std::cout << tlabels_temp << std::endl;
  std::cout << "total squared error: " << error << std::endl;

  if (rtrees != NULL)
    delete rtrees;
  return 0;
}

int args(charA& data, charA& labels, uint& type, uint& label_idx, charA& rtrees, uint& num_trees, float& fraction, int argc, char** argv)
{
  int num_args = 4;
  int status = 0;
  int ch = 1;

  // default values
  type = 0;
  num_trees = 100;
  fraction = 0.8;
  label_idx = 0;

  while (ch != -1) 
  {
    ch = getopt_long(argc, argv, shortopts, longopts, NULL);
    switch(ch)
    {
      case 'd':
        data.resize(strlen(optarg)+1);
        data = '\0';
        memcpy(data.p, optarg, strlen(optarg));
        status++;
        break;
      case 'l':
        labels.resize(strlen(optarg)+1);
        labels = '\0';
        memcpy(labels.p, optarg, strlen(optarg));
        status++;
        break;
      case 'r':
        rtrees.resize(strlen(optarg)+1);
        rtrees = '\0';
        memcpy(rtrees.p, optarg, strlen(optarg));
        status++;
        break;
      case 't':
        type = atoi(optarg);
        if (type > 3) type = 3;
        break;
      case 'i':
        label_idx = atoi(optarg);
        status++;
        break;

      case 'n':
        num_trees = atoi(optarg);
        break;
      case 'f':
        fraction = np::atod(optarg);
        break;

      case 0:
      case 1:
      case 2:
      case '?':
      case ':' :
      case 'h':
        std::cout << "Usage:" << std::endl
         << "  " << argv[0] << " [--help|-h]" << std::endl
         << "  [--data|-d path/to/file/block.features.array]" << std::endl
         << "  [--labels|-l path/to/file/block.labels.array]" << std::endl
         << "  [--labelidx|-i path/to/file/block.labels.array]" << std::endl
         << "  [--type|-t [0-3]]" << std::endl
         << "  [--rtree|-r path/to/file/rtree.xml]" << std::endl
         << "  [--numtrees|-n <no. trees>]"  
         << std::endl << std::endl;
         exit(0);
      default:
        break;
    }
  }

  if (status < num_args)
    np::msg_error(HERE, "wrong usage!");

  return (status >= num_args);
}

