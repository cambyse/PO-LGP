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

static char shortopts[] = "hd:l:t:r:i:p:";
static struct option longopts[] = {
  { "help",            no_argument,            NULL,              'h' },
  /* working mode */
  { "data",            required_argument,      NULL,              'd' },
  { "labels",          required_argument,      NULL,              'l' },
  { "type",            required_argument,      NULL,              't' },
  { "rtree",           required_argument,      NULL,              'r' },
  { "labelidx",        required_argument,      NULL,              'i' },
  { "pout",            required_argument,      NULL,              'p' },
  { NULL,              0,                      NULL,              0   }
};

int args(charA& data, charA& labels, uint& type, charA& rtrees, uint& label_idx, charA& pout, int argc, char** argv);
float errors(floatA& err, const floatA& x1, const floatA& x2);

int main(int argc, char** argv)
{
  charA labels_file, data_file, rtree_file, pout_file;
  floatA data, labels, labels_type;
  floatA pout, err;
  float e;
  CvRTrees rtrees;
  uint type, label_idx;
  bool use_labels = false;

  // parse command line arguments
  args(data_file, labels_file, type, rtree_file, label_idx, pout_file, argc, argv);
  std::cout << "label_idx = " << label_idx << std::endl;

  // load data
  np::ucm_final_features_load(data, data_file.p);
  rtrees.load(rtree_file.p);
  std::cout << "features: " << data.d0 << " " << data.d1 << " " << data.d2 << std::endl;

  if (labels_file.N > 0 && type < 4)
    use_labels = true;

  // load labels, if that is what the user wants
  if (use_labels)
  {
    np::load_array(labels, labels_file.p, "labels_floatA");
    std::cout << labels.d0 << " " << labels.d1 << " " << labels.d2 << std::endl; // TODO debug
    labels_type.resize(labels.d1);
    for (uint i = 0; i < labels_type.d0; i++)
      labels_type(i) = labels(type, i, label_idx);
    std::cout << labels_type.d0 << " " << labels_type.d1 << " " << labels_type.d2 << std::endl; // TODO debug
  }
  else
    std::cout << "NOT using label information, prediction only" << std::endl;

  // run predction
  np::rtree_predict(pout, &rtrees, data);

  // compute errors
  if (use_labels)
  {
    e = errors(err, pout, labels_type);
    std::cout << "total error: " << e << std::endl;
  }
  else
    err.clear();

  // save results
  np::rtree_predict_save(pout_file.p, pout, err);
  std::cout << "done!" << std::endl;

  return 0;
}

int args(charA& data, charA& labels, uint& type, charA& rtrees, uint& label_idx, charA& pout, int argc, char** argv)
{
  int num_args = 3;
  int status = 0;
  int ch = 1;

  // default values
  type = 5;
  labels.clear();
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
        break;
      case 'p':
        pout.resize(strlen(optarg)+1);
        pout = '\0';
        memcpy(pout.p, optarg, strlen(optarg));
        status++;
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
         << "  [--type|-t [0-3]]" << std::endl
         << "  [--rtree|-r path/to/file/rtree.xml]" << std::endl
         << "  [--pout|-p path/to/prediction.rtree.prediction.array>]"  
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

float errors(floatA& err, const floatA& x1, const floatA& x2)
{
  if ((x1.N != x2.N) && (x1.d0 != x2.d0))
    np::msg_error(HERE, "((x1.N != x2.N) && (x1.d0 != x2.d0))");

  uint rows = x1.d0;
  err.resize(rows);
  for (uint row = 0; row < rows; row++)
    err(row) = (x1(row)-x2(row))*(x1(row)-x2(row));
//    err(row) = sqrDistance(x1[row], x2[row]);

  return sumOfAbs(err);
}
