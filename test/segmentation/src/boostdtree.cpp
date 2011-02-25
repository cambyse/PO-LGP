#include <algorithm>
#include <iostream>
#include <ctime>
#include <cstdio>
#include <limits>

extern "C" {
  #include <getopt.h>
}

#include <MT/array.h>
#include <MT/util.h>
#include <nputils.h>
#include <desc.h>
#include <vocutils.h>
#include <rtree.h>
#include <cvutils.h>
#include <ucm.h>

#include <opencv/ml.h>

static char shortopts[] = "hd:l:o:n:m:c:t:p";
static struct option longopts[] = {
  { "help",            no_argument,            NULL,              'h' },
  /* working mode */
  { "data",            required_argument,      NULL,              'd' },
  { "labels",          required_argument,      NULL,              'l' },
  { "output",          required_argument,      NULL,              'o' },
  { "numtrees",        required_argument,      NULL,              'n' },
  { "maxdepth",        required_argument,      NULL,              'm' },
  { "column",          required_argument,      NULL,              'c' },
  { "type",            required_argument,      NULL,              't' },
  { "predict",         no_argument,            NULL,              'p' },
  { NULL,              0,                      NULL,              0   }
};

int args(charA& datafile, charA& labelsfile, charA& bdtoutfile, uint& num_trees,
         uint& max_depth, uint& clss, uint& type, bool& p, int argc, char** argv);

float predict(CvBoost* boost, uint type);
// float errors(floatA& err, const floatA& x1, const floatA& x2);

int main(int argc, char** argv)
{
  charA datafile, labelsfile, bdtoutfile;
  uint num_trees=10, max_depth=3, clss=0, type=2;
  bool p = false;
  CvBoost* boost = NULL;
  const uint num_classes        = 2;
  const int boost_type          = CvBoost::REAL;         // real valued AdaBoost
  const float priors[]          = {1.0, 10.0};
  const bool use_surrogates     = false;
  const double weight_trim_rate = 0.95;


  // initialize random number generator
  MT::rnd.clockSeed();

  // parse command line arguments
  args(datafile, labelsfile, bdtoutfile, num_trees, max_depth,
       clss, type, p, argc, argv);

  // disply parameters summary
  std::cout << "data file   = " << datafile.p << std::endl;
  std::cout << "labels file = " << labelsfile.p << std::endl;
  std::cout << "output      = " << bdtoutfile.p << std::endl;
  std::cout << "num_trees   = " << num_trees << std::endl;
  std::cout << "max_depth   = " << max_depth << std::endl;
  std::cout << "labels col  = " << clss << " (" << np::voc_classes[clss]
                                << ")" << std::endl;
  std::cout << "type        = " << type << " (" << np::label_types[type]
                                << ")" << std::endl;

  // load training data and labels
  uintA permutation;
  floatA labels, features, labels_temp, predictions, sums;
  if (np::labels_load(labels, labelsfile.p) != 0)
    np::msg_error(HERE, "Cannot load labels file");
  if (np::ucm_final_features_load(features, datafile.p) != 0)
    np::msg_error(HERE, "Cannot load data file");

  labels_temp.resize(labels.d1);
  for (uint i = 0; i < labels_temp.d0; i++)
    labels_temp(i) = (labels(type,i,1) >= 0.5);

  // permutate rows of data (+labels)
  np::permute_rows(features, permutation);
  np::permute_rows(labels_temp, permutation);

  // train decision trees
  boost = np::boostDTrees_train(features, labels_temp, boost_type, num_classes,
                num_trees, weight_trim_rate, max_depth, use_surrogates, priors);

  float prate = 0;
  if (p)
    prate = predict(boost, type);

  std::ostringstream outfile;
  outfile << std::setw(3) << std::setfill('0') << prate << "_" << bdtoutfile.p;

  // save classifier
  if (bdtoutfile.N != 0 && boost != NULL)
    boost->save(outfile.str().c_str());

//  // save classifier
//  if (bdtoutfile.N != 0 && boost != NULL)
//    boost->save(bdtoutfile.p);

  if (boost != NULL)
    delete boost;
  return 0;
}

float predict(CvBoost *boost, uint type)
{
  // testing
  std::cout << "Testing Boosted Decision Trees ... " << std::endl;
  char test_data_file[] =
       "voc2009/features/final/samples.aeroplane.features.array";
  char test_labels_file[] =
       "voc2009/features/labels/samples.aeroplane.labels.array";
  floatA tlabels, tfeatures, tlabels_temp, predictions, sums;

   // disply parameters summary
  std::cout << "test file   = " << test_data_file << std::endl;
  std::cout << "test labels = " << test_labels_file << std::endl;
  std::cout << "type        = " << type << " (" << np::label_types[type]
                                << ")" << std::endl;

  // load data
  np::labels_load(tlabels, test_labels_file);
  np::ucm_final_features_load(tfeatures, test_data_file);

  tlabels_temp.resize(tlabels.d1);
  for (uint i = 0; i < tlabels_temp.d0; i++)
    tlabels_temp(i) = (tlabels(type,i,1) >= 0.5);

  // run prediction
  np::boostDTrees_predict(predictions, sums, boost, tfeatures, 2);

  // evaluate
  float tp=0, fp=0, tn=0, fn=0, awk=0;
  for (uint i = 0; i < tlabels_temp.d0; i++)
  {
    if ((tlabels_temp(i) == 1) && (predictions(i) == 1))
      tp++;
    else if ((tlabels_temp(i) == 0) && (predictions(i) == 0))
      tn++;
    else if ((tlabels_temp(i) == 1) && (predictions(i) == 0))
      fn++;
    else if ((tlabels_temp(i) == 0) && (predictions(i) == 1))
      fp++;
    else
    {
      awk++;
      std::cout << "tlabels_temp("<<i<<") = " << tlabels_temp(i)
      << "predictions(i) = " << predictions(i) << std::endl;
    }
  }

  // print results
  std::cout << "tlabels.d0  = " << tlabels_temp.d0 << ", (tp+tn+fn+fp) = " << (tp+tn+fn+fp) << std::endl;
  std::cout << "awk         = " << awk << std::endl;
  std::cout << "tp = " << tp << ", tn = " << tn << std::endl;
  std::cout << "fp = " << fp << ", fn = " << fn << std::endl;
  std::cout << "(tp+tn)/(tp+tn+fn+fp) = " << (tp+tn)/(tp+tn+fn+fp) << std::endl;
  std::cout << "(fp+fn)/(tp+tn+fn+fp) = " << (fp+fn)/(tp+tn+fn+fp) << std::endl;
  std::cout << "(tp)/(tp+fp)          = " << (tp)/(tp+fp) << std::endl;
  std::cout << "(tn)/(tn+fn)          = " << (tn)/(tn+fn) << std::endl;
  
  return round((tp+tn)/(tp+tn+fn+fp)*100);
};

//int test02()
//{
//  // parse command line arguments
////   args(potfile, ucmtfile, gamma, outfile, argc, argv);
////   MT::rnd.clockSeed();

//  int boost_type          = CvBoost::REAL;
//  uint num_classes        = 2;
//  int weak_count          = 10;
//  double weight_trim_rate = 0.95;
//  int max_depth           = 3;
//  bool use_surrogates     = false;
//  const float priors[]    = {1.0, 10.0};

//  uintA permutation;
//  floatA labels, features, labels_temp, predictions, sums;
//  np::labels_load(labels, "voc2009/features/labels/testsamples.aeroplane.labels.array");
//  np::ucm_final_features_load(features, "voc2009/features/final/testsamples.aeroplane.features.array");

//  labels_temp.resize(labels.d1);
//  for (uint i = 0; i < labels_temp.d0; i++)
//    labels_temp(i) = (labels(2,i,1) >= 0.5);

//  uintA zeros, ones;
//  for (uint i = 0; i < labels_temp.d0; i++)
//    if (labels_temp(i) == 1)
//      ones.append(i);
//    else
//      zeros.append(i);
//  zeros.permuteRandomly();

//  np::permute_rows(features, permutation);
//  np::permute_rows(labels_temp, permutation);

////   CvBoost *boost = new CvBoost;
////   boost->load("zzz.boost.xml");
//  CvBoost *boost = np::boostDTrees_train
//      (features, labels_temp, boost_type, num_classes, weak_count,
//       weight_trim_rate, max_depth, use_surrogates, priors);
//  boost->save("zzz.boost.xml");

//  // testing
//  std::cout << "Testing Boosted Decision Trees ... " << std::endl;
//  floatA tlabels, tfeatures, tlabels_temp;
//  np::labels_load(tlabels, "voc2009/features/labels/samples.aeroplane.labels.array");
//  np::ucm_final_features_load(tfeatures, "voc2009/features/final/samples.aeroplane.features.array");

//  tlabels_temp.resize(tlabels.d1);
//  for (uint i = 0; i < tlabels_temp.d0; i++)
//    tlabels_temp(i) = (tlabels(2,i,1) >= 0.5);

//  np::boostDTrees_predict(predictions, sums, boost, tfeatures, 2);

//  float tp=0, fp=0, tn=0, fn=0, awk=0;
//  for (uint i = 0; i < tlabels_temp.d0; i++)
//  {
//    if ((tlabels_temp(i) == 1) && (predictions(i) == 1))
//      tp++;
//    else if ((tlabels_temp(i) == 0) && (predictions(i) == 0))
//      tn++;
//    else if ((tlabels_temp(i) == 1) && (predictions(i) == 0))
//      fn++;
//    else if ((tlabels_temp(i) == 0) && (predictions(i) == 1))
//      fp++;
//    else
//    {
//      awk++;
//      std::cout << "tlabels_temp("<<i<<") = " << tlabels_temp(i)
//      << "predictions(i) = " << predictions(i) << std::endl;
//    }
//  }


//  std::cout << "tlabels_temp.d0 = " << tlabels_temp.d0 << ", (tp+tn+fn+fp) = " << (tp+tn+fn+fp) << std::endl;
//  std::cout << "awk = " << awk << std::endl;
//  std::cout << "tp = " << tp << ", tn = " << tn << std::endl;
//  std::cout << "fp = " << fp << ", fn = " << fn << std::endl;
//  std::cout << "(tp+tn)/(tp+tn+fn+fp) = " << (tp+tn)/(tp+tn+fn+fp) << std::endl;
//  return 0;
//}

int args(charA& datafile, charA& labelsfile, charA& bdtoutfile, uint& num_trees, uint& max_depth, uint& clss, uint& type, bool& p, int argc, char** argv)
{
  int num_args = 3;
  int status = 0;
  int ch = 1;

  // default values
  while (ch != -1)
  {
    ch = getopt_long(argc, argv, shortopts, longopts, NULL);
    switch(ch)
    {
      case 'd':
        datafile.resize(strlen(optarg)+1);
        datafile = '\0';
        memcpy(datafile.p, optarg, strlen(optarg));
        status++;
        break;
      case 'l':
        labelsfile.resize(strlen(optarg)+1);
        labelsfile = '\0';
        memcpy(labelsfile.p, optarg, strlen(optarg));
        status++;
        break;
      case 'o':
        bdtoutfile.resize(strlen(optarg)+1);
        bdtoutfile= '\0';
        memcpy(bdtoutfile.p, optarg, strlen(optarg));
        status++;
        break;
      case 'n':
        num_trees = atoi(optarg);
        break;
      case 'm':
        max_depth = atoi(optarg);
        break;
      case 'c':
        clss = atoi(optarg);
        break;
      case 't':
        type = atoi(optarg);
        break;
      case 'p':
        p = true;
        break;

      case 0:
      case 1:
      case 2:
      case '?':
      case ':' :
      case 'h':
        std::cout << "Usage:" << std::endl
         << "  " << argv[0] << " [--help|-h]" << std::endl
         << "  [--datafile|-d path/to/file/features.array]" << std::endl
         << "  [--labelsfile|-l path/to/file/labels.array]" << std::endl
         << "  [--output|-o path/to/bdt.xml]" << std::endl
         << "  [--numtrees|-n <N>]" << std::endl
         << "  [--maxdepth|-m <M>]" << std::endl
         << "  [--class|-c <0...21>]" << std::endl
         << "  [--type|-t <0...3>]"
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


//void test()
//{
//  MT::rnd.clockSeed();
//  uint N = 100000, D = 1500, K = 10000;
//  floatA data, responses, test_data, test_responses;
//  floatA temp_data, temp_responses, sums;
//  uintA permutation;

//  int boost_type          = CvBoost::REAL;
//  uint num_classes        = 2;
//  int weak_count          = 100;
//  double weight_trim_rate = 0.95;
//  int max_depth           = 5;
//  bool use_surrogates     = false;
//  const float* priors     = 0;

//  // create data
//  data.reshape(0,D);
//  responses.reshape(0);
//  test_data.reshape(0,D);
//  test_responses.reshape(0);

//  temp_data.resize(N/2,D); temp_responses.resize(N/2);
//  temp_data=-1; rndGauss(temp_data,1,true); data.append(temp_data);
//  temp_data=+1; rndGauss(temp_data,1,true); data.append(temp_data);
//  temp_responses = 0; responses.append(temp_responses);
//  temp_responses = +1; responses.append(temp_responses);

//  temp_data.resize(K/2,D); temp_responses.resize(K/2);
//  temp_data=-1; rndGauss(temp_data,1,true); test_data.append(temp_data);
//  temp_data=+1; rndGauss(temp_data,1,true); test_data.append(temp_data);
//  temp_responses = 0; test_responses.append(temp_responses);
//  temp_responses = +1; test_responses.append(temp_responses);

//  np::permute_rows(data, permutation);
//  np::permute_rows(responses, permutation);
////   std::cout << data << std::endl;
////   std::cout << responses << std::endl;

//  CvBoost *boost = np::boostDTrees_train(data, responses, boost_type, num_classes, weak_count, weight_trim_rate, max_depth, use_surrogates, priors);
//  np::boostDTrees_predict(responses, sums, boost, data, 2);
//  np::boostDTrees_predict(responses, sums, boost, test_data, 2);

//  std::cout << "true" << std::endl << test_responses << std::endl;
//  std::cout << "actual" << std::endl << responses << std::endl;

//}
