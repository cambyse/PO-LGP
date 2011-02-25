#include <iostream>
#include <ctime>
#include <cstdio>
#include <limits>

extern "C" {
  #include <getopt.h>
}

#include <NP/nputils.h>
#include <NP/ucm.h>
#include <NP/cvutils.h>
#include <NP/desc.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

static char shortopts[] = "hi:o:c:t:e:la:";
static struct option longopts[] = {
  { "help",            no_argument,            NULL,              'h' },
  /* working mode */
  { "input",           required_argument,      NULL,              'i' },
  { "output",          required_argument,      NULL,              'o' },
  { "num_clusters",    required_argument,      NULL,              'c' },
  { "tcd",             required_argument,      NULL,              't' },
  { "tci",             required_argument,      NULL,              'e' },
  { "labels",          no_argument,            NULL,              'l' },
  { "attempts",        required_argument,      NULL,              'a' },
  { NULL,              0,                      NULL,              0   }
};

int args
(
 charA& input,
 charA& output,
 uint& num_clusters,
 double& tc_d,
 uint& tc_i,
 bool& use_labels,
 int& attempts,
 int argc,
 char** argv
);

int main(int argc, char** argv)
{

  // results and input data
  floatA clusters, samples;
  intA labels;
  double compactness;

  // args
  charA input, output, filename;
  uint num_clusters;
  double termcrit_d;
  uint termcrit_i;
  bool use_labels;
  int attempts;

  // parse the input arguments
  args(input, output, num_clusters, termcrit_d, termcrit_i, use_labels, attempts, argc, argv);

  // load all SURF descriptors --> assuming all features have been lumped together
  np::load_array(samples, input.p, "texturedesc_floatA");
  for (uint i = 0; i < samples.N; i++)
    if(isnan(samples.p[i])) // TODO debug
      std::cout << "isnan(samples.p[i])" << std::endl;

  // start KMeans
  np::kmeans(clusters, labels, compactness, samples, num_clusters, termcrit_d, termcrit_i, attempts, use_labels);
  std::cout << "compactness = " << compactness << std::endl;
  for (uint i = 0; i < clusters.N; i++)
    if(isnan(clusters.p[i])) // TODO debug
      std::cout << "isnan(clusters.p[i])" << std::endl;

  // save results
  np::codebook_name(filename, num_clusters, compactness, output.p);
  np::codebook_save(clusters, filename.p);

  return 0;
}

int args
(
 charA& input,
 charA& output,
 uint& num_clusters,
 double& tc_d,
 uint& tc_i,
 bool& use_labels,
 int& attempts,
 int argc,
 char** argv
)
{
  int status = 0;
  int ch = 1;

  tc_d = 1.0;
  tc_i = 10;
  use_labels = false;
  attempts = 1;

  while (ch != -1) 
  {
    ch = getopt_long(argc, argv, shortopts, longopts, NULL);
    switch(ch)
    {
      case 'i':
        input.resize(strlen(optarg)+1);
        input = '\0';
        memcpy(input.p, optarg, strlen(optarg));
        status++;
        break;
      case 'o':
        output.resize(strlen(optarg)+1);
        output = '\0';
        memcpy(output.p, optarg, strlen(optarg));
        status++;
        break;
      case 'c':
        num_clusters = atoi(optarg);
        status++;
        break;
      case 't':
        tc_d = np::atod(optarg);
        break;
      case 'e':
        tc_i = atoi(optarg);
        break;
      case 'l':
        use_labels = true;
        break;
      case 'a':
        attempts = atoi(optarg);
        break;

      case 0:
      case 1:
      case 2:
      case '?':
      case ':' :
      case 'h':
        std::cout << "Usage:" << std::endl
         << "  " << argv[0] << " [--help|-h]" << std::endl
         << "  [--input|-i path/to/file/with/all/features.array]" << std::endl
         << "  [--output|-o path/to/codebooks/]" << std::endl
         << "  [--num_cluster|-c num]" << std::endl
         << "  [--tcd|-t thresh] [--tci|-e iter]" << std::endl
         << "  [--labels|-l] [--attempts|-a att]"
         << std::endl << std::endl;
         exit(0);
      default:
        break;
    }
  }

  if (status < 3)
    np::msg_error(HERE, "wrong usage!");

  return (status >= 3);
}


