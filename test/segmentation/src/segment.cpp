#include <iostream>
#include <ctime>
#include <cstdio>
#include <limits>

extern "C" {
  #include <getopt.h>
}

#include <MT/array.h>
#include <../src/nputils.h>
#include <../src/desc.h>
#include <../src/vocutils.h>
#include <../src/rtree.h>
#include <../src/cvutils.h>
#include <../src/ucm.h>

#include <opencv/ml.h>

static char shortopts[] = "hp:u:g:o:";
static struct option longopts[] = {
  { "help",            no_argument,            NULL,              'h' },
  /* working mode */
  { "potentials",      required_argument,      NULL,              'p' },
  { "ucmtree",         required_argument,      NULL,              'u' },
  { "gamma",           required_argument,      NULL,              'g' },
  { "output",          required_argument,      NULL,              'o' },
  { NULL,              0,                      NULL,              0   }
};

int args(charA& potfile, charA& ucmtfile, float& gamma, charA& outfile, int argc, char** argv);
float errors(floatA& err, const floatA& x1, const floatA& x2);

int main(int argc, char** argv)
{
  charA potfile, ucmtfile, rtreefile, outfile;
  floatA potentials, errors, posteriors;
  floatA out;
  float gamma;
  seg::UcmTree tree;

  // parse command line arguments
  args(potfile, ucmtfile, gamma, outfile, argc, argv);

  // load data
  np::rtree_predict_load(potfile.p, potentials, errors);
  seg::ucm_load(tree, ucmtfile.p);
  
  // do inference
  seg::ucm_infer(posteriors, tree, potentials, gamma);

  std::cout << "posteriors = " << std::endl << posteriors << std::endl;

  return 0;
}

int args(charA& potfile, charA& ucmtfile, float& gamma, charA& outfile, int argc, char** argv)
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
      case 'p':
        potfile.resize(strlen(optarg)+1);
        potfile = '\0';
        memcpy(potfile.p, optarg, strlen(optarg));
        status++;
        break;
      case 'u':
        ucmtfile.resize(strlen(optarg)+1);
        ucmtfile = '\0';
        memcpy(ucmtfile.p, optarg, strlen(optarg));
        status++;
        break;
      case 'g':
        gamma = np::atod(optarg);
        status++;
        break;
      case 'o':
        outfile.resize(strlen(optarg)+1);
        outfile = '\0';
        memcpy(outfile.p, optarg, strlen(optarg));
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
         << "  [--potentials|-p path/to/file/potentials.array]" << std::endl
         << "  [--ucmtree|-u path/to/file/ucm.tree.array]" << std::endl
         << "  [--gamma|-g [0-1]]" << std::endl
         << "  [--rtrees|-r path/to/file/rtree.xml]" << std::endl
         << "  [--output|-o path/to/segmentation.array>]"  
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
