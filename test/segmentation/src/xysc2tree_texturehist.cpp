#include <iostream>
#include <ctime>
#include <cstdio>
#include <limits>

extern "C" {
  #include <getopt.h>
}

#include <NP/nputils.h>
#include <NP/ucm.h>
#include <NP/desc.h>

static char shortopts[] = "hx:t:o:";
static struct option longopts[] = {
  { "help",            no_argument,            NULL,              'h' },
  /* working mode */
  { "xysc",            required_argument,      NULL,              'x' },
  { "output",          required_argument,      NULL,              'o' },
  { "tree",            required_argument,      NULL,              't' },
  { NULL,              0,                      NULL,              0   }
};

int args
(
 charA& xysc,
 charA& tree,
 charA& output,
 int argc,
 char** argv
);

int main(int argc, char** argv)
{
  // results and input data
  uint num_codewords;
  floatA hist, xysc;
  seg::UcmTree tree;

  // args
  charA xysc_fn, tree_fn, output;

  // parse the input arguments
  args(xysc_fn, tree_fn, output, argc, argv);

  // load input
  np::quantization_load(xysc, num_codewords, xysc_fn.p);
  seg::ucm_load(tree, tree_fn.p);

  // generate histograms step
  np::ucm_texturehist(hist, xysc, num_codewords, tree);

  // save results
  np::ucm_texturehist_save(hist, output.p);

  return 0;
}

int args
(
 charA& xysc,
 charA& tree,
 charA& output,
 int argc,
 char** argv
)
{
  int num_args = 3;
  int status = 0;
  int ch = 1;

  while (ch != -1) 
  {
    ch = getopt_long(argc, argv, shortopts, longopts, NULL);
    switch(ch)
    {
      case 'x':
        xysc.resize(strlen(optarg)+1);
        xysc = '\0';
        memcpy(xysc.p, optarg, strlen(optarg));
        status++;
        break;
      case 't':
        tree.resize(strlen(optarg)+1);
        tree = '\0';
        memcpy(tree.p, optarg, strlen(optarg));
        status++;
        break;
      case 'o':
        output.resize(strlen(optarg)+1);
        output = '\0';
        memcpy(output.p, optarg, strlen(optarg));
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
         << "  [--xysc|-x path/to/file/img.xysc.array]" << std::endl
         << "  [--tree|-t path/to/file/img.tree.array]" << std::endl
         << "  [--output|-o path/to/file/img.texturehist.array]"
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


