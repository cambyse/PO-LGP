
#include <iostream>

extern "C" {
  #include <getopt.h>
  #include <sys/time.h>
}

#include <NP/nputils.h>
#include <NP/ucm.h>
#include <NP/cvutils.h>
#include <NP/desc.h>

static char shortopts[] = "hi:o:c:";
static struct option longopts[] = {
  { "help",            no_argument,            NULL,              'h' },
  /* working mode */
  { "input",           required_argument,      NULL,              'i' },
  { "output",          required_argument,      NULL,              'o' },
  { "coef",            required_argument,      NULL,              'c' },
  { NULL,              0,                      NULL,              0   }
};

int args(charA& input, charA& output, int& num_coef, int argc, char** argv);

int main(int argc, char** argv)
{
  charA input, output;
  int num_coef;
  seg::UcmTree tree;
  floatA efd, centers, sizes;

  args(input, output, num_coef, argc, argv);        // parse the input arguments
  seg::ucm_load(tree, input.p);                                 // load UCM tree
  np::ucm_shapedesc(efd, centers, sizes, tree, num_coef);        // computer EFD
  np::save_ucm_shapedesc(efd, centers, sizes, output.p);         // save results

  return 0;
}


int args(charA& input, charA& output, int& num_coef, int argc, char** argv)
{
  int status = 0;
  int ch = 1;

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
        num_coef = atoi(optarg);
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
         << "  [--input|-i path/to/img.tree.bmp] [--output|-o path/to/img.shapedesc.array]"
         << std::endl << "  [--coef|-c co]"
         << std::endl << std::endl;
         exit(0);
      default:
        break;
    }
  }
  
  if (status != 3)
    np::msg_error(HERE, "wrong usage!");

  return (status == 3);
};

