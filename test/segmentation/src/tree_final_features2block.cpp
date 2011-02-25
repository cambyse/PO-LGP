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

static char shortopts[] = "hi:o:";
static struct option longopts[] = {
  { "help",            no_argument,            NULL,              'h' },
  /* working mode */
  { "input",           required_argument,      NULL,              'i' },
  { "output",          required_argument,      NULL,              'o' },
  { NULL,              0,                      NULL,              0   }
};

int args(charA& input, charA& output, int argc, char** argv);

int main(int argc, char** argv)
{
  // results and input data
  stringA list;
  floatA desc, scales;
  intA grid;

  // args
  charA input, output;

  // parse the input arguments
  args(input, output, argc, argv);

  // load input
  np::voc_load_list(list, input.p);

  // write all features into one matrix
  np::final_features2block(desc, list);

  // save results
  np::ucm_final_features_save(desc, output.p);

  return 0;
}

int args(charA& input, charA& output, int argc, char** argv)
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

      case 0:
      case 1:
      case 2:
      case '?':
      case ':' :
      case 'h':
        std::cout << "Usage:" << std::endl
         << "  " << argv[0] << " [--help|-h]" << std::endl
         << "  [--input|-i path/to/file/files.list]" << std::endl
         << "  [--output|-o path/to/file/img.features.array]" 
         << std::endl << std::endl;
         exit(0);
      default:
        break;
    }
  }

  if (status < 2)
    np::msg_error(HERE, "wrong usage!");

  return (status >= 2);
}


