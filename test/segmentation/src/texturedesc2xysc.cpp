#include <iostream>
#include <ctime>
#include <cstdio>
#include <limits>

extern "C" {
  #include <getopt.h>
}

#include <NP/nputils.h>
#include <NP/desc.h>

static char shortopts[] = "hi:o:c:";
static struct option longopts[] = {
  { "help",            no_argument,            NULL,              'h' },
  /* working mode */
  { "input",           required_argument,      NULL,              'i' },
  { "output",          required_argument,      NULL,              'o' },
  { "codebook",        required_argument,      NULL,              'c' },
  { NULL,              0,                      NULL,              0   }
};

int args
(
 charA& input,
 charA& output,
 charA& codebook,
 int argc,
 char** argv
);

int main(int argc, char** argv)
{
  // results and input data
  floatA desc, scales, codebook, xysc;
  intA grid;

  // args
  charA input, output, codebook_filename;

  // parse the input arguments
  args(input, output, codebook_filename, argc, argv);

  // load input
  np::codebook_load(codebook, codebook_filename.p);             // load codebook
  np::img_texturedesc_load(desc, grid, scales, input.p);     // load descriptors

  // quantization step
  np::quantization(xysc, codebook, desc, grid, scales);

  // save results
  np::quantization_save(xysc, codebook.d0, output.p);

  return 0;
}

int args
(
 charA& input,
 charA& output,
 charA& codebook,
 int argc,
 char** argv
)
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
        codebook.resize(strlen(optarg)+1);
        codebook = '\0';
        memcpy(codebook.p, optarg, strlen(optarg));
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
         << "  [--input|-i path/to/file/img.surf.array]" << std::endl
         << "  [--output|-o path/to/file/img.xysc.array]" << std::endl
         << "  [--codebook|-c path/to/file/good.codebook.array]"
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


