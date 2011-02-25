#include <iostream>

extern "C" {
  #include <getopt.h>
  #include <sys/time.h>
}

#include <NP/nputils.h>
#include <NP/ucm.h>
#include <NP/cvutils.h>

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
  charA input, output;
  seg::UcmTree tree;
  byteA ucm2;

  args(input, output, argc, argv);                  // parse the input arguments
  np::load_image(ucm2, input.p, 0);                            // load UCM image
  seg::ucm_gen_tree(tree, ucm2);                                // generate tree
  seg::ucm_save(tree, output.p);           // save tree generated from UCM image

  return 0;
}

int args(charA& input, charA& output, int argc, char** argv)
{
  int status = -1;
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
         << "  [--input|-i path/to/ucm2.bmp] [--output|-o path/to/ucm2.tree.array]"
         << std::endl << std::endl;
         exit(0);
      default:
        break;
    }
  }
  
  if (status < 1)
    np::msg_error(HERE, "wrong usage!");

  return (status >= 1);
};

