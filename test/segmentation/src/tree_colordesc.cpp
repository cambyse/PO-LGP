
#include <iostream>

extern "C" {
  #include <getopt.h>
  #include <sys/time.h>
}

#include <NP/nputils.h>
#include <NP/ucm.h>
#include <NP/cvutils.h>
#include <NP/desc.h>

static char shortopts[] = "Hi:o:t:h:s:v:";
static struct option longopts[] = {
  { "help",            no_argument,            NULL,              'H' },
  /* working mode */
  { "image",           required_argument,      NULL,              'i' },
  { "output",          required_argument,      NULL,              'o' },
  { "tree",            required_argument,      NULL,              't' },
  { "hue",             required_argument,      NULL,              'h' },
  { "sat",             required_argument,      NULL,              's' },
  { "val",             required_argument,      NULL,              'v' },
  { NULL,              0,                      NULL,              0   }
};

int args
(
 charA& input,
 charA& output,
 charA& tree,
 uint& num_bH,
 uint& num_bS,
 uint& num_bV,
 int argc, 
 char** argv
);

int main(int argc, char** argv)
{
  charA input, output, tree_fn;
  floatA desc_HS, desc_V;
  seg::UcmTree tree;
  byteA image;
  uint num_bH, num_bS, num_bV;

  args(input, output, tree_fn, num_bH, num_bS, num_bV, argc, argv);
  np::load_image(image, input.p);
  seg::ucm_load(tree, tree_fn.p);
  np::ucm_colordesc_hsv(desc_HS, desc_V, tree, image, num_bH, num_bS, num_bV);
  np::ucm_colordesc_hsv_save(desc_HS, num_bH, num_bS, desc_V, output.p);

  return 0;
}


int args
(
 charA& input,
 charA& output,
 charA& tree,
 uint& num_bH,
 uint& num_bS,
 uint& num_bV,
 int argc, 
 char** argv
)
{
  int num_arg = 6;
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
      case 't':
        tree.resize(strlen(optarg)+1);
        tree = '\0';
        memcpy(tree.p, optarg, strlen(optarg));
        status++;
        break;
      case 'h':
        num_bH = (uint) atoi(optarg);
        status++;
      case 's':
        num_bS = (uint) atoi(optarg);
        status++;
      case 'v':
        num_bV = (uint) atoi(optarg);
        status++;
        break;

      case 0:
      case 1:
      case 2:
      case '?':
      case ':' :
      case 'H':
        std::cout << "Usage:" << std::endl
         << "  " << argv[0] << " [--help|-H]" << std::endl
         << "  [--image|-i path/to/img.jpg] [--output|-o path/to/img.colordesc.array]"
         << std::endl << "  [--tree|-t path/to/img.tree.array]" << std::endl
         << "  [--hue|-h bH] [--sat|-s bS] [--val|-v bV] "
         << std::endl << std::endl;
         exit(0);
      default:
        break;
    }
  }
  
  if (status < num_arg)
    np::msg_error(HERE, "wrong usage!");

  return (status == num_arg);
};

