#include <MT/array.h>

extern "C" {
  #include <getopt.h>
  #include <sys/time.h>
}

#include <NP/nputils.h>
#include <NP/cvutils.h>
#include <NP/imgproc.h>
#include <NP/desc.h>

static char shortopts[] = "hi:o:w:nc";
static struct option longopts[] = {
  { "help",            no_argument,            NULL,              'h' },
  /* working mode */
  { "input",           required_argument,      NULL,              'i' },
  { "output",          required_argument,      NULL,              'o' },
  { "widths",          required_argument,      NULL,              'w' },
  { "notupright",      no_argument,            NULL,              'n' },
  { "color",           no_argument,            NULL,              'c' },
  { NULL,              0,                      NULL,              0   }
};

int args(charA& input, charA& output, uintA& widths, bool& upright, bool& color, int argc, char** argv);

int main(int argc, char** argv)
{
  charA input, output;
  byteA image;
  floatA desc, desc_t;
  intA grid;
  floatA scales;
  bool upright, color;
  uintA widths;

  // parse input arguments
  args(input, output, widths, upright, color, argc, argv);

  // load image
  if (color)
  {
    np::load_image(image, input.p, -1);                        // load UCM image
    desc.reshape(0, 192);
  }
  else
  {
    np::load_image(image, input.p, 0);                         // load UCM image
    desc.reshape(0, 64);
  }

  // compute texture descriptors
  np::img_texturedesc(desc, grid, scales, image, widths, upright);
  // NOTE to make sure that the data is not NAN
  for (uint i = 0; i < desc.N; i++)
    if (isnan(desc.p[i]))
    {
      std::cout << input.p << " has nans!!! :-(" << std::endl;
      break;
    }

  // save texture descriptors
  np::img_texturedesc_save(desc, grid, scales, output.p);

  return 0;
}

int args(charA& input, charA& output, uintA& widths, bool& upright, bool& color, int argc, char** argv)
{
  upright = true;
  color = false;
  int status = 0;
  int ch = 1;
  stringA tokens;

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
      case 'w':
        np::tokenize(tokens, std::string(optarg), ',');
        widths.resize(tokens.N);
        for (uint i = 0; i < tokens.N; i++)
          widths(i) = atoi(tokens(i).c_str());
        status++;
        break;
      case 'n':
        upright = false;
        break;
      case 'c':
        color = true;
        break;

      case 0:
      case 1:
      case 2:
      case '?':
      case ':' :
      case 'h':
        std::cout << "Usage:" << std::endl
         << "  " << argv[0] << " [--help|-h]" << std::endl
         << "  [--input|-i path/to/image] [--output|-o path/to/image.surf.array]" << std::endl
         << "  [--widths|-w w1,w2,w3,...]" << std::endl
         << "  [--notupright|-n] [--color|-c]"
         << std::endl << std::endl;
         exit(0);
      default:
        break;
    }
  }
  
  if (status < 3)
    np::msg_error(HERE, "wrong usage!");

  return (status == 3);
}

