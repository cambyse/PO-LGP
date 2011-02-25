
#include <iostream>

extern "C" {
  #include <getopt.h>
  #include <sys/time.h>
}

#include <NP/nputils.h>
#include <NP/ucm.h>
#include <NP/cvutils.h>
#include <NP/desc.h>

static char shortopts[] = "ht:c:s:o:u:";
static struct option longopts[] = {
  { "help",            no_argument,            NULL,              'h' },
  /* working mode */
  { "texture",         required_argument,      NULL,              't' },
  { "color",           required_argument,      NULL,              'c' },
  { "shape",           required_argument,      NULL,              's' },
  { "tree",            required_argument,      NULL,              'u' },
  { "output",          required_argument,      NULL,              'o' },
  { NULL,              0,                      NULL,              0   }
};

int args(charA& texture, charA& color, charA& shape, charA& tree, charA& output, int argc, char** argv);

int main(int argc, char** argv)
{
  charA texture_fn, color_fn, shape_fn, tree_fn, output;
  floatA texture, color_HS, color_V, shape, centers, sizes, ff;
  seg::UcmTree tree;
  uint num_bH, num_bS;

  args(texture_fn, color_fn, shape_fn, tree_fn, output, argc, argv);
  seg::ucm_load(tree, tree_fn.p);
  np::ucm_texturehist_load(texture, texture_fn.p);
  np::ucm_colordesc_hsv_load(color_HS, num_bH, num_bS, color_V, color_fn.p);
  np::load_ucm_shapedesc(shape, centers, sizes, shape_fn.p);

  for (uint i = 0; i < texture.N; i++)
    if (isnan(texture.p[i]))
    {std::cout << "texture" << std::endl; exit(0);}

  for (uint i = 0; i < color_HS.N; i++)
    if (isnan(color_HS.p[i]))
    {std::cout << "color_HS" << std::endl; exit(0);}

  for (uint i = 0; i < color_V.N; i++)
    if (isnan(color_V.p[i]))
    {std::cout << "color_V" << std::endl; exit(0);}

  for (uint i = 0; i < shape.N; i++)
    if (isnan(shape.p[i]))
    {std::cout << "shape" << std::endl; exit(0);}

  np::ucm_final_features(ff, tree, texture, color_HS, color_V, shape, centers, sizes);
  np::ucm_final_features_save(ff, output.p);

  return 0;
}


int args(charA& texture, charA& color, charA& shape, charA& tree, charA& output, int argc, char** argv)
{
  int num_args = 5;
  int status = 0;
  int ch = 1;

  while (ch != -1) 
  {
    ch = getopt_long(argc, argv, shortopts, longopts, NULL);
    switch(ch)
    {
      case 't':
        texture.resize(strlen(optarg)+1);
        texture = '\0';
        memcpy(texture.p, optarg, strlen(optarg));
        status++;
        break;
      case 'c':
        color.resize(strlen(optarg)+1);
        color = '\0';
        memcpy(color.p, optarg, strlen(optarg));
        status++;
        break;
      case 's':
        shape.resize(strlen(optarg)+1);
        shape = '\0';
        memcpy(shape.p, optarg, strlen(optarg));
        status++;
        break;
      case 'u':
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
         << "  [--texture|-t path/to/img.texturedesc.array] [--color|-c path/to/img.colordesc.array]" << std::endl
         << "  [--shape|-s path/to/img.shapedesc.array] [--tree|-u path/to/img.tree.array]" << std::endl
         << "  [--output|-o path/to/img.features.array]"
         << std::endl << std::endl;
         exit(0);
      default:
        break;
    }
  }
  
  if (status < num_args)
    np::msg_error(HERE, "wrong usage!");

  return (status == num_args);
};
