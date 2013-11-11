

/* How I would like to write the code!! */

DECLARE_MODULE(ImageProducer){
  ACCESS(int, count);
  ACCESS(byteA, image);
};

DECLARE_MODULE(ImageConsumer){
  ACCESS(byteA, rgb);
};

void main(int argc, char** argv){
  //direct instantiation
  ImageProducer cam;
  ImageConsumer view;
  connect(cam.image, view.rgb);

  //instantiation via registry
  ModuleRef cam("ImageProducer");
  ModuleRef view("ImageConsumer");
  connect(cam, "image", view, "rgb");

  //instantiation in group
  Group G;
  G <<ImageProducerRef <<ImageConsumerRef;
  G.tie("image", "rgb");

  //instantiation in group described in file
  Group G("mygroup.mof");
  G.run();

  return 0;
};
