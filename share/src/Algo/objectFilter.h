



struct FilterObject{
  double sensorTime;
  uint id;
  virtual bool idIsConfident();
  virtual double idMatchingCost(const FilterObject& o);
  virtual void mergeWithInputObject(const FilterObject& o);
};

void filterStep(const FilterObjectL& perceptualInputs, FilterObjectL& objectDataBase);

