#include <MT/array.h>
#include <MT/ors.h>

namespace relational {
void generateOrsBlocksSample(ors::Graph& ors, const uint numOfBlocks);
void generateOrsFromSample(ors::Graph& ors, const MT::Array<arr>& sample);
void generateBlocksSample(MT::Array<arr>& sample, uint numOfBlocks);
void createCylinder(ors::Body& cyl, const ors::Vector& pos, const arr& color, const arr& size);
void createCylinder(ors::Body& cyl, const ors::Vector& pos, const arr& color);
}
