#include <MT/array.h>
#include <MT/ors.h>

namespace relational {
void generateOrsBlocksSample(ors::Graph& ors, const uint numOfBlocks);
void generateBlocksSample(MT::Array<arr>& sample, uint numOfBlocks);
void createCylinder(ors::Body& cyl, const ors::Vector& pos, const arr& color);
}
