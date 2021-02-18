#ifndef TRANSITION_COST_PARTITIONING_OPERATOR_MASK_GENERATOR_H
#define TRANSITION_COST_PARTITIONING_OPERATOR_MASK_GENERATOR_H

#include "types.h"

using namespace std;

namespace options {
class Options;
}

namespace transition_cost_partitioning {
class Abstraction;

/**
 * The MaskGeneratorApplicability generates an operator mask
 */
class OperatorMaskGenerator {
  public:
    OperatorMaskGenerator(const options::Options &opts);
    virtual ~OperatorMaskGenerator() = default;

    /* Generate a mask for the set of abstractions.
    */
    virtual vector<bool> generate_mask(
      const Abstraction &abstraction) const = 0;
};

}

#endif
