#ifndef TRANSITION_COST_PARTITIONING_OPERATOR_MASK_GENERATOR_NONE_H
#define TRANSITION_COST_PARTITIONING_OPERATOR_MASK_GENERATOR_NONE_H

#include "operator_mask_generator.h"

#include <memory>

using namespace std;

namespace transition_cost_partitioning {
class Abstraction;


class OperatorMaskGeneratorNone : public OperatorMaskGenerator {
  public:
    OperatorMaskGeneratorNone(const options::Options &opts);
    virtual ~OperatorMaskGeneratorNone() = default;

    virtual vector<bool> generate_mask(
      const Abstraction &abstraction) const override;
};

}

#endif
