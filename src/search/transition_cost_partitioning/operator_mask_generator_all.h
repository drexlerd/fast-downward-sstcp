#ifndef TRANSITION_COST_PARTITIONING_OPERATOR_MASK_GENERATOR_ALL_H
#define TRANSITION_COST_PARTITIONING_OPERATOR_MASK_GENERATOR_ALL_H

#include "operator_mask_generator.h"

using namespace std;

namespace transition_cost_partitioning {
class Abstraction;


class OperatorMaskGeneratorAll : public OperatorMaskGenerator {
  public:
    OperatorMaskGeneratorAll(const options::Options &opts);
    virtual ~OperatorMaskGeneratorAll() = default;

    virtual vector<bool> generate_mask(
      const Abstraction &abstraction) const override;
};

}

#endif
