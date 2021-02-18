#ifndef TRANSITION_COST_PARTITIONING_OPERATOR_MASK_GENERATOR_SIZE_H
#define TRANSITION_COST_PARTITIONING_OPERATOR_MASK_GENERATOR_SIZE_H

#include "operator_mask_generator.h"

#include <vector>

using namespace std;

namespace options {
class Options;
}

namespace transition_cost_partitioning {
class Abstraction;

/**
 * The MaskGeneratorSize generates an operator mask 
 * that sets an operator to state-dependent iff
 * (1) the operator was considered state-dependent in at most as many abstractions as the threshold _max_num_abstractions allows,
 * (2) the operator induces at most _max_num_transitions many state-changing transitions.
 */
class OperatorMaskGeneratorSize : public OperatorMaskGenerator {
  private:
    /**
     * num transitions threshold
     * {0, 10, 100, 1000, infty}
     */
    const int _max_num_transitions;

    /**
     * num abstractions threshold
     * {2, 4, 8, 16, infty}
     */
    const int _max_num_abstractions;

    /**
     * Count number of times that an operator is marked as state-dependent.
     */
    mutable vector<int> _count_sd;

  public:
    OperatorMaskGeneratorSize(const options::Options &opts);
    virtual ~OperatorMaskGeneratorSize() = default;

    virtual vector<bool> generate_mask(
      const Abstraction &abstraction) const override;
};

}

#endif
