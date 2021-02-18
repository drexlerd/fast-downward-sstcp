#ifndef TRANSITION_COST_PARTITIONING_ABSTRACTION_MASK_GENERATOR_H
#define TRANSITION_COST_PARTITIONING_ABSTRACTION_MASK_GENERATOR_H

#include "types.h"

using namespace std;

namespace options {
class Options;
}

namespace transition_cost_partitioning {
class Abstraction;
class TaskInfo;

/**
 * The MaskGeneratorApplicability generates an operator mask
 */
class AbstractionMaskGenerator {
  public:
    /**
     * R6: Moveable and not copyable.
     */
    AbstractionMaskGenerator(const options::Options &opts);
    AbstractionMaskGenerator(const AbstractionMaskGenerator &other) = delete;
    AbstractionMaskGenerator& operator=(const AbstractionMaskGenerator &other) = delete;
    AbstractionMaskGenerator(AbstractionMaskGenerator &&other) = default;
    AbstractionMaskGenerator& operator=(AbstractionMaskGenerator &&other) = default;
    virtual ~AbstractionMaskGenerator() = default;

    /* Generate a mask for the given abstraction
    */
    virtual bool generate_mask(
      const Abstraction &abstraction,
      const TaskInfo &task_info) const = 0;

    /**
     * Precompute information.
     */
    virtual void initialize(const Abstractions &abstractions) const;
};

}

#endif
