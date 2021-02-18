#ifndef TRANSITION_COST_PARTITIONING_ABSTRACTION_MASK_GENERATOR_SIZE_H
#define TRANSITION_COST_PARTITIONING_ABSTRACTION_MASK_GENERATOR_SIZE_H

#include "abstraction_mask_generator.h"

using namespace std;

namespace transition_cost_partitioning {
class Abstraction;

/**
 * Abstractions with at most max_num_transitions are handled state-dependent.
 */
class AbstractionMaskGeneratorSize : public AbstractionMaskGenerator {
  private:
    const int max_num_transitions;

  public:
    /**
     * R6: Moveable and not copyable.
     */
    AbstractionMaskGeneratorSize(const options::Options &opts);
    AbstractionMaskGeneratorSize(const AbstractionMaskGeneratorSize &other) = delete;
    AbstractionMaskGeneratorSize& operator=(const AbstractionMaskGeneratorSize &other) = delete;
    AbstractionMaskGeneratorSize(AbstractionMaskGeneratorSize &&other) = default;
    AbstractionMaskGeneratorSize& operator=(AbstractionMaskGeneratorSize &&other) = default;
    virtual ~AbstractionMaskGeneratorSize() = default;

    virtual bool generate_mask(
      const Abstraction &abstraction,
      const TaskInfo &task_info) const override;
};

}

#endif
