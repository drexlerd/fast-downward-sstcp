#ifndef TRANSITION_COST_PARTITIONING_ABSTRACTION_MASK_GENERATOR_SPLIT_H
#define TRANSITION_COST_PARTITIONING_ABSTRACTION_MASK_GENERATOR_SPLIT_H

#include "abstraction_mask_generator.h"

using namespace std;

namespace transition_cost_partitioning {
class Abstraction;

class AbstractionMaskGeneratorSplit : public AbstractionMaskGenerator {
  private:
    const int max_num_transitions;

    const int max_variables_count;
  public:
    /**
     * R6: Moveable and not copyable.
     */
    AbstractionMaskGeneratorSplit(const options::Options &opts);
    AbstractionMaskGeneratorSplit(const AbstractionMaskGeneratorSplit &other) = delete;
    AbstractionMaskGeneratorSplit& operator=(const AbstractionMaskGeneratorSplit &other) = delete;
    AbstractionMaskGeneratorSplit(AbstractionMaskGeneratorSplit &&other) = default;
    AbstractionMaskGeneratorSplit& operator=(AbstractionMaskGeneratorSplit &&other) = default;
    virtual ~AbstractionMaskGeneratorSplit() = default;

    virtual bool generate_mask(
      const Abstraction &abstraction,
      const TaskInfo &task_info) const override;

    virtual void initialize(const Abstractions &abstractions) const override;
};

}

#endif
