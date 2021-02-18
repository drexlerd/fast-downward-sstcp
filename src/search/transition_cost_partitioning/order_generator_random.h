#ifndef TRANSITION_COST_PARTITIONING_ORDER_GENERATOR_RANDOM_H
#define TRANSITION_COST_PARTITIONING_ORDER_GENERATOR_RANDOM_H

#include "order_generator.h"

namespace options {
class Options;
}

namespace utils {
class RandomNumberGenerator;
}

namespace transition_cost_partitioning {
class Abstraction;

class OrderGeneratorRandom : public OrderGenerator {
    const std::shared_ptr<utils::RandomNumberGenerator> rng;
    std::vector<int> random_order;

public:
    explicit OrderGeneratorRandom(const options::Options &opts);

    virtual void initialize(
        const vector<unique_ptr<Abstraction>> &abstractions,
        const std::vector<int> &costs) override;

    virtual Order compute_order_for_state(
        const vector<unique_ptr<Abstraction>> &abstractions,
        const std::vector<int> &costs,
        const std::vector<int> &abstract_state_ids,
        bool verbose) override;
};
}

#endif
