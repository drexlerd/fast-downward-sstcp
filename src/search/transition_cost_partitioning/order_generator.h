#ifndef TRANSITION_COST_PARTITIONING_ORDER_GENERATOR_H
#define TRANSITION_COST_PARTITIONING_ORDER_GENERATOR_H

#include "types.h"

#include <vector>

namespace options {
class OptionParser;
class Options;
}

namespace transition_cost_partitioning {
class Abstraction;

class OrderGenerator {
public:
    OrderGenerator(const options::Options &opts);
    virtual ~OrderGenerator() = default;

    virtual void initialize(
        const vector<unique_ptr<Abstraction>> &abstractions,
        const std::vector<int> &costs) = 0;

    virtual Order compute_order_for_state(
        const vector<unique_ptr<Abstraction>> &abstractions,
        const std::vector<int> &costs,
        const std::vector<int> &abstract_state_ids,
        bool verbose) = 0;
};

}

#endif
