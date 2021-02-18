#ifndef COST_SATURATION_SATURATOR_H
#define COST_SATURATION_SATURATOR_H

#include "types.h"

class State;

namespace options {
class OptionParser;
class Options;
}

namespace cost_saturation {
struct SaturatorResult {
    std::vector<int> saturated_costs;
    std::vector<int> h_values;

    SaturatorResult(std::vector<int> &&saturated_costs, std::vector<int> &&h_values)
        : saturated_costs(std::move(saturated_costs)),
          h_values(std::move(h_values)) {
    }
};

class Saturator {
protected:
    const bool use_general_costs;
    const bool recompute_h_values;

    virtual SaturatorResult get_saturator_result(
        const Abstraction &abstraction,
        const std::vector<int> &costs,
        std::vector<int> &&saturated_costs,
        std::vector<int> &&h_values,
        int state_h) const;
public:
    explicit Saturator(const options::Options &opts);
    virtual ~Saturator() = default;

    virtual void initialize(
        const Abstractions &abstractions,
        const std::vector<int> &costs,
        const State &initial_state);

    virtual SaturatorResult saturate(
        const Abstraction &abstraction,
        int abstraction_id,
        const std::vector<int> &costs,
        std::vector<int> &&h_values,
        int state_id) const = 0;
};

extern void add_saturator_options(options::OptionParser &parser);
}

#endif
