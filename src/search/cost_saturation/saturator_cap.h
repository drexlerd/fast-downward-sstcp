#ifndef COST_SATURATION_SATURATOR_CAP_H
#define COST_SATURATION_SATURATOR_CAP_H

#include "saturator.h"
#include "types.h"

#include <cassert>

namespace cost_saturation {
enum class Reachable {
    FROM_INIT,
    FROM_STATE,
    ALL,
};

class SaturatorCap : public Saturator {
    const Reachable reachable;
    const bool cap;
    std::vector<std::vector<int>> unreachable_from_init;
public:
    explicit SaturatorCap(const options::Options &opts);

    virtual void initialize(
        const Abstractions &abstractions,
        const std::vector<int> &costs,
        const State &initial_state) override;

    virtual SaturatorResult saturate(
        const Abstraction &abstraction,
        int abstraction_id,
        const std::vector<int> &costs,
        std::vector<int> &&h_values,
        int state_id) const override;
};
}

#endif
