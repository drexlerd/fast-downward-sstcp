#include "operator_mask_generator_all.h"

#include "abstraction.h"

#include "../option_parser.h"
#include "../plugin.h"


namespace transition_cost_partitioning {


// ____________________________________________________________________________
OperatorMaskGeneratorAll::OperatorMaskGeneratorAll(const options::Options &opts) :
    OperatorMaskGenerator(opts) {
}

// ____________________________________________________________________________
vector<bool> OperatorMaskGeneratorAll::generate_mask(
    const Abstraction &abstraction) const {
    return vector<bool>(abstraction.get_num_operators(), true);
}

// ____________________________________________________________________________
static shared_ptr<OperatorMaskGenerator> _parse_all(OptionParser &parser) {
    Options opts = parser.parse();
    if (parser.dry_run())
        return nullptr;
    else
        return make_shared<OperatorMaskGeneratorAll>(opts);
}

// ____________________________________________________________________________
static Plugin<OperatorMaskGenerator> _plugin("operator_mask_generator_all", _parse_all);
    
}