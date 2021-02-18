#include "operator_mask_generator_none.h"

#include "abstraction.h"

#include "../option_parser.h"
#include "../plugin.h"


namespace transition_cost_partitioning {


// ____________________________________________________________________________
OperatorMaskGeneratorNone::OperatorMaskGeneratorNone(const options::Options &opts) :
    OperatorMaskGenerator(opts) {
}

// ____________________________________________________________________________
vector<bool> OperatorMaskGeneratorNone::generate_mask(
    const Abstraction &abstraction) const {
    return vector<bool>(abstraction.get_num_operators(), false);
}

// ____________________________________________________________________________
static shared_ptr<OperatorMaskGenerator> _parse_all(OptionParser &parser) {
    Options opts = parser.parse();
    if (parser.dry_run())
        return nullptr;
    else
        return make_shared<OperatorMaskGeneratorNone>(opts);
}

// ____________________________________________________________________________
static Plugin<OperatorMaskGenerator> _plugin("operator_mask_generator_none", _parse_all);
    
}