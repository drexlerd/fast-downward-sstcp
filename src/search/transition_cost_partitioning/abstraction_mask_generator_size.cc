#include "abstraction_mask_generator_size.h"

#include "abstraction.h"

#include "../option_parser.h"
#include "../plugin.h"
#include "../utils/logging.h"

namespace transition_cost_partitioning {


// ____________________________________________________________________________
AbstractionMaskGeneratorSize::AbstractionMaskGeneratorSize(const options::Options &opts) :
    AbstractionMaskGenerator(opts),
    max_num_transitions(opts.get<int>("max_num_transitions")) {
}

// ____________________________________________________________________________
bool AbstractionMaskGeneratorSize::generate_mask(
    const Abstraction &abstraction,
    const TaskInfo &) const {
    if (abstraction.get_num_transitions() < max_num_transitions) {
        return true;
    }
    return false;
}


// ____________________________________________________________________________
static shared_ptr<AbstractionMaskGenerator> _parse_all(OptionParser &parser) {
    parser.add_option<int>(
        "max_num_transitions",
        "",
        "infinity",
        Bounds("0", "infinity"));

    Options opts = parser.parse();
    if (parser.dry_run())
        return nullptr;
    else
        return make_shared<AbstractionMaskGeneratorSize>(opts);
}

// ____________________________________________________________________________
static Plugin<AbstractionMaskGenerator> _plugin("abstraction_mask_generator_size", _parse_all);
    
}