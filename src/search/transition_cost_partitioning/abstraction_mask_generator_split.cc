#include "abstraction_mask_generator_split.h"

#include "abstraction.h"
#include "task_info.h"

#include "../option_parser.h"
#include "../plugin.h"
#include "../utils/logging.h"

namespace transition_cost_partitioning {

// ____________________________________________________________________________
AbstractionMaskGeneratorSplit::AbstractionMaskGeneratorSplit(const options::Options &opts) :
    AbstractionMaskGenerator(opts),
    max_num_transitions(opts.get<int>("max_num_transitions")),
    max_variables_count(opts.get<int>("max_variables_count")) {
}

// ____________________________________________________________________________
bool AbstractionMaskGeneratorSplit::generate_mask(
    const Abstraction &abstraction,
    const TaskInfo &task_info) const {
    // abstraction is too large => state-independent
    if (abstraction.get_num_transitions() >= max_num_transitions) {
        return false;
    }
    return true;
}

// ____________________________________________________________________________
void AbstractionMaskGenerator::initialize(const Abstractions &abstractions) const {

}

// ____________________________________________________________________________
static shared_ptr<AbstractionMaskGenerator> _parse_all(OptionParser &parser) {
    parser.add_option<int>(
        "max_num_transitions",
        "",
        "infinity",
        Bounds("0", "infinity"));

    parser.add_option<int>(
        "max_variables_count",
        "",
        "infinity",
        Bounds("0", "infinity"));

    Options opts = parser.parse();
    if (parser.dry_run())
        return nullptr;
    else
        return make_shared<AbstractionMaskGeneratorSplit>(opts);
}

// ____________________________________________________________________________
static Plugin<AbstractionMaskGenerator> _plugin("abstraction_mask_generator_split", _parse_all);
    
}