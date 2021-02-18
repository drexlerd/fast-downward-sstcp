#include "operator_mask_generator_size.h"

#include "abstraction.h"

#include "../option_parser.h"
#include "../plugin.h"

#include "../utils/timer.h"

namespace transition_cost_partitioning {


// ____________________________________________________________________________
OperatorMaskGeneratorSize::OperatorMaskGeneratorSize(const options::Options &opts) :
    OperatorMaskGenerator(opts),
    _max_num_transitions(opts.get<int>("max_num_transitions")),
    _max_num_abstractions(opts.get<int>("max_num_abstractions")) {
}


// ____________________________________________________________________________
vector<bool> OperatorMaskGeneratorSize::generate_mask(
    const Abstraction &abstraction) const {
    utils::Timer timer;
    int num_operators = abstraction.get_num_operators();
    if (_count_sd.empty()) {
        _count_sd = vector<int>(num_operators, 0);
    }

    int count_sd = 0;
    vector<bool> sd(num_operators, false);
    for (int op_id = 0; op_id < num_operators; ++op_id) {
        if (abstraction.get_num_transitions(op_id) <= _max_num_transitions &&
            _count_sd[op_id] < _max_num_abstractions) {
            sd[op_id] = true;
            ++_count_sd[op_id];
            ++count_sd;
        }
    }
    // cout << "Total sd operators: " << count_sd << "\n";
    // cout << "Total time to compute operator mask: " << timer() << "\n";
    
    return sd;
}


// ____________________________________________________________________________
static shared_ptr<OperatorMaskGenerator> _parse_all(OptionParser &parser) {
    parser.add_option<int>(
        "max_num_transitions",
        "",
        "infinity",
        Bounds("0", "infinity"));

    parser.add_option<int>(
        "max_num_abstractions",
        "",
        "infinity",
        Bounds("0", "infinity"));

    Options opts = parser.parse();
    if (parser.dry_run())
        return nullptr;
    else
        return make_shared<OperatorMaskGeneratorSize>(opts);
}

// ____________________________________________________________________________
static Plugin<OperatorMaskGenerator> _plugin("operator_mask_generator_size", _parse_all);
    
}