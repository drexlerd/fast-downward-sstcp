#include "operator_mask_generator.h"

#include "../option_parser.h"
#include "../plugin.h"

namespace transition_cost_partitioning {

// ____________________________________________________________________________
OperatorMaskGenerator::OperatorMaskGenerator(const options::Options &) {
}

// ____________________________________________________________________________
static PluginTypePlugin<OperatorMaskGenerator> _type_plugin(
    "operator_mask_generator",
    "");

}