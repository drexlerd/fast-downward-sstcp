#include "abstraction_mask_generator.h"

#include "../option_parser.h"
#include "../plugin.h"

namespace transition_cost_partitioning {

// ____________________________________________________________________________
AbstractionMaskGenerator::AbstractionMaskGenerator(const options::Options &) {
}

// ____________________________________________________________________________
void AbstractionMaskGenerator::initialize(const Abstractions &) const {

}

// ____________________________________________________________________________
static PluginTypePlugin<AbstractionMaskGenerator> _type_plugin(
    "abstraction_mask_generator",
    "");

}