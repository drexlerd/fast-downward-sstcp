#include "order_generator.h"

#include "../option_parser.h"
#include "../plugin.h"

using namespace std;

namespace transition_cost_partitioning {

// ____________________________________________________________________________
OrderGenerator::OrderGenerator(const options::Options &) {
}


static PluginTypePlugin<OrderGenerator> _type_plugin(
    "cp_order_generator",
    "Generate heuristic orders.");
}
