#include "abstraction_generator.h"

#include "../plugin.h"

using namespace std;

namespace transition_cost_partitioning {
static PluginTypePlugin<AbstractionGenerator> _type_plugin(
    "cp_abstraction_generator",
    "");
}
