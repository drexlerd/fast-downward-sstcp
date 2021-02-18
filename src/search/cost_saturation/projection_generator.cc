#include "projection_generator.h"

#include "explicit_projection_factory.h"
#include "projection.h"

#include "../option_parser.h"
#include "../plugin.h"

#include "../pdbs/dominance_pruning.h"
#include "../pdbs/pattern_database.h"
#include "../pdbs/pattern_generator.h"
#include "../task_utils/task_properties.h"
#include "../utils/logging.h"

#include <memory>

using namespace pdbs;
using namespace std;

namespace cost_saturation {
ProjectionGenerator::ProjectionGenerator(const options::Options &opts)
    : pattern_generator(
          opts.get<shared_ptr<pdbs::PatternCollectionGenerator>>("patterns")),
      dominance_pruning(opts.get<bool>("dominance_pruning")),
      create_complete_transition_system(opts.get<bool>("create_complete_transition_system")),
      use_add_after_delete_semantics(opts.get<bool>("use_add_after_delete_semantics")),
      debug(opts.get<bool>("debug")) {
}

Abstractions ProjectionGenerator::generate_abstractions(
    const shared_ptr<AbstractTask> &task) {
    utils::Timer patterns_timer;
    utils::Log log;
    TaskProxy task_proxy(*task);

    task_properties::verify_no_axioms(task_proxy);
    if (!create_complete_transition_system) {
        task_properties::verify_no_conditional_effects(task_proxy);
    }

    log << "Compute patterns" << endl;
    PatternCollectionInformation pattern_collection_info =
        pattern_generator->generate(task);
    shared_ptr<pdbs::PatternCollection> patterns =
        pattern_collection_info.get_patterns();

    int max_pattern_size = 0;
    for (const pdbs::Pattern &pattern : *patterns) {
        max_pattern_size = max(max_pattern_size, static_cast<int>(pattern.size()));
    }

    log << "Number of patterns: " << patterns->size() << endl;
    log << "Maximum pattern size: " << max_pattern_size << endl;
    log << "Time for computing patterns: " << patterns_timer << endl;

    if (dominance_pruning) {
        shared_ptr<PDBCollection> pdbs = pattern_collection_info.get_pdbs();
        shared_ptr<std::vector<PatternClique>> pattern_cliques =
            pattern_collection_info.get_pattern_cliques();
        prune_dominated_cliques(
            *patterns,
            *pdbs, *pattern_cliques, task_proxy.get_variables().size(),
            numeric_limits<double>::infinity());
    }

    log << "Build projections" << endl;
    utils::Timer pdbs_timer;
    shared_ptr<TaskInfo> task_info = make_shared<TaskInfo>(task_proxy);
    Abstractions abstractions;
    for (const pdbs::Pattern &pattern : *patterns) {
        if (debug) {
            log << "Pattern " << abstractions.size() + 1 << ": "
                << pattern << endl;
        }
        abstractions.push_back(
            create_complete_transition_system ?
            ExplicitProjectionFactory(
                task_proxy, pattern, use_add_after_delete_semantics).convert_to_abstraction() :
            utils::make_unique_ptr<Projection>(task_proxy, task_info, pattern));
        if (debug) {
            abstractions.back()->dump();
        }
    }

    int collection_size = 0;
    for (auto &abstraction : abstractions) {
        collection_size += abstraction->get_num_states();
    }

    log << "Time for building projections: " << pdbs_timer << endl;
    log << "Number of projections: " << abstractions.size() << endl;
    log << "Number of states in projections: " << collection_size << endl;
    return abstractions;
}

static shared_ptr<AbstractionGenerator> _parse(OptionParser &parser) {
    parser.document_synopsis(
        "Projection generator",
        "");

    parser.add_option<shared_ptr<pdbs::PatternCollectionGenerator>>(
        "patterns",
        "pattern generation method",
        OptionParser::NONE);
    parser.add_option<bool>(
        "dominance_pruning",
        "prune dominated patterns",
        "false");
    parser.add_option<bool>(
        "create_complete_transition_system",
        "create complete transition system",
        "false");
    parser.add_option<bool>(
        "use_add_after_delete_semantics",
        "skip transitions that are invalid according to add-after-delete semantics",
        "false");
    parser.add_option<bool>(
        "debug",
        "print debugging info",
        "false");

    Options opts = parser.parse();
    if (parser.dry_run())
        return nullptr;

    return make_shared<ProjectionGenerator>(opts);
}

static Plugin<AbstractionGenerator> _plugin("projections", _parse);
}
