#include "cartesian_abstraction_generator.h"

#include "task_info.h"
#include "explicit_abstraction_cegar.h"
#include "abstraction.h"
#include "types.h"

#include "../option_parser.h"
#include "../plugin.h"

#include "../cegar/abstraction.h"
#include "../cegar/abstract_state.h"
#include "../cegar/cegar.h"
#include "../cegar/cost_saturation.h"
#include "../cegar/refinement_hierarchy.h"
#include "../cegar/split_selector.h"
#include "../cegar/subtask_generators.h"
#include "../cegar/transition_system.h"
#include "../task_utils/task_properties.h"
#include "../utils/logging.h"
#include "../utils/rng_options.h"

using namespace std;

namespace transition_cost_partitioning {

// ____________________________________________________________________________
static pair<bool, unique_ptr<Abstraction>> convert_abstraction(
    cegar::Abstraction &cartesian_abstraction, 
    const TaskInfo &task_info, 
    const BddBuilder &bdd_builder) {
    // Compute g and h values.
    const cegar::TransitionSystem &ts =
        cartesian_abstraction.get_transition_system();
    int num_states = cartesian_abstraction.get_num_states();
    int initial_state_id = cartesian_abstraction.get_initial_state().get_id();
    //vector<int> g_values = cegar::compute_distances(
    //    ts.get_outgoing_transitions(), task_info->get_operator_costs(), {initial_state_id});
    vector<int> h_values = cegar::compute_distances(
        ts.get_incoming_transitions(), task_info.get_operator_costs(), cartesian_abstraction.get_goals());

    vector<bool> has_loop(task_info.get_num_operators(), false);
    vector<bool> has_outgoing(task_info.get_num_operators(), false);
    vector<int> num_transitions_by_operator(task_info.get_num_operators(), 0);
    vector<vector<Successor>> backward_graph(num_states);
    int num_transitions = 0;
    for (int target_id = 0; target_id < num_states; ++target_id) {   
        for (const cegar::Transition &transition : ts.get_incoming_transitions()[target_id]) {
            int source_id = transition.target_id;
            int op_id = transition.op_id;
            // ignore transitions that start in a dead-end.
            // We can do this because we saturate for such transitions by restricting unreachable states to each operator.
            if (h_values[source_id] != INF) {
                backward_graph[target_id].emplace_back(num_transitions, op_id, source_id);
                ++num_transitions_by_operator[op_id];
                ++num_transitions;
                has_outgoing[op_id] = true;
            }
        }

        if (h_values[target_id] != INF) {
            for (int op_id : ts.get_loops()[target_id]) {     
                has_loop[op_id] = true;
            }
        }
    }
    

    // initialize goal states
    unordered_set<int> goal_states(
        cartesian_abstraction.get_goals().begin(),
        cartesian_abstraction.get_goals().end());

    unique_ptr<cegar::SplitTree> split_tree = cartesian_abstraction.extract_split_tree();
    split_tree->initialize();

    bool unsolvable = h_values[initial_state_id] == INF;
    return {
        unsolvable,
        utils::make_unique_ptr<ExplicitAbstractionCegar>(
            task_info, 
            bdd_builder,
            utils::make_unique_ptr<CartesianAbstractionFunction>(
                cartesian_abstraction.extract_refinement_hierarchy()),                     
            num_transitions,
            num_states,
            initial_state_id, 
            move(goal_states),  
            move(backward_graph),
            move(num_transitions_by_operator),
            move(has_outgoing),
            move(has_loop),            
            move(split_tree))
    };
}


// ____________________________________________________________________________
CartesianAbstractionGenerator::CartesianAbstractionGenerator(
    const options::Options &opts)
    : subtask_generators(
          opts.get_list<shared_ptr<cegar::SubtaskGenerator>>("subtasks")),
      max_states(opts.get<int>("max_states")),
      max_transitions(opts.get<int>("max_transitions")),
      rng(utils::parse_rng_from_options(opts)),
      pick_split(static_cast<cegar::PickSplit>(opts.get<int>("pick"))),
      debug(opts.get<bool>("debug")),
      num_states(0),
      num_transitions(0) {
}


// ____________________________________________________________________________
void CartesianAbstractionGenerator::build_abstractions_for_subtasks(
    const vector<shared_ptr<AbstractTask>> &subtasks,
    const TaskInfo &task_info,
    const BddBuilder &bdd_builder,
    function<bool()> total_size_limit_reached,
    vector<unique_ptr<Abstraction>> &abstractions) {
    int remaining_subtasks = subtasks.size();
    for (const shared_ptr<AbstractTask> &subtask : subtasks) {
        /* To make the abstraction refinement process deterministic, we don't
           set a time limit. */
        const double max_time = numeric_limits<double>::infinity();

        cegar::CEGAR cegar(
            subtask,
            max(1, (max_states - num_states) / remaining_subtasks),
            max(1, (max_transitions - num_transitions) / remaining_subtasks),
            max_time,
            cegar::PickSplit::MAX_REFINED,
            *rng,
            debug);

        unique_ptr<cegar::Abstraction> cartesian_abstraction = cegar.extract_abstraction();

        num_states += cartesian_abstraction->get_num_states();
        num_transitions += cartesian_abstraction->get_transition_system().get_num_non_loops();

        pair<bool, unique_ptr<Abstraction>> result = convert_abstraction(*cartesian_abstraction, task_info, bdd_builder);
        bool unsolvable = result.first;
        abstractions.push_back(move(result.second));

        if (total_size_limit_reached() || unsolvable) {
            break;
        }

        --remaining_subtasks;
    }
}

vector<unique_ptr<Abstraction>> CartesianAbstractionGenerator::generate_abstractions(
    const shared_ptr<AbstractTask> &task,
    const TaskInfo &task_info,
    const BddBuilder &bdd_builder) {
    utils::Timer timer;
    utils::Log log;
    log << "Build Cartesian abstractions" << endl;

    /* The CEGAR code expects that some extra memory is reserved. Since using
       a memory padding leads to nondeterministic results, we only reserve
       some "dummy" memory. */
    utils::reserve_extra_memory_padding(0);

    function<bool()> total_size_limit_reached =
        [&] () {
            return num_states >= max_states || num_transitions >= max_transitions;
        };

    vector<unique_ptr<Abstraction>> abstractions;
    for (const auto &subtask_generator : subtask_generators) {
        cegar::SharedTasks subtasks = subtask_generator->get_subtasks(task);
        build_abstractions_for_subtasks(
            subtasks, 
            task_info,
            bdd_builder,
            total_size_limit_reached,            
            abstractions);
        if (total_size_limit_reached()) {
            break;
        }
    }

    if (utils::extra_memory_padding_is_reserved()) {
        utils::release_extra_memory_padding();
    }

    log << "Cartesian abstractions built: " << abstractions.size() << endl;
    log << "Time for building Cartesian abstractions: " << timer << endl << endl;
    return abstractions;
}

static shared_ptr<AbstractionGenerator> _parse(OptionParser &parser) {
    parser.document_synopsis(
        "Cartesian abstraction generator",
        "");

    parser.add_list_option<shared_ptr<cegar::SubtaskGenerator>>(
        "subtasks",
        "subtask generators",
        "[landmarks(order=random, random_seed=0), goals(order=random, random_seed=0)]");
    parser.add_option<int>(
        "max_states",
        "maximum sum of abstract states over all abstractions",
        "infinity",
        Bounds("0", "infinity"));
    parser.add_option<int>(
        "max_transitions",
        "maximum sum of state-changing transitions (excluding self-loops) over "
        "all abstractions",
        "1000000",
        Bounds("0", "infinity"));
    parser.add_option<bool>(
        "debug",
        "print debugging info",
        "false");
    vector<string> pick_strategies;
    pick_strategies.push_back("RANDOM");
    pick_strategies.push_back("MIN_UNWANTED");
    pick_strategies.push_back("MAX_UNWANTED");
    pick_strategies.push_back("MIN_REFINED");
    pick_strategies.push_back("MAX_REFINED");
    pick_strategies.push_back("MIN_HADD");
    pick_strategies.push_back("MAX_HADD");
    parser.add_enum_option(
        "pick", pick_strategies, "split-selection strategy", "MAX_REFINED");
    utils::add_rng_options(parser);

    Options opts = parser.parse();
    if (parser.dry_run())
        return nullptr;

    return make_shared<CartesianAbstractionGenerator>(opts);
}

static Plugin<AbstractionGenerator> _plugin("cp_cartesian", _parse);
}
