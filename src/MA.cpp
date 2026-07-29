//
// Created by Yinghao Qin on 19/12/2023.
//

#include "MA.hpp"

#include "algorithm_constants.hpp"
#include "follower.hpp"
#include "initializer.hpp"
#include "leader.hpp"
#include "reproduction.hpp"

#include <cfloat>
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <stdexcept>

namespace {

void add_operator_stats(
    std::array<
        LocalSearchOperatorStats,
        LOCAL_SEARCH_OPERATOR_COUNT>& generationStats,
    const LocalSearchResult& result) {
    for (std::size_t operatorIndex = 0;
         operatorIndex < LOCAL_SEARCH_OPERATOR_COUNT;
         ++operatorIndex) {
        auto& destination = generationStats[operatorIndex];
        const auto& source = result.operatorStats[operatorIndex];
        destination.calls += source.calls;
        destination.accepts += source.accepts;
        destination.distanceCalls += source.distanceCalls;
        destination.upperGain += source.upperGain;
        destination.gammaCrosses += source.gammaCrosses;
    }
}

void add_allocation_stats(
    LocalSearchAllocationStats& destination,
    const LocalSearchAllocationStats& source) {
    destination.selections += source.selections;
    destination.forcedLocalOptima += source.forcedLocalOptima;
    destination.exploratorySelections += source.exploratorySelections;
    destination.localOptimumTerminations +=
        source.localOptimumTerminations;
    destination.moveLimitTerminations +=
        source.moveLimitTerminations;
    destination.distanceLimitTerminations +=
        source.distanceLimitTerminations;
    destination.acceptedMoves += source.acceptedMoves;
    destination.neighborhoodCalls += source.neighborhoodCalls;
    destination.distanceCalls += source.distanceCalls;
    destination.upperGain += source.upperGain;
    destination.gammaCrosses += source.gammaCrosses;
    destination.parentUses += source.parentUses;
    destination.lowerArchiveEntries += source.lowerArchiveEntries;
    destination.parentReward += source.parentReward;
    destination.lowerReward += source.lowerReward;
    destination.gammaReward += source.gammaReward;
    destination.continuationGainSignal +=
        source.continuationGainSignal;
    destination.reward += source.reward;
    destination.incrementalCostUnits +=
        source.incrementalCostUnits;
    destination.selectionScore += source.selectionScore;
}

void add_operator_learning_stats(
    OperatorLearningStats& destination,
    const OperatorLearningStats& source) {
    destination.operatorStats.calls +=
        source.operatorStats.calls;
    destination.operatorStats.accepts +=
        source.operatorStats.accepts;
    destination.operatorStats.distanceCalls +=
        source.operatorStats.distanceCalls;
    destination.operatorStats.upperGain +=
        source.operatorStats.upperGain;
    destination.operatorStats.gammaCrosses +=
        source.operatorStats.gammaCrosses;
    destination.creditedReward += source.creditedReward;
    destination.score += source.score;
    destination.selectionProbability +=
        source.selectionProbability;
}

}  // namespace

using std::endl;
using std::fixed;
using std::make_shared;
using std::make_unique;
using std::pair;
using std::setprecision;
using std::shared_ptr;
using std::size_t;
using std::sort;
using std::string;
using std::to_string;
using std::uniform_real_distribution;
using std::vector;

MA::MA(Case* instance, const Parameters& parameters) {
    // init parameters
    this->instance = instance;
    this->randomEngine = std::mt19937(
        static_cast<std::mt19937::result_type>(parameters.seed));
    std::seed_seq localSearchSeed{parameters.seed, 0x4C53, 0x52564E44};
    this->localSearchEngine.seed(localSearchSeed);
    std::seed_seq allocationSeed{parameters.seed, 0x4C53, 0x414C4C4F};
    this->localSearchAllocationEngine.seed(allocationSeed);
    std::seed_seq operatorSelectionSeed{
        parameters.seed,
        0x4C53,
        0x4F504552};
    this->operatorSelectionEngine.seed(operatorSelectionSeed);
    this->seed = parameters.seed;
    this->isMaxEvals = parameters.stopCriteria;
    this->enableLogging = parameters.enableLogging;
    this->statsDirectory = parameters.statsPath;

    uniform_real_distribution<double> udist(0.0, 1.0);
    this->uniformRealDis = udist;

    // hyperparameters for MA
    this->popSize = parameters.popSize;
    this->mutationProb = parameters.mutationProb;
    this->mutationIndProb = parameters.mutationIndProb;
    this->tournamentSize = parameters.tournamentSize;
    this->localSearchIntensity = parameters.localSearchIntensity;
    this->localSearchPolicy = parameters.localSearchPolicy;
    this->operatorSelectionPolicy =
        parameters.operatorSelectionPolicy;
    this->operatorCostWeight =
        parameters.operatorCostWeight;
    this->parentPoolRatio = parameters.parentPoolRatio;
    this->qualityRatio = parameters.qualityRatio;
    this->verifiedUpperRatio = parameters.verifiedUpperRatio;
    this->pureImmigrantRatio = parameters.pureImmigrantRatio;

    this->routeCapacity = this->instance->vehicleNumber * 3;
    // One upper-level route can contain depot + all customers + depot.
    this->nodeCapacity = this->instance->customerNumber + 2;
    this->generation = 0;
    this->lowerLevelTriggerRatio = parameters.gamma;
    this->globalBestUpperCost = DBL_MAX;
    this->lastUpperImprovementDistanceCalls = 0;
    this->lastLowerImprovementDistanceCalls = 0;
    this->recentGammaEntryRate = 0.0;
}

MA::~MA() {
    verifiedBest.reset();
    upperBestIndividual.reset();
    population.clear();
    populationBuffer.clear();
}

void MA::run() {
    start = std::chrono::high_resolution_clock::now();
    end = start;
    duration = end - start;
    if (enableLogging) {
        open_log_for_evolution();
        open_log_for_local_search();
    }

    initialize_search();
    while (isMaxEvals == 1
        ? !reached_evaluation_limit()
        : !reached_time_limit(duration)) {
        run_generation();
        duration = std::chrono::high_resolution_clock::now() - start;
        if (enableLogging) {
            const bool stopping =
                isMaxEvals == 1
                ? reached_evaluation_limit()
                : reached_time_limit(duration);
            if (generation % EVOLUTION_LOG_INTERVAL == 0
                || stopping) {
                flush_row_into_evol_log();
            }
        }
    }

    if (enableLogging) {
        close_log_for_evolution();
        close_log_for_local_search();
    }

    if (verifiedBest == nullptr
        || verifiedBest->get_lower_cost() >= INFEASIBLE_COST) {
        if (upperBestIndividual != nullptr) {
            if (verifiedBest == nullptr) {
                verifiedBest =
                    make_unique<Individual>(*upperBestIndividual);
            } else {
                verifiedBest->copy_from(*upperBestIndividual);
            }
        }
    }

    if (verifiedBest == nullptr) {
        throw std::runtime_error("search finished without a feasible complete solution");
    }
    Follower::refine_charging_by_enumeration(*verifiedBest, *instance);
    if (verifiedBest->get_lower_cost() >= INFEASIBLE_COST) {
        throw std::runtime_error("search finished without a feasible complete solution");
    }
    if (enableLogging) {
        save_log_for_solution();
    }
}

// stop criterion: max evals
bool MA::reached_evaluation_limit() const {
    return instance->get_distance_calls()
        >= instance->get_evaluation_limit_distance_calls();
}

// stop criterion: max execute time
bool MA::reached_time_limit(const std::chrono::duration<double>& runningTime) const {
    bool flag;
    if (runningTime.count() >= instance->maxExecTime)
        flag = true;
    else
        flag = false;

    return flag;
}

void MA::initialize_population_with_clustering() {
    for (int i = 0; i < popSize; ++i) {
        vector<vector<int>> routes = Initializer::build_with_clustering(*instance, randomEngine);
        population.push_back(std::make_shared<Individual>(routeCapacity, nodeCapacity, routes,
                                                          instance->fitness_evaluation(routes),
                                                          instance->compute_demand_sum(routes)));
    }
}

void MA::initialize_population_with_random_split() {
    for (int i = 0; i < popSize; ++i) {
        vector<vector<int>> routes = Initializer::build_with_random_split(*instance, randomEngine);
        population.push_back(std::make_shared<Individual>(routeCapacity, nodeCapacity, routes,
                                                          instance->fitness_evaluation(routes),
                                                          instance->compute_demand_sum(routes)));
    }
}

void MA::initialize_population_with_direct_encoding() {
    for (int i = 0; i < popSize; ++i) {
        vector<vector<int>> routes = Initializer::build_with_direct_encoding(*instance, randomEngine);
        population.push_back(std::make_shared<Individual>(routeCapacity, nodeCapacity, routes,
                                                          instance->fitness_evaluation(routes),
                                                          instance->compute_demand_sum(routes)));
    }
}

void MA::open_log_for_evolution() {
    const std::filesystem::path directoryPath =
        std::filesystem::path(statsDirectory) / instance->instanceName / to_string(seed);
    create_directories_if_not_exists(directoryPath.string());

    string filename = "evols." + instance->instanceName + ".csv";
    logEvolution.open(directoryPath / filename);
    logEvolution << EVOLUTION_LOG_HEADER << "\n";
}

void MA::flush_row_into_evol_log() {
    const double evalsUsed = instance->get_evals();
    const double progress = evalsUsed / instance->maxEvals;
    evolutionRows << generation << ","
                  << evalsUsed << ","
                  << globalBestUpperCost << ","
                  << verifiedBest->get_lower_cost() << ","
                  << progress << ","
                  << duration.count() << "\n";
}

void MA::close_log_for_evolution() {
    logEvolution << evolutionRows.str();
    evolutionRows.clear();
    logEvolution.close();
}

void MA::open_log_for_local_search() {
    const std::filesystem::path directoryPath =
        std::filesystem::path(statsDirectory) / instance->instanceName / to_string(seed);
    create_directories_if_not_exists(directoryPath.string());

    logLocalSearchOperators.open(
        directoryPath / "local-search-operators.tsv");
    logLocalSearchOperators << LOCAL_SEARCH_OPERATOR_LOG_HEADER << "\n";
    if (localSearchPolicy != LocalSearchPolicy::Static) {
        logLocalSearchAllocation.open(
            directoryPath / "local-search-allocation.tsv");
        logLocalSearchAllocation
            << LOCAL_SEARCH_ALLOCATION_LOG_HEADER << "\n";
    }
    if (operatorSelectionPolicy
        == OperatorSelectionPolicy::Online) {
        logOperatorLearning.open(
            directoryPath / "operator-learning.tsv");
        logOperatorLearning
            << OPERATOR_LEARNING_LOG_HEADER << "\n";
    }
}

void MA::flush_local_search_log() {
    if (logLocalSearchOperators.is_open()) {
        logLocalSearchOperators << localSearchOperatorRows.str();
        localSearchOperatorRows.str("");
        localSearchOperatorRows.clear();
    }
    if (logLocalSearchAllocation.is_open()) {
        logLocalSearchAllocation << localSearchAllocationRows.str();
        localSearchAllocationRows.str("");
        localSearchAllocationRows.clear();
    }
    if (logOperatorLearning.is_open()) {
        logOperatorLearning << operatorLearningRows.str();
        operatorLearningRows.str("");
        operatorLearningRows.clear();
    }
}

void MA::accumulate_local_search_operator_stats(
    const std::array<
        LocalSearchOperatorStats,
        LOCAL_SEARCH_OPERATOR_COUNT>& stats) {
    for (std::size_t index = 0; index < stats.size(); ++index) {
        auto& destination = pendingLocalSearchOperatorStats[index];
        const auto& source = stats[index];
        destination.calls += source.calls;
        destination.accepts += source.accepts;
        destination.distanceCalls += source.distanceCalls;
        destination.upperGain += source.upperGain;
        destination.gammaCrosses += source.gammaCrosses;
    }
    ++pendingLocalSearchOperatorGenerations;
}

void MA::write_local_search_operator_snapshot() {
    if (pendingLocalSearchOperatorGenerations == 0) {
        return;
    }

    for (std::size_t index = 0;
         index < LOCAL_SEARCH_OPERATOR_COUNT;
         ++index) {
        const auto localSearchOperator =
            static_cast<LocalSearchOperator>(index);
        const auto& stats =
            pendingLocalSearchOperatorStats[index];
        localSearchOperatorRows
            << setprecision(12)
            << generation << "\t"
            << pendingLocalSearchOperatorGenerations << "\t"
            << Leader::operator_name(localSearchOperator) << "\t"
            << stats.calls << "\t"
            << stats.accepts << "\t"
            << instance->distance_calls_to_evals(
                stats.distanceCalls) << "\t"
            << stats.upperGain << "\t"
            << stats.gammaCrosses << "\n";
    }

    pendingLocalSearchOperatorStats = {};
    pendingLocalSearchOperatorGenerations = 0;
}

void MA::accumulate_local_search_allocation_stats(
    const std::array<LocalSearchAllocationStats, 3>& stats) {
    for (std::size_t index = 0; index < stats.size(); ++index) {
        add_allocation_stats(
            pendingLocalSearchAllocationStats[index],
            stats[index]);
    }
    ++pendingLocalSearchAllocationGenerations;
}

void MA::write_local_search_allocation_snapshot() {
    if (pendingLocalSearchAllocationGenerations == 0
        || localSearchPolicy == LocalSearchPolicy::Static) {
        return;
    }

    const std::array<LocalSearchIntensity, 3> intensities = {
        LocalSearchIntensity::Weak,
        LocalSearchIntensity::Medium,
        localSearchIntensity,
    };
    for (std::size_t index = 0; index < intensities.size(); ++index) {
        const LocalSearchIntensity intensity = intensities[index];
        const auto& stats =
            pendingLocalSearchAllocationStats[index];
        const double selectionCount =
            static_cast<double>(stats.selections);
        localSearchAllocationRows
            << setprecision(12)
            << generation << "\t"
            << pendingLocalSearchAllocationGenerations << "\t"
            << local_search_policy_name(localSearchPolicy) << "\t"
            << LocalSearchAllocationRunner::intensity_name(
                intensity) << "\t"
            << stats.selections << "\t"
            << stats.forcedLocalOptima << "\t"
            << stats.exploratorySelections << "\t"
            << stats.localOptimumTerminations << "\t"
            << stats.moveLimitTerminations << "\t"
            << stats.distanceLimitTerminations << "\t"
            << stats.acceptedMoves << "\t"
            << stats.neighborhoodCalls << "\t"
            << instance->distance_calls_to_evals(
                stats.distanceCalls) << "\t"
            << stats.upperGain << "\t"
            << stats.gammaCrosses << "\t"
            << stats.parentUses << "\t"
            << stats.lowerArchiveEntries << "\t"
            << stats.parentReward << "\t"
            << stats.lowerReward << "\t"
            << stats.gammaReward << "\t"
            << stats.continuationGainSignal << "\t"
            << stats.reward << "\t"
            << (stats.selections > 0
                ? stats.incrementalCostUnits / selectionCount
                : 0.0) << "\t"
            << (stats.selections > 0
                ? stats.selectionScore / selectionCount
                : 0.0) << "\n";
    }

    pendingLocalSearchAllocationStats = {};
    pendingLocalSearchAllocationGenerations = 0;
}

void MA::accumulate_operator_learning_stats(
    const OnlineOperatorLearner::GenerationStats& stats) {
    for (std::size_t index = 0; index < stats.size(); ++index) {
        add_operator_learning_stats(
            pendingOperatorLearningStats[index],
            stats[index]);
    }
    ++pendingOperatorLearningGenerations;
}

void MA::write_operator_learning_snapshot() {
    if (pendingOperatorLearningGenerations == 0
        || operatorSelectionPolicy
            != OperatorSelectionPolicy::Online) {
        return;
    }

    const double generationCount = static_cast<double>(
        pendingOperatorLearningGenerations);
    for (std::size_t index = 0;
         index < LOCAL_SEARCH_OPERATOR_COUNT;
         ++index) {
        const auto localSearchOperator =
            static_cast<LocalSearchOperator>(index);
        const auto& stats =
            pendingOperatorLearningStats[index];
        const double callCount = static_cast<double>(
            stats.operatorStats.calls);
        operatorLearningRows
            << setprecision(12)
            << generation << "\t"
            << pendingOperatorLearningGenerations << "\t"
            << Leader::operator_name(localSearchOperator) << "\t"
            << stats.operatorStats.calls << "\t"
            << stats.operatorStats.accepts << "\t"
            << instance->distance_calls_to_evals(
                stats.operatorStats.distanceCalls) << "\t"
            << stats.operatorStats.upperGain << "\t"
            << stats.operatorStats.gammaCrosses << "\t"
            << stats.creditedReward << "\t"
            << (stats.operatorStats.calls > 0
                ? static_cast<double>(
                    stats.operatorStats.distanceCalls)
                    / callCount
                : 0.0) << "\t"
            << stats.score / generationCount << "\t"
            << stats.selectionProbability / generationCount
            << "\n";
    }

    pendingOperatorLearningStats = {};
    pendingOperatorLearningGenerations = 0;
}

void MA::close_log_for_local_search() {
    write_local_search_operator_snapshot();
    write_local_search_allocation_snapshot();
    write_operator_learning_snapshot();
    flush_local_search_log();
    logLocalSearchOperators.close();
    logLocalSearchAllocation.close();
    logOperatorLearning.close();
}

void MA::save_log_for_solution() {
    const std::filesystem::path directoryPath =
        std::filesystem::path(statsDirectory) / instance->instanceName / to_string(seed);
    create_directories_if_not_exists(directoryPath.string());
    string filename = "solution." + instance->instanceName + ".txt";

    logSolution.open(directoryPath / filename);
    logSolution << fixed << setprecision(5) << verifiedBest->get_lower_cost() << endl;
    pair<int*, int> tourInfo = verifiedBest->get_tour();
    for (int i = 0; i < tourInfo.second; ++i) {
        logSolution << tourInfo.first[i] << ",";
    }
    logSolution << endl;
    logSolution.close();
}

void MA::initialize_search() {
    localSearchAllocator.reset();
    operatorLearner.reset(operatorCostWeight);
    pendingLocalSearchOperatorStats = {};
    pendingLocalSearchOperatorGenerations = 0;
    pendingLocalSearchAllocationStats = {};
    pendingLocalSearchAllocationGenerations = 0;
    pendingOperatorLearningStats = {};
    pendingOperatorLearningGenerations = 0;
    retainedLowerElite.reset();
    upperBestIndividual.reset();
    population.clear();
    populationBuffer.clear();
    population.reserve(static_cast<size_t>(popSize));
    populationBuffer.reserve(static_cast<size_t>(popSize));
    mixedLocalSearchWorkspaces.clear();
    initialize_population_with_clustering();
    std::vector<std::vector<int>> emptyVector2D;
    std::vector<int> emptyVector1D;
    verifiedBest = make_unique<Individual>(
        routeCapacity,
        nodeCapacity,
        emptyVector2D,
        INFEASIBLE_COST,
        emptyVector1D);
    if (!population.empty()) {
        const shared_ptr<Individual> initialBest =
            Reproduction::best_by_upper_cost(population);
        upperBestIndividual =
            make_unique<Individual>(*initialBest);
        globalBestUpperCost =
            upperBestIndividual->get_upper_cost();
    } else {
        globalBestUpperCost = DBL_MAX;
    }
    lastUpperImprovementDistanceCalls =
        instance->get_distance_calls();
    lastLowerImprovementDistanceCalls =
        instance->get_distance_calls();
    recentGammaEntryRate = 0.0;
}

void MA::run_generation() {
    generation++;

    shared_ptr<Individual> unchangedLowerElite = retainedLowerElite;
    LocalSearchAllocationRun mixedLocalSearch;
    std::array<
        LocalSearchOperatorStats,
        LOCAL_SEARCH_OPERATOR_COUNT> generationOperatorStats{};
    OnlineOperatorLearner::GenerationStats
        generationOperatorLearningStats{};
    LocalSearchOperatorSelectionTable
        generationOperatorSelectionTable;
    const LocalSearchOperatorSelectionTable*
        generationOperatorSelectionTablePointer = nullptr;
    if (operatorSelectionPolicy
        == OperatorSelectionPolicy::Online) {
        generationOperatorSelectionTable =
            Leader::build_operator_selection_table(
                operatorLearner.selection_weights(),
                OnlineOperatorLearner::
                    UNIFORM_EXPLORATION_RATE);
        generationOperatorSelectionTablePointer =
            &generationOperatorSelectionTable;
    }
    shared_ptr<Individual> bestUpperCandidate = Reproduction::best_by_upper_cost(population);
    const ParentCandidate frozenUpperReference =
        Reproduction::make_parent_candidate(*upperBestIndividual);
    const double frozenTriggerUpperBound =
        frozenUpperReference.upperCost * lowerLevelTriggerRatio;
    const std::uint64_t evaluationLimitDistanceCalls =
        instance->get_evaluation_limit_distance_calls();
    const std::uint64_t generationStartDistanceCalls =
        instance->get_distance_calls();
    const double budgetProgress =
        evaluationLimitDistanceCalls > 0
        ? static_cast<double>(generationStartDistanceCalls)
            / static_cast<double>(evaluationLimitDistanceCalls)
        : 0.0;
    const double upperStagnation =
        evaluationLimitDistanceCalls > 0
        ? static_cast<double>(
            generationStartDistanceCalls
            - lastUpperImprovementDistanceCalls)
            / static_cast<double>(evaluationLimitDistanceCalls)
        : 0.0;
    const double lowerStagnation =
        evaluationLimitDistanceCalls > 0
        ? static_cast<double>(
            generationStartDistanceCalls
            - lastLowerImprovementDistanceCalls)
            / static_cast<double>(evaluationLimitDistanceCalls)
        : 0.0;
    double populationDispersion = 0.0;
    if (localSearchPolicy == LocalSearchPolicy::OnlineIndividual
        && !population.empty()) {
        for (const auto& individual : population) {
            populationDispersion +=
                Reproduction::adjacency_distance(
                    Reproduction::make_parent_candidate(*individual),
                    frozenUpperReference);
        }
        populationDispersion /=
            static_cast<double>(population.size());
    }

    if (localSearchPolicy == LocalSearchPolicy::Static) {
        auto applyLocalSearch =
            [&](const shared_ptr<Individual>& individual) {
            const bool canReuseLowerElite =
                individual == unchangedLowerElite
                && individual->is_upper_locally_optimal()
                && individual->get_lower_cost() < INFEASIBLE_COST;
            if (canReuseLowerElite) {
                return;
            }

            if (enableLogging) {
                const LocalSearchResult result =
                    Leader::improve_with_eight_neighborhood_rvnd_one_move(
                        *individual,
                        *instance,
                        localSearchEngine,
                        localSearchIntensity,
                        localSearchWorkspace,
                        frozenTriggerUpperBound);
                add_operator_stats(generationOperatorStats, result);
            } else {
                Leader::improve_with_eight_neighborhood_rvnd_one_move(
                    *individual,
                    *instance,
                    localSearchEngine,
                    localSearchIntensity,
                    localSearchWorkspace);
            }
        };

        for (auto& individual : population) {
            applyLocalSearch(individual);
        }
    } else {
        mixedLocalSearch = LocalSearchAllocationRunner::run(
            population,
            unchangedLowerElite,
            *instance,
            generation,
            localSearchPolicy,
            localSearchIntensity,
            frozenUpperReference,
            frozenTriggerUpperBound,
            budgetProgress,
            upperStagnation,
            lowerStagnation,
            populationDispersion,
            recentGammaEntryRate,
            localSearchEngine,
            localSearchAllocationEngine,
            mixedLocalSearchWorkspaces,
            localSearchAllocator,
            generationOperatorSelectionTablePointer,
            operatorSelectionPolicy
                    == OperatorSelectionPolicy::Online
                ? &operatorSelectionEngine
                : nullptr);
        generationOperatorStats =
            mixedLocalSearch.operatorStats;
    }

    // Build the quality-diversity parent pool before follower evaluation, so
    // reproduction remains independent from lower-level triggering.
    vector<shared_ptr<Individual>> rankedUpperSolutions = population;
    sort(rankedUpperSolutions.begin(), rankedUpperSolutions.end(), [](const shared_ptr<Individual>& lhs, const shared_ptr<Individual>& rhs) {
        return lhs->get_upper_cost() < rhs->get_upper_cost();
    });
    const size_t targetParentPoolSize = static_cast<size_t>(
        std::ceil(static_cast<double>(popSize) * parentPoolRatio));
    vector<ParentCandidate> parentPool = Reproduction::build_quality_diversity_parent_pool(
        rankedUpperSolutions,
        targetParentPoolSize,
        qualityRatio);

    vector<shared_ptr<Individual>> followerCandidates;
    shared_ptr<Individual> generationBestUpper = Reproduction::best_by_upper_cost(population);
    if (generationBestUpper->get_upper_cost() < globalBestUpperCost) {
        upperBestIndividual->copy_from(*generationBestUpper);
        globalBestUpperCost =
            upperBestIndividual->get_upper_cost();
        lastUpperImprovementDistanceCalls =
            instance->get_distance_calls();
    }

    const double triggerUpperBound = globalBestUpperCost * lowerLevelTriggerRatio;
    for (auto& individual : population) {
        if (individual->get_upper_cost() <= triggerUpperBound) {
            followerCandidates.push_back(individual);
        }
    }

    vector<shared_ptr<Individual>> evaluatedCompleteSolutions;
    for (auto& individual : followerCandidates) {
        const bool canReuseLowerElite = individual == unchangedLowerElite
            && individual->get_lower_cost() < INFEASIBLE_COST;
        if (!canReuseLowerElite) {
            Follower::optimize_charging(
                *individual,
                *instance,
                followerWorkspace);
        }
        evaluatedCompleteSolutions.push_back(individual);
    }

    shared_ptr<Individual> lowerElite;
    if (!evaluatedCompleteSolutions.empty()) {
        const shared_ptr<Individual> bestEvaluatedComplete =
            Reproduction::best_by_lower_cost(evaluatedCompleteSolutions);
        if (bestEvaluatedComplete->get_lower_cost() < INFEASIBLE_COST) {
            lowerElite = bestEvaluatedComplete;
        }
        if (verifiedBest->get_lower_cost() > bestEvaluatedComplete->get_lower_cost()) {
            verifiedBest->copy_from(*bestEvaluatedComplete);
            lastLowerImprovementDistanceCalls =
                instance->get_distance_calls();
        }
    }
    const bool retainVerifiedFallback =
        lowerElite == nullptr && verifiedBest->get_lower_cost() < INFEASIBLE_COST;

    if (localSearchPolicy == LocalSearchPolicy::OnlineNonContextual
        || localSearchPolicy == LocalSearchPolicy::OnlineIndividual) {
        LocalSearchAllocationRunner::assign_lower_archive_feedback(
            mixedLocalSearch,
            evaluatedCompleteSolutions,
            localSearchAllocator);
    }

    const bool hasLowerElite = lowerElite != nullptr || retainVerifiedFallback;
    const int offspringTarget = popSize - (hasLowerElite ? 1 : 0);
    const bool hasVerifiedBest = verifiedBest->get_lower_cost() < INFEASIBLE_COST;
    vector<int> parentUseCounts;
    vector<vector<int>> chromosomes = Reproduction::create_offspring(
        parentPool,
        verifiedBest.get(),
        hasVerifiedBest,
        instance->customers,
        offspringTarget,
        tournamentSize,
        mutationProb,
        mutationIndProb,
        verifiedUpperRatio,
        pureImmigrantRatio,
        randomEngine,
        uniformRealDis,
        reproductionWorkspace,
        &parentUseCounts);

    if (localSearchPolicy != LocalSearchPolicy::Static) {
        LocalSearchAllocationRunner::assign_parent_use_feedback(
            mixedLocalSearch,
            parentPool,
            parentUseCounts);
        LocalSearchAllocationRunner::finalize_feedback(
            mixedLocalSearch,
            localSearchPolicy,
            localSearchAllocator);
        if (operatorSelectionPolicy
            == OperatorSelectionPolicy::Online) {
            generationOperatorLearningStats =
                operatorLearner.update(mixedLocalSearch);
        }
        const int gammaEntries = static_cast<int>(std::count_if(
            mixedLocalSearch.records.begin(),
            mixedLocalSearch.records.end(),
            [](const AllocatedLocalSearchRecord& record) {
                return record.crossedGamma;
            }));
        recentGammaEntryRate = mixedLocalSearch.records.empty()
            ? 0.0
            : static_cast<double>(gammaEntries)
                / static_cast<double>(mixedLocalSearch.records.size());
    }

    if (enableLogging) {
        accumulate_local_search_operator_stats(
            generationOperatorStats);
        if (pendingLocalSearchOperatorGenerations
            == LOCAL_SEARCH_OPERATOR_LOG_INTERVAL) {
            write_local_search_operator_snapshot();
        }
        if (localSearchPolicy != LocalSearchPolicy::Static) {
            accumulate_local_search_allocation_stats(
                mixedLocalSearch.stats);
            if (pendingLocalSearchAllocationGenerations
                == LOCAL_SEARCH_ALLOCATION_LOG_INTERVAL) {
                write_local_search_allocation_snapshot();
            }
        }
        if (operatorSelectionPolicy
            == OperatorSelectionPolicy::Online) {
            accumulate_operator_learning_stats(
                generationOperatorLearningStats);
            if (pendingOperatorLearningGenerations
                == OPERATOR_LEARNING_LOG_INTERVAL) {
                write_operator_learning_snapshot();
            }
        }
        if (generation % LOCAL_SEARCH_OPERATOR_LOG_INTERVAL == 0) {
            flush_local_search_log();
        }
    }

    // All information needed from the old population is now materialized.
    evaluatedCompleteSolutions.clear();
    followerCandidates.clear();
    rankedUpperSolutions.clear();
    mixedLocalSearch.records.clear();
    bestUpperCandidate.reset();
    generationBestUpper.reset();
    unchangedLowerElite.reset();
    retainedLowerElite.reset();

    populationBuffer.clear();
    size_t lowerEliteIndex = population.size();
    if (lowerElite != nullptr) {
        const auto lowerElitePosition = std::find(
            population.begin(),
            population.end(),
            lowerElite);
        if (lowerElitePosition == population.end()) {
            throw std::logic_error("lower elite is not part of the current population");
        }
        lowerEliteIndex = static_cast<size_t>(
            std::distance(population.begin(), lowerElitePosition));
        populationBuffer.push_back(lowerElite);
    }

    size_t reusableIndex = 0;
    auto takeReusableIndividual = [&]() -> shared_ptr<Individual> {
        while (reusableIndex < population.size()
               && reusableIndex == lowerEliteIndex) {
            ++reusableIndex;
        }
        if (reusableIndex >= population.size()) {
            throw std::logic_error("population reuse pool was exhausted");
        }
        return population[reusableIndex++];
    };

    if (retainVerifiedFallback) {
        lowerElite = takeReusableIndividual();
        lowerElite->copy_from(*verifiedBest);
        populationBuffer.push_back(lowerElite);
    }

    for (const auto& chromosome : chromosomes) {
        splitWorkspace.giantTour.clear();
        splitWorkspace.giantTour.push_back(instance->depot);
        splitWorkspace.giantTour.insert(
            splitWorkspace.giantTour.end(),
            chromosome.begin(),
            chromosome.end());

        vector<vector<int>> offspringRoutes = Initializer::split_giant_tour(
            splitWorkspace.giantTour,
            *instance,
            splitWorkspace);

        for (auto& route : offspringRoutes) {
            route.insert(route.begin(), instance->depot);
            route.push_back(instance->depot);
        }

        shared_ptr<Individual> offspring = takeReusableIndividual();
        const double upperCost = instance->fitness_evaluation(offspringRoutes);
        const vector<int> demandSums = instance->compute_demand_sum(offspringRoutes);
        offspring->load_upper_solution(offspringRoutes, upperCost, demandSums);
        populationBuffer.push_back(std::move(offspring));
    }

    if (populationBuffer.size() != static_cast<size_t>(popSize)) {
        throw std::logic_error("rebuilt population has an unexpected size");
    }
    retainedLowerElite = lowerElite;
    population.clear();
    population.swap(populationBuffer);
}
