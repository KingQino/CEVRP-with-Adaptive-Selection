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
            flush_row_into_evol_log();
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
}

void MA::close_log_for_local_search() {
    flush_local_search_log();
    logLocalSearchOperators.close();
    logLocalSearchAllocation.close();
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
            localSearchAllocator);
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
        for (std::size_t operatorIndex = 0;
             operatorIndex < LOCAL_SEARCH_OPERATOR_COUNT;
             ++operatorIndex) {
            const auto localSearchOperator =
                static_cast<LocalSearchOperator>(operatorIndex);
            const auto& operatorStats =
                generationOperatorStats[operatorIndex];
            localSearchOperatorRows
                << setprecision(12)
                << generation << "\t"
                << Leader::operator_name(localSearchOperator) << "\t"
                << operatorStats.calls << "\t"
                << operatorStats.accepts << "\t"
                << instance->distance_calls_to_evals(
                    operatorStats.distanceCalls) << "\t"
                << operatorStats.upperGain << "\t"
                << operatorStats.gammaCrosses << "\n";
        }
        if (localSearchPolicy != LocalSearchPolicy::Static) {
            const std::array<LocalSearchIntensity, 3> intensities = {
                    LocalSearchIntensity::Weak,
                    LocalSearchIntensity::Medium,
                    localSearchIntensity,
                };
            for (const LocalSearchIntensity intensity
                 : intensities) {
                const auto& stats = mixedLocalSearch.stats[
                    LocalSearchAllocationRunner::intensity_index(
                        intensity)];
                const double selectionCount =
                    static_cast<double>(stats.selections);
                localSearchAllocationRows
                    << setprecision(12)
                    << generation << "\t"
                    << local_search_policy_name(
                        localSearchPolicy) << "\t"
                    << LocalSearchAllocationRunner::intensity_name(
                        intensity) << "\t"
                    << stats.selections << "\t"
                    << stats.forcedLocalOptima << "\t"
                    << stats.exploratorySelections << "\t"
                    << stats.acceptedMoves << "\t"
                    << stats.neighborhoodCalls << "\t"
                    << instance->distance_calls_to_evals(
                        stats.distanceCalls) << "\t"
                    << stats.upperGain << "\t"
                    << stats.gammaCrosses << "\t"
                    << stats.parentUses << "\t"
                    << stats.lowerArchiveEntries << "\t"
                    << stats.utility << "\t"
                    << (stats.selections > 0
                        ? stats.costUnits / selectionCount
                        : 0.0) << "\t"
                    << (stats.selections > 0
                        ? stats.selectionScore / selectionCount
                        : 0.0) << "\n";
            }
        }
        flush_local_search_log();
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
