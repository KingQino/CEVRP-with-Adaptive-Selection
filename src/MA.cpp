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
#include <cmath>
#include <filesystem>
#include <stdexcept>

namespace {

struct LocalSearchLogRecord {
    std::shared_ptr<Individual> individual;
    double qualityGap{};
    double distanceBefore{};
    double distanceAfter{};
    LocalSearchResult result;
    LocalSearchIntensityDecision learningDecision;
    bool hasLearningDecision{};
    bool crossedGamma{};
    bool lowerEvaluated{};
    double verifiedLowerImprovement{};
};

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
    std::seed_seq localSearchLearningSeed{
        parameters.seed,
        0x4C53,
        0x4C454152};
    this->localSearchLearningEngine.seed(localSearchLearningSeed);
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
    this->enableLocalSearchLearning =
        parameters.enableLocalSearchLearning;
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
}

MA::~MA() {
    verifiedBest.reset();
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
        const shared_ptr<Individual> bestUpperCandidate =
            Reproduction::best_by_upper_cost(population);
        if (bestUpperCandidate != nullptr) {
            if (verifiedBest == nullptr) {
                verifiedBest = make_unique<Individual>(*bestUpperCandidate);
            } else {
                verifiedBest->copy_from(*bestUpperCandidate);
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

    logLocalSearch.open(directoryPath / "local-search.tsv");
    logLocalSearch << LOCAL_SEARCH_LOG_HEADER << "\n";
    logLocalSearchOperators.open(
        directoryPath / "local-search-operators.tsv");
    logLocalSearchOperators << LOCAL_SEARCH_OPERATOR_LOG_HEADER << "\n";
    if (enableLocalSearchLearning) {
        logLocalSearchLearning.open(
            directoryPath / "local-search-learning.tsv");
        logLocalSearchLearning
            << LOCAL_SEARCH_LEARNING_LOG_HEADER << "\n";
    }
}

void MA::flush_local_search_log() {
    if (logLocalSearch.is_open()) {
        logLocalSearch << localSearchRows.str();
        localSearchRows.str("");
        localSearchRows.clear();
    }
    if (logLocalSearchOperators.is_open()) {
        logLocalSearchOperators << localSearchOperatorRows.str();
        localSearchOperatorRows.str("");
        localSearchOperatorRows.clear();
    }
    if (logLocalSearchLearning.is_open()) {
        logLocalSearchLearning << localSearchLearningRows.str();
        localSearchLearningRows.str("");
        localSearchLearningRows.clear();
    }
}

void MA::close_log_for_local_search() {
    flush_local_search_log();
    logLocalSearch.close();
    logLocalSearchOperators.close();
    logLocalSearchLearning.close();
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
    localSearchIntensityLearner.reset();
    retainedLowerElite.reset();
    population.clear();
    populationBuffer.clear();
    population.reserve(static_cast<size_t>(popSize));
    populationBuffer.reserve(static_cast<size_t>(popSize));
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
        globalBestUpperCost = Reproduction::best_by_upper_cost(population)->get_upper_cost();
    } else {
        globalBestUpperCost = DBL_MAX;
    }
}

void MA::run_generation() {
    generation++;

    shared_ptr<Individual> unchangedLowerElite = retainedLowerElite;
    vector<LocalSearchLogRecord> localSearchRecords;
    std::array<
        LocalSearchOperatorStats,
        LOCAL_SEARCH_OPERATOR_COUNT> generationOperatorStats{};
    shared_ptr<Individual> bestUpperCandidate = Reproduction::best_by_upper_cost(population);
    const bool needsLocalSearchContext =
        enableLogging || enableLocalSearchLearning;
    ParentCandidate localSearchBestReference;
    if (needsLocalSearchContext) {
        localSearchBestReference =
            Reproduction::make_parent_candidate(*bestUpperCandidate);
    }
    double localSearchBestCost = std::min(
        globalBestUpperCost,
        bestUpperCandidate->get_upper_cost());

    auto applyLocalSearch = [&](const shared_ptr<Individual>& individual) {
        const bool canReuseLowerElite = individual == unchangedLowerElite
            && individual->is_upper_locally_optimal()
            && individual->get_lower_cost() < INFEASIBLE_COST;
        if (!needsLocalSearchContext) {
            if (!canReuseLowerElite) {
                Leader::improve_with_eight_neighborhood_rvnd_one_move(
                    *individual,
                    *instance,
                    localSearchEngine,
                    localSearchIntensity,
                    localSearchWorkspace);
            }
            return;
        }

        const ParentCandidate beforeCandidate =
            Reproduction::make_parent_candidate(*individual);
        const double referenceCost = localSearchBestReference.upperCost;
        const double qualityGap = referenceCost > 0.0
            ? (beforeCandidate.upperCost - referenceCost) / referenceCost
            : 0.0;
        const double distanceBefore = Reproduction::adjacency_distance(
            beforeCandidate,
            localSearchBestReference);
        const double triggerUpperBoundBefore =
            localSearchBestCost * lowerLevelTriggerRatio;
        const bool outsideGammaBefore =
            beforeCandidate.upperCost > triggerUpperBoundBefore;

        LocalSearchIntensity selectedIntensity =
            localSearchIntensity;
        LocalSearchIntensityDecision learningDecision;
        bool hasLearningDecision = false;
        if (enableLocalSearchLearning && !canReuseLowerElite) {
            const double budgetProgress =
                instance->get_evaluation_limit_distance_calls() > 0
                ? static_cast<double>(
                    instance->get_distance_calls())
                    / static_cast<double>(
                        instance->get_evaluation_limit_distance_calls())
                : 0.0;
            const double gammaMargin =
                triggerUpperBoundBefore > 0.0
                ? (beforeCandidate.upperCost
                   - triggerUpperBoundBefore)
                    / triggerUpperBoundBefore
                : 0.0;
            learningDecision =
                localSearchIntensityLearner.select(
                    {
                        qualityGap,
                        distanceBefore,
                        budgetProgress,
                        gammaMargin,
                    },
                    localSearchLearningEngine);
            selectedIntensity = learningDecision.intensity;
            hasLearningDecision = true;
        }

        LocalSearchResult result;
        if (canReuseLowerElite) {
            result.moveLimit = -1;
            result.reachedLocalOptimum = true;
        } else {
            result = Leader::improve_with_eight_neighborhood_rvnd_one_move(
                *individual,
                *instance,
                localSearchEngine,
                selectedIntensity,
                localSearchWorkspace,
                triggerUpperBoundBefore);
        }
        for (std::size_t operatorIndex = 0;
             operatorIndex < LOCAL_SEARCH_OPERATOR_COUNT;
             ++operatorIndex) {
            auto& generationStats =
                generationOperatorStats[operatorIndex];
            const auto& individualStats =
                result.operatorStats[operatorIndex];
            generationStats.calls += individualStats.calls;
            generationStats.accepts += individualStats.accepts;
            generationStats.distanceCalls +=
                individualStats.distanceCalls;
            generationStats.upperGain += individualStats.upperGain;
            generationStats.gammaCrosses +=
                individualStats.gammaCrosses;
        }
        const ParentCandidate afterCandidate =
            Reproduction::make_parent_candidate(*individual);

        LocalSearchLogRecord record;
        record.individual = individual;
        record.qualityGap = qualityGap;
        record.distanceBefore = distanceBefore;
        record.distanceAfter = Reproduction::adjacency_distance(
            afterCandidate,
            localSearchBestReference);
        record.result = result;
        record.learningDecision = learningDecision;
        record.hasLearningDecision = hasLearningDecision;
        record.crossedGamma = outsideGammaBefore
            && afterCandidate.upperCost <= triggerUpperBoundBefore;
        localSearchRecords.push_back(std::move(record));

        if (afterCandidate.upperCost < localSearchBestReference.upperCost) {
            localSearchBestReference = afterCandidate;
        }
        if (afterCandidate.upperCost < localSearchBestCost) {
            localSearchBestCost = afterCandidate.upperCost;
        }
    };

    for (auto& individual : population) {
        applyLocalSearch(individual);
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
        globalBestUpperCost = generationBestUpper->get_upper_cost();
    }

    const double triggerUpperBound = globalBestUpperCost * lowerLevelTriggerRatio;
    for (auto& individual : population) {
        if (individual->get_upper_cost() <= triggerUpperBound) {
            followerCandidates.push_back(individual);
        }
    }

    vector<shared_ptr<Individual>> evaluatedCompleteSolutions;
    vector<shared_ptr<Individual>> followerEvaluatedSolutions;
    const double verifiedLowerCostBefore = verifiedBest->get_lower_cost();
    for (auto& individual : followerCandidates) {
        const bool canReuseLowerElite = individual == unchangedLowerElite
            && individual->get_lower_cost() < INFEASIBLE_COST;
        if (!canReuseLowerElite) {
            Follower::optimize_charging(
                *individual,
                *instance,
                followerWorkspace);
            if (enableLogging) {
                followerEvaluatedSolutions.push_back(individual);
            }
        }
        evaluatedCompleteSolutions.push_back(individual);
    }
    if (enableLogging) {
        for (auto& record : localSearchRecords) {
            record.lowerEvaluated = std::find(
                followerEvaluatedSolutions.begin(),
                followerEvaluatedSolutions.end(),
                record.individual) != followerEvaluatedSolutions.end();
        }
    }

    shared_ptr<Individual> lowerElite;
    if (!evaluatedCompleteSolutions.empty()) {
        const shared_ptr<Individual> bestEvaluatedComplete =
            Reproduction::best_by_lower_cost(evaluatedCompleteSolutions);
        if (bestEvaluatedComplete->get_lower_cost() < INFEASIBLE_COST) {
            lowerElite = bestEvaluatedComplete;
        }
        if (verifiedBest->get_lower_cost() > bestEvaluatedComplete->get_lower_cost()) {
            if (needsLocalSearchContext
                && verifiedLowerCostBefore < INFEASIBLE_COST) {
                for (auto& record : localSearchRecords) {
                    if (record.individual == bestEvaluatedComplete) {
                        record.verifiedLowerImprovement =
                            verifiedLowerCostBefore - bestEvaluatedComplete->get_lower_cost();
                        break;
                    }
                }
            }
            verifiedBest->copy_from(*bestEvaluatedComplete);
        }
    }
    const bool retainVerifiedFallback =
        lowerElite == nullptr && verifiedBest->get_lower_cost() < INFEASIBLE_COST;

    vector<LocalSearchLearningOutcome> learningOutcomes;
    if (enableLocalSearchLearning) {
        learningOutcomes.reserve(localSearchRecords.size());
        for (const auto& record : localSearchRecords) {
            if (!record.hasLearningDecision) {
                continue;
            }
            LocalSearchLearningOutcome outcome;
            outcome.decision = record.learningDecision;
            outcome.relativeUpperImprovement =
                record.result.relativeUpperImprovement;
            outcome.distanceCalls =
                record.result.distanceCallsUsed;
            outcome.evaluationLimitDistanceCalls =
                instance->get_evaluation_limit_distance_calls();
            outcome.crossedGamma = record.crossedGamma;
            if (verifiedLowerCostBefore < INFEASIBLE_COST
                && verifiedLowerCostBefore > 0.0
                && record.verifiedLowerImprovement > 0.0) {
                outcome.relativeVerifiedLowerImprovement =
                    record.verifiedLowerImprovement
                    / verifiedLowerCostBefore;
            }
            learningOutcomes.push_back(std::move(outcome));
        }
    }

    if (enableLogging) {
        for (const auto& record : localSearchRecords) {
            localSearchRows << setprecision(12)
                            << generation << "\t"
                            << record.qualityGap << "\t"
                            << record.distanceBefore << "\t"
                            << record.distanceAfter << "\t"
                            << record.result.moveLimit << "\t"
                            << record.result.acceptedMoves << "\t"
                            << record.result.neighborhoodCalls << "\t"
                            << instance->distance_calls_to_evals(
                                record.result.distanceCallsUsed) << "\t"
                            << record.result.relativeUpperImprovement << "\t"
                            << record.result.reachedLocalOptimum << "\t"
                            << record.crossedGamma << "\t"
                            << record.lowerEvaluated << "\t"
                            << record.verifiedLowerImprovement << "\n";
        }
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
        for (const auto& outcome : learningOutcomes) {
            const double benefit =
                LocalSearchIntensityLearner::observed_benefit(
                    outcome);
            const double cost =
                LocalSearchIntensityLearner::observed_cost(
                    outcome);
            localSearchLearningRows
                << setprecision(12)
                << generation << "\t"
                << outcome.decision.context.qualityGap << "\t"
                << outcome.decision.context.adjacencyDistance << "\t"
                << outcome.decision.context.budgetProgress << "\t"
                << outcome.decision.context.gammaMargin << "\t"
                << LocalSearchIntensityLearner::intensity_name(
                    outcome.decision.intensity) << "\t"
                << instance->distance_calls_to_evals(
                    outcome.distanceCalls) << "\t"
                << benefit << "\t"
                << outcome.decision.budgetPrice << "\t"
                << benefit
                    - outcome.decision.budgetPrice * cost
                << "\n";
        }
        flush_local_search_log();
    }

    if (enableLocalSearchLearning) {
        localSearchIntensityLearner.update_batch(
            learningOutcomes);
    }

    const bool hasLowerElite = lowerElite != nullptr || retainVerifiedFallback;
    const int offspringTarget = popSize - (hasLowerElite ? 1 : 0);
    const bool hasVerifiedBest = verifiedBest->get_lower_cost() < INFEASIBLE_COST;
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
        reproductionWorkspace);

    // All information needed from the old population is now materialized.
    evaluatedCompleteSolutions.clear();
    followerEvaluatedSolutions.clear();
    followerCandidates.clear();
    rankedUpperSolutions.clear();
    localSearchRecords.clear();
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
