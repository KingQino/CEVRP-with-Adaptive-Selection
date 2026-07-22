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
        throw std::runtime_error("search finished without a feasible complete solution");
    }
    Follower::refine_charging_by_enumeration(*verifiedBest, *instance);
    if (enableLogging) {
        save_log_for_solution();
    }
}

// stop criterion: max evals
bool MA::reached_evaluation_limit() const {
    bool flag;
    if (instance->get_evals() >= instance->maxEvals)
        flag = true;
    else
        flag = false;

    return flag;
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
}

void MA::flush_local_search_log() {
    if (!logLocalSearch.is_open()) {
        return;
    }
    logLocalSearch << localSearchRows.str();
    localSearchRows.str("");
    localSearchRows.clear();
}

void MA::close_log_for_local_search() {
    flush_local_search_log();
    logLocalSearch.close();
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
    retainedLowerElite.reset();
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

    const shared_ptr<Individual> unchangedLowerElite = retainedLowerElite;
    vector<LocalSearchLogRecord> localSearchRecords;
    shared_ptr<Individual> bestUpperCandidate = Reproduction::best_by_upper_cost(population);
    ParentCandidate localSearchBestReference;
    if (enableLogging) {
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
        if (!enableLogging) {
            if (!canReuseLowerElite) {
                Leader::improve_with_seven_neighborhood_rvnd_one_move(
                    *individual,
                    *instance,
                    localSearchEngine,
                    localSearchIntensity);
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

        LocalSearchResult result;
        if (canReuseLowerElite) {
            result.moveLimit = -1;
            result.reachedLocalOptimum = true;
        } else {
            result = Leader::improve_with_seven_neighborhood_rvnd_one_move(
                *individual,
                *instance,
                localSearchEngine,
                localSearchIntensity);
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
            Follower::optimize_charging(*individual, *instance);
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
            lowerElite = make_shared<Individual>(*bestEvaluatedComplete);
        }
        if (verifiedBest->get_lower_cost() > bestEvaluatedComplete->get_lower_cost()) {
            if (enableLogging && verifiedLowerCostBefore < INFEASIBLE_COST) {
                for (auto& record : localSearchRecords) {
                    if (record.individual == bestEvaluatedComplete) {
                        record.verifiedLowerImprovement =
                            verifiedLowerCostBefore - bestEvaluatedComplete->get_lower_cost();
                        break;
                    }
                }
            }
            verifiedBest = make_unique<Individual>(*bestEvaluatedComplete);
        }
    }
    if (lowerElite == nullptr && verifiedBest->get_lower_cost() < INFEASIBLE_COST) {
        lowerElite = make_shared<Individual>(*verifiedBest);
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
                            << record.result.evalsUsed << "\t"
                            << record.result.relativeUpperImprovement << "\t"
                            << record.result.reachedLocalOptimum << "\t"
                            << record.crossedGamma << "\t"
                            << record.lowerEvaluated << "\t"
                            << record.verifiedLowerImprovement << "\n";
        }
        flush_local_search_log();
    }


    const int offspringTarget = popSize - (lowerElite == nullptr ? 0 : 1);
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
        uniformRealDis);

    // Release the old population vector capacity before rebuilding it.
    evaluatedCompleteSolutions.clear();
    followerEvaluatedSolutions.clear();
    followerCandidates.clear();
    retainedLowerElite = lowerElite;
    population.clear();
    population.shrink_to_fit();


    // update population
    population.reserve(static_cast<size_t>(popSize));
    if (lowerElite != nullptr) {
        population.push_back(std::move(lowerElite));
    }
    for (const auto& chromosome : chromosomes) {
        vector<int> giantTour = {instance->depot};
        giantTour.insert(giantTour.end(), chromosome.begin(), chromosome.end());

        vector<vector<int>> offspringRoutes = Initializer::split_giant_tour(giantTour, *instance);

        for (auto& route : offspringRoutes) {
            route.insert(route.begin(), instance->depot);
            route.push_back(instance->depot);
        }

        population.push_back(make_shared<Individual>(routeCapacity, nodeCapacity, offspringRoutes,
                                                     instance->fitness_evaluation(offspringRoutes),
                                                     instance->compute_demand_sum(offspringRoutes))
                                                     );
    }
}
