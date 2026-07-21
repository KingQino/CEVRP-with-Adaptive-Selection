//
// Created by Yinghao Qin on 19/12/2023.
//

#include "../include/MA.hpp"

#include "../include/algorithm_constants.hpp"
#include "../include/follower.hpp"
#include "../include/initializer.hpp"
#include "../include/leader.hpp"
#include "../include/reproduction.hpp"

#include <cfloat>

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

MA::MA(Case* instance, int seed, int isMaxEvals, int popSize, double immigrantRatio, double crossoverProb,
       double /*mutationProb*/, double mutationIndProb, int tournamentSize,
       LocalSearchIntensity localSearchIntensity) {
    // init parameters
    this->instance = instance;
    this->randomEngine = std::default_random_engine(seed);
    std::seed_seq localSearchSeed{seed, 0x4C53, 0x52564E44};
    this->localSearchEngine.seed(localSearchSeed);
    this->seed = seed;
    this->isMaxEvals = isMaxEvals;

    uniform_real_distribution<double> udist(0.0, 1.0);
    this->uniformRealDis = udist;

    // hyperparameters for MA
    this->popSize = popSize;
    this->immigrantRatio = immigrantRatio;
    this->crossoverProb = crossoverProb;
    this->mutationProb = mutationIndProb;
    this->mutationIndProb = mutationIndProb;
    this->tournamentSize = tournamentSize;
    this->localSearchIntensity = localSearchIntensity;

    this->routeCapacity = this->instance->vehicleNumber * 3;
    // One upper-level route can contain depot + all customers + depot.
    this->nodeCapacity = this->instance->customerNumber + 2;
    this->generation = 0;
    this->lowerLevelTriggerRatio = 1.02;
    this->globalBestUpperCost = DBL_MAX;
}

MA::~MA() {
    verifiedBest.reset();
    population.clear();
}

void MA::run() {
    if (this->isMaxEvals == 1) {
        start = std::chrono::high_resolution_clock::now();
        end = std::chrono::high_resolution_clock::now();
        duration = end - start;

        open_log_for_evolution();
        open_log_for_local_search();
        initialize_search();
        while (!reached_evaluation_limit()) {
            run_generation();
            duration = std::chrono::high_resolution_clock::now() - start;
            flush_row_into_evol_log();
        }
        close_log_for_evolution();
        close_log_for_local_search();
        save_log_for_solution();
    } else {
        start = std::chrono::high_resolution_clock::now();
        end = std::chrono::high_resolution_clock::now();
        duration = end - start;

        open_log_for_evolution();
        open_log_for_local_search();
        initialize_search();
        while (!reached_time_limit(duration)) {
            run_generation();
            duration = std::chrono::high_resolution_clock::now() - start;
            flush_row_into_evol_log();
        }
        close_log_for_evolution();
        close_log_for_local_search();
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
    string directoryPath = "../" + statsPath + "/" + instance->instanceName + "/" + to_string(seed);
    create_directories_if_not_exists(directoryPath);

    string filename = "evols." + instance->instanceName + ".csv";
    logEvolution.open(directoryPath + "/" + filename);
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
    const string directoryPath =
        "../" + statsPath + "/" + instance->instanceName + "/" + to_string(seed);
    create_directories_if_not_exists(directoryPath);

    logLocalSearch.open(directoryPath + "/local-search.tsv");
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
    Follower::refine_charging_by_enumeration(*verifiedBest, *instance);

    string directoryPath = "../" + statsPath + "/" + instance->instanceName + "/" + to_string(seed);
    create_directories_if_not_exists(directoryPath);
    string filename = "solution." + instance->instanceName + ".txt";

    logSolution.open(directoryPath + "/" + filename);
    logSolution << fixed << setprecision(5) << verifiedBest->get_lower_cost() << endl;
    pair<int*, int> tourInfo = verifiedBest->get_tour();
    for (int i = 0; i < tourInfo.second; ++i) {
        logSolution << tourInfo.first[i] << ",";
    }
    logSolution << endl;
    logSolution.close();
}

void MA::initialize_search() {
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

    vector<LocalSearchLogRecord> localSearchRecords;
    shared_ptr<Individual> bestUpperCandidate = Reproduction::best_by_upper_cost(population);
    ParentCandidate localSearchBestReference =
        Reproduction::make_parent_candidate(*bestUpperCandidate);
    double localSearchBestCost = std::min(
        globalBestUpperCost,
        localSearchBestReference.upperCost);

    auto applyLocalSearch = [&](const shared_ptr<Individual>& individual) {
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

        const LocalSearchResult result =
            Leader::improve_with_seven_neighborhood_rvnd_one_move(
                *individual,
                *instance,
                localSearchEngine,
                localSearchIntensity);
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
    const size_t targetParentPoolSize = std::max<size_t>(2, (static_cast<size_t>(popSize) + 9) / 10);
    vector<ParentCandidate> parentPool = Reproduction::build_quality_diversity_parent_pool(
        rankedUpperSolutions,
        targetParentPoolSize);

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
    const double verifiedLowerCostBefore = verifiedBest->get_lower_cost();
    for (auto& individual : followerCandidates) {
        Follower::optimize_charging(*individual, *instance);
        evaluatedCompleteSolutions.push_back(individual);
    }
    for (auto& record : localSearchRecords) {
        record.lowerEvaluated = std::find(
            evaluatedCompleteSolutions.begin(),
            evaluatedCompleteSolutions.end(),
            record.individual) != evaluatedCompleteSolutions.end();
    }

    if (!evaluatedCompleteSolutions.empty()) {
        const shared_ptr<Individual> bestEvaluatedComplete =
            Reproduction::best_by_lower_cost(evaluatedCompleteSolutions);
        if (verifiedBest->get_lower_cost() > bestEvaluatedComplete->get_lower_cost()) {
            if (verifiedLowerCostBefore < INFEASIBLE_COST) {
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


    const int offspringTarget = popSize;
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
        randomEngine,
        uniformRealDis);

    // Release the old population vector capacity before rebuilding it.
    evaluatedCompleteSolutions.clear();
    followerCandidates.clear();
    population.clear();
    population.shrink_to_fit();


    // update population
    population.reserve(popSize);
    for (int i = 0; i < popSize; ++i) {
        vector<int> giantTour = {instance->depot};
        giantTour.insert(giantTour.end(), chromosomes[i].begin(), chromosomes[i].end());

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
