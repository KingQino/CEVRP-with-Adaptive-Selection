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

MA::MA(Case* instance, int seed, int isMaxEvals, int popSize, double eliteRatio, double immigrantRatio, double crossoverProb,
       double /*mutationProb*/, double mutationIndProb, int tournamentSize) {
    // init parameters
    this->instance = instance;
    this->randomEngine = std::default_random_engine(seed);
    this->seed = seed;
    this->isMaxEvals = isMaxEvals;

    uniform_real_distribution<double> udist(0.0, 1.0);
    this->uniformRealDis = udist;

    // hyperparameters for MA
    this->popSize = popSize;
    this->eliteRatio = eliteRatio;
    this->immigrantRatio = immigrantRatio;
    this->crossoverProb = crossoverProb;
    this->mutationProb = mutationIndProb;
    this->mutationIndProb = mutationIndProb;
    this->tournamentSize = tournamentSize;

    this->routeCapacity = this->instance->vehicleNumber * 3;
    // One upper-level route can contain depot + all customers + depot.
    this->nodeCapacity = this->instance->customerNumber + 2;
    this->generation = 0;
    this->localSearchConfidenceMultiplier = 1.2;
    this->chargingConfidenceMultiplier = 0.8;
    this->lowerLevelTriggerRatio = 1.02;
    this->confidenceWindowSize = 30;
    this->bestObservedChargingPenalty = 0.0;
    this->globalBestUpperCost = DBL_MAX;
}

MA::~MA() {
    generationBestComplete.reset();
    verifiedBest.reset();
    population.clear();
}

void MA::run() {
    if (this->isMaxEvals == 1) {
        start = std::chrono::high_resolution_clock::now();
        end = std::chrono::high_resolution_clock::now();
        duration = end - start;

        open_log_for_evolution();
        initialize_search();
        while (!reached_evaluation_limit()) {
            run_generation();
            duration = std::chrono::high_resolution_clock::now() - start;
            flush_row_into_evol_log();
        }
        close_log_for_evolution();
        save_log_for_solution();
    } else {
        start = std::chrono::high_resolution_clock::now();
        end = std::chrono::high_resolution_clock::now();
        duration = end - start;

        open_log_for_evolution();
        initialize_search();
        while (!reached_time_limit(duration)) {
            run_generation();
            duration = std::chrono::high_resolution_clock::now() - start;
            flush_row_into_evol_log();
        }
        close_log_for_evolution();
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

vector<double> MA::collect_upper_costs(const vector<shared_ptr<Individual>>& group) {
    std::vector<double> ans;
    ans.reserve(group.size());

    std::transform(group.begin(), group.end(), std::back_inserter(ans),
                   [](const auto& ind) { return ind->get_upper_cost(); });

    return ans;
}

vector<double> MA::collect_lower_costs(const vector<shared_ptr<Individual>>& group) {
    std::vector<double> ans;
    ans.reserve(group.size());

    std::transform(group.begin(), group.end(), std::back_inserter(ans),
                   [](const auto& ind) { return ind->get_lower_cost(); });

    return ans;
}

void MA::open_log_for_evolution() {
    string directoryPath = "../" + statsPath + "/" + instance->instanceName + "/" + to_string(seed);
    create_directories_if_not_exists(directoryPath);

    string filename = "evols." + instance->instanceName + ".csv";
    logEvolution.open(directoryPath + "/" + filename);
    logEvolution << "generation,pop_size,"
                    "offspring_size,S_min_upper_cost,S_avg_upper_cost,S_max_upper_cost,S_std_upper_cost,"
                    "upper_pop_size,S1_min_upper_cost,S1_avg_upper_cost,S1_max_upper_cost,S1_std_upper_cost,"
                    "lower_pop_size,S3_min_lower_cost,S3_avg_lower_cost,S3_max_lower_cost,S3_std_lower_cost,S3_infeasible_size,"
                    "evaluations,progress,duration\n";
}

void MA::flush_row_into_evol_log() {
    double evals_used = instance->get_evals();
    double progress = evals_used/instance->maxEvals;
    ossRowEvol << generation << "," << population.size() << ","
               << populationMetrics.size << "," << populationMetrics.min << "," << populationMetrics.average << "," << populationMetrics.max << "," << populationMetrics.standardDeviation << ","
               << upperCandidateMetrics.size << "," << upperCandidateMetrics.min << "," << upperCandidateMetrics.average << "," << upperCandidateMetrics.max << "," << upperCandidateMetrics.standardDeviation << ","
               << followerEvaluatedMetrics.size << "," << followerEvaluatedMetrics.min << "," << followerEvaluatedMetrics.average << "," << followerEvaluatedMetrics.max << "," << followerEvaluatedMetrics.standardDeviation << "," << followerEvaluatedMetrics.infeasibleSize << ","
               << evals_used << "," << progress << "," << duration.count() << "\n";
}

void MA::close_log_for_evolution() {
    logEvolution << ossRowEvol.str();
    ossRowEvol.clear();
    logEvolution.close();
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
    generationBestComplete = make_unique<Individual>(
        routeCapacity,
        nodeCapacity,
        emptyVector2D,
        INFEASIBLE_COST,
        emptyVector1D);
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

    populationMetrics = calculate_population_metrics(collect_upper_costs(population));

    vector<shared_ptr<Individual>> upperCandidates = population;
    double bestCandidateImprovement = 0;
    double improvementThreshold;
    shared_ptr<Individual> bestUpperCandidate = Reproduction::best_by_upper_cost(population);
    if (generation > confidenceWindowSize) {
        const double oldUpperCost = bestUpperCandidate->get_upper_cost();

        Leader::improve_with_three_neighborhood_vnd(*bestUpperCandidate, *instance);

        const double newUpperCost = bestUpperCandidate->get_upper_cost();
        bestCandidateImprovement = oldUpperCost - newUpperCost;
        improvementThreshold = *std::max_element(
            recentUpperImprovements.begin(),
            recentUpperImprovements.end());
        if (improvementThreshold < bestCandidateImprovement) {
            improvementThreshold = bestCandidateImprovement * localSearchConfidenceMultiplier;
        }

        upperCandidates.clear();
        for (auto& individual : population) {
            if (individual->get_upper_cost() - improvementThreshold <= newUpperCost) {
                upperCandidates.push_back(individual);
            }
        }

        auto bestCandidateIt = std::find(
            upperCandidates.begin(),
            upperCandidates.end(),
            bestUpperCandidate);
        if (bestCandidateIt != upperCandidates.end()) {
            upperCandidates.erase(bestCandidateIt);
        }
    }

    double maximumUpperImprovement = 0;
    for (auto& individual : upperCandidates) {
        const double oldUpperCost = individual->get_upper_cost();
        Leader::improve_with_three_neighborhood_vnd(*individual, *instance);
        if (maximumUpperImprovement < oldUpperCost - individual->get_upper_cost()) {
            maximumUpperImprovement = oldUpperCost - individual->get_upper_cost();
        }
    }
    maximumUpperImprovement = std::max(bestCandidateImprovement, maximumUpperImprovement);
    recentUpperImprovements.push_back(maximumUpperImprovement);
    if (recentUpperImprovements.size() > static_cast<size_t>(confidenceWindowSize)) {
        recentUpperImprovements.pop_front();
    }
    if (generation > confidenceWindowSize) {
        upperCandidates.push_back(bestUpperCandidate);
    }

    upperCandidateMetrics = calculate_population_metrics(
        collect_upper_costs(upperCandidates));

    // Build the quality-diversity parent pool before follower evaluation, so
    // reproduction remains independent from lower-level triggering.
    vector<shared_ptr<Individual>> rankedUpperSolutions = upperCandidates;
    sort(rankedUpperSolutions.begin(), rankedUpperSolutions.end(), [](const shared_ptr<Individual>& lhs, const shared_ptr<Individual>& rhs) {
        return lhs->get_upper_cost() < rhs->get_upper_cost();
    });
    const size_t targetParentPoolSize = std::max<size_t>(2, (static_cast<size_t>(popSize) + 9) / 10);
    vector<ParentCandidate> parentPool = Reproduction::build_quality_diversity_parent_pool(
        rankedUpperSolutions,
        targetParentPoolSize);

    vector<shared_ptr<Individual>> followerCandidates;
    double minimumChargingPenalty;
    shared_ptr<Individual> generationBestUpper = Reproduction::best_by_upper_cost(upperCandidates);
    if (generationBestUpper->get_upper_cost() < globalBestUpperCost) {
        globalBestUpperCost = generationBestUpper->get_upper_cost();
    }

    const double triggerUpperBound = globalBestUpperCost * lowerLevelTriggerRatio;
    for (auto& individual : upperCandidates) {
        if (individual->get_upper_cost() <= triggerUpperBound) {
            followerCandidates.push_back(individual);
        }
    }

    auto generationBestIt = std::find(
        followerCandidates.begin(),
        followerCandidates.end(),
        generationBestUpper);
    // The generation best is always evaluated below, so avoid evaluating it twice.
    if (generationBestIt != followerCandidates.end()) {
        followerCandidates.erase(generationBestIt);
    }

    vector<shared_ptr<Individual>> evaluatedCompleteSolutions;
    evaluatedCompleteSolutions.push_back(generationBestUpper);
    const double generationBestUpperCost = generationBestUpper->get_upper_cost();
    Follower::optimize_charging(*generationBestUpper, *instance);
    minimumChargingPenalty = generationBestUpper->get_lower_cost() - generationBestUpperCost;
    for (auto& individual : followerCandidates) {
        const double oldUpperCost = individual->get_upper_cost();
        Follower::optimize_charging(*individual, *instance);
        const double newLowerCost = individual->get_lower_cost();
        evaluatedCompleteSolutions.push_back(individual);
        if (minimumChargingPenalty > newLowerCost - oldUpperCost) {
            minimumChargingPenalty = newLowerCost - oldUpperCost;
        }
    }
    if (bestObservedChargingPenalty == 0
        || bestObservedChargingPenalty > minimumChargingPenalty) {
        bestObservedChargingPenalty = minimumChargingPenalty;
    }

    followerEvaluatedMetrics = calculate_population_metrics(
        collect_lower_costs(evaluatedCompleteSolutions));

    generationBestComplete = make_unique<Individual>(
        *Reproduction::best_by_lower_cost(evaluatedCompleteSolutions));
    if (verifiedBest->get_lower_cost() > generationBestComplete->get_lower_cost()) {
        verifiedBest = make_unique<Individual>(*generationBestComplete);
    }


    const int offspringTarget = popSize - 1;
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

    // destroy all the individual objects
    evaluatedCompleteSolutions.clear();
    followerCandidates.clear();
    upperCandidates.clear();
    population.clear();
    population.shrink_to_fit();


    // update population
    population.reserve(popSize);
    population.push_back(make_shared<Individual>(*generationBestComplete));
    for (int i = 0; i < popSize - 1; ++i) {
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
