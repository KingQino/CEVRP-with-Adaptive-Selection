//
// Created by Yinghao Qin on 19/12/2023.
//

#include "../include/MA.hpp"

#include <cmath>
#include <cstdint>
#include <cfloat>

namespace {
using AdjacencySignature = std::vector<std::uint64_t>;

struct ParentCandidate {
    std::vector<int> chromosome;
    double upperCost;
    AdjacencySignature signature;
};

std::uint64_t encode_undirected_edge(int u, int v) {
    if (u > v) {
        std::swap(u, v);
    }
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(u)) << 32)
         | static_cast<std::uint32_t>(v);
}

AdjacencySignature build_adjacency_signature(const std::vector<int>& chromosome) {
    AdjacencySignature signature;
    if (chromosome.size() < 2) {
        return signature;
    }

    signature.reserve(chromosome.size() - 1);
    for (size_t i = 1; i < chromosome.size(); ++i) {
        signature.push_back(encode_undirected_edge(chromosome[i - 1], chromosome[i]));
    }
    std::sort(signature.begin(), signature.end());
    return signature;
}

double adjacency_similarity(const AdjacencySignature& lhs, const AdjacencySignature& rhs, size_t chromosomeSize) {
    if (chromosomeSize <= 1) {
        return 1.0;
    }

    size_t commonAdjacencies = 0;
    size_t i = 0;
    size_t j = 0;
    while (i < lhs.size() && j < rhs.size()) {
        if (lhs[i] == rhs[j]) {
            ++commonAdjacencies;
            ++i;
            ++j;
        } else if (lhs[i] < rhs[j]) {
            ++i;
        } else {
            ++j;
        }
    }

    return static_cast<double>(commonAdjacencies) / static_cast<double>(chromosomeSize - 1);
}

double adjacency_distance(const ParentCandidate& lhs, const ParentCandidate& rhs) {
    return 1.0 - adjacency_similarity(lhs.signature, rhs.signature, lhs.chromosome.size());
}

std::vector<int> make_random_immigrant(const std::vector<int>& customers, std::default_random_engine& rng) {
    std::vector<int> immigrant(customers);
    std::shuffle(immigrant.begin(), immigrant.end(), rng);
    return immigrant;
}
}

MA::MA(Case* instance, int seed, int isMaxEvals, int popSize, double eliteRatio, double immigrantRatio, double crossoverProb,
       double mutationProb, double mutationIndProb, int tournamentSize) {
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
    this->gen = 0;
    this->gammaL = 1.2;
    this->gammaR = 0.8;
    this->gammaTrigger = 1.02;
    this->delta = 30;
    this->r = 0.0;
    this->globalBestUpper = DBL_MAX;
}

MA::~MA() {
    iterBest.reset();
    globalBest.reset();
    population.clear();
}

void MA::run() {
    if (this->isMaxEvals == 1) {
        start = std::chrono::high_resolution_clock::now();
        end = std::chrono::high_resolution_clock::now();
        duration = end - start;

        open_log_for_evolution();
        initialize_heuristic();
        while (!termination_criteria_1()) {
            //Execute your heuristic
            run_heuristic();
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
        initialize_heuristic();
        while (!termination_criteria_2(duration)) {
            //Execute your heuristic
            run_heuristic();
            duration = std::chrono::high_resolution_clock::now() - start;
            flush_row_into_evol_log();
        }
        close_log_for_evolution();
        save_log_for_solution();
    }
}

// stop criterion: max evals
bool MA::termination_criteria_1() const {
    bool flag;
    if (instance->get_evals() >= instance->maxEvals)
        flag = true;
    else
        flag = false;

    return flag;
}

// stop criterion: max execute time
bool MA::termination_criteria_2(const std::chrono::duration<double>& runningTime) const {
    bool flag;
    if (runningTime.count() >= instance->maxExecTime)
        flag = true;
    else
        flag = false;

    return flag;
}

void MA::pop_init_with_clustering() {
    for (int i = 0; i < popSize; ++i) {
        vector<vector<int>> routes = routes_constructor_with_hien_method(*instance, randomEngine);
        population.push_back(std::make_shared<Individual>(routeCapacity, nodeCapacity, routes,
                                                          instance->fitness_evaluation(routes),
                                                          instance->compute_demand_sum(routes)));
    }
}

void MA::pop_init_with_order_split() {
    for (int i = 0; i < popSize; ++i) {
        vector<vector<int>> routes = routes_constructor_with_split(*instance, randomEngine);
        population.push_back(std::make_shared<Individual>(routeCapacity, nodeCapacity, routes,
                                                          instance->fitness_evaluation(routes),
                                                          instance->compute_demand_sum(routes)));
    }
}

void MA::pop_init_with_direct_encoding() {
    for (int i = 0; i < popSize; ++i) {
        vector<vector<int>> routes = routes_construct_with_direct_encoding(*instance, randomEngine);
        population.push_back(std::make_shared<Individual>(routeCapacity, nodeCapacity, routes,
                                                          instance->fitness_evaluation(routes),
                                                          instance->compute_demand_sum(routes)));
    }
}

vector<double> MA::get_upper_cost_vector_from_group(const vector<shared_ptr<Individual>>& group) {
    std::vector<double> ans;
    ans.reserve(group.size());

    std::transform(group.begin(), group.end(), std::back_inserter(ans),
                   [](const auto& ind) { return ind->get_upper_cost(); });

    return ans;
}

vector<double> MA::get_lower_cost_vector_from_group(const vector<shared_ptr<Individual>>& group) {
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
    ossRowEvol << gen << "," << population.size() << ","
               << S_stats.size << "," << S_stats.min << "," << S_stats.avg << "," << S_stats.max << "," << S_stats.std << ","
               << S1_stats.size << "," << S1_stats.min << "," << S1_stats.avg << "," << S1_stats.max << "," << S1_stats.std << ","
               << S3_stats.size << "," << S3_stats.min << "," << S3_stats.avg << "," << S3_stats.max << "," << S3_stats.std << "," << S3_stats.dumbSize << ","
               << evals_used << "," << progress << "," << duration.count() << "\n";
}

void MA::close_log_for_evolution() {
    logEvolution << ossRowEvol.str();
    ossRowEvol.clear();
    logEvolution.close();
}

void MA::save_log_for_solution() {
    refine_one_solution_by_all_enumeration(*globalBest, *instance);

    string directoryPath = "../" + statsPath + "/" + instance->instanceName + "/" + to_string(seed);
    create_directories_if_not_exists(directoryPath);
    string filename = "solution." + instance->instanceName + ".txt";

    logSolution.open(directoryPath + "/" + filename);
    logSolution << fixed << setprecision(5) << globalBest->get_lower_cost() << endl;
    pair<int*, int> tourInfo = globalBest->get_tour();
    for (int i = 0; i < tourInfo.second; ++i) {
        logSolution << tourInfo.first[i] << ",";
    }
    logSolution << endl;
    logSolution.close();
}

void MA::initialize_heuristic() {
    pop_init_with_clustering();
    std::vector<std::vector<int>> emptyVector2D;
    std::vector<int> emptyVector1D;
    iterBest = make_unique<Individual>(routeCapacity, nodeCapacity, emptyVector2D, INFEASIBLE, emptyVector1D);
    globalBest = make_unique<Individual>(routeCapacity, nodeCapacity, emptyVector2D, INFEASIBLE, emptyVector1D);
    if (!population.empty()) {
        globalBestUpper = select_best_individual_by_upper_cost(population)->get_upper_cost();
    } else {
        globalBestUpper = DBL_MAX;
    }
}

void MA::run_heuristic() {
    gen++;

    S_stats = calculate_population_metrics(get_upper_cost_vector_from_group(population));

    vector<shared_ptr<Individual>> S1 = population;
    double v1 = 0;
    double v2;
    shared_ptr<Individual> talentedInd = select_best_individual_by_upper_cost(population);
    if (gen > delta) { //  switch off - False
        // when the generations are greater than the threshold, part of the upper-level sub-solutions S1 will be selected for local search
        double oldUpperCost = talentedInd->get_upper_cost();

        ls_3_vnd(*talentedInd, *instance);

        double newUpperCost = talentedInd->get_upper_cost();
        v1 = oldUpperCost - newUpperCost;
        v2 = *std::max_element(P.begin(), P.end());
        if (v2 < v1) {
            v2 = v1 * gammaL;
        }

        S1.clear();
        for (auto& ind:population) {
            if (ind->get_upper_cost() - v2 <= newUpperCost) S1.push_back(ind);
        }

        auto it = std::find(S1.begin(), S1.end(), talentedInd);
        // If genius_upper is found, remove it from S2
        if (it != S1.end()) {
            S1.erase(it);
        }
    }


    // make local search on S1
    v2 = 0;
    for(auto& ind : S1) {
        double oldUpperCost = ind->get_upper_cost();
        ls_3_vnd(*ind, *instance);
        if (v2 < oldUpperCost - ind->get_upper_cost())
            v2 = oldUpperCost - ind->get_upper_cost();
    }
    v2 = (v1 > v2) ? v1 : v2;
    P.push_back(v2);
    if (P.size() > delta)  P.pop_front();
    if (gen > delta) S1.push_back(talentedInd); //  *** switch off ***


    S1_stats = calculate_population_metrics(get_upper_cost_vector_from_group(S1));

    // Snapshot the upper-level parent pool before follower evaluation.
    // Reproduction should only see upper-level quality.
    vector<shared_ptr<Individual>> rankedUpperSolutions = S1;
    sort(rankedUpperSolutions.begin(), rankedUpperSolutions.end(), [](const shared_ptr<Individual>& lhs, const shared_ptr<Individual>& rhs) {
        return lhs->get_upper_cost() < rhs->get_upper_cost();
    });
    const size_t targetParentPoolSize = std::max<size_t>(2, (static_cast<size_t>(popSize) + 9) / 10);
    if (rankedUpperSolutions.size() > targetParentPoolSize) {
        rankedUpperSolutions.resize(targetParentPoolSize);
    }

    vector<ParentCandidate> parentPool;
    parentPool.reserve(rankedUpperSolutions.size());
    for (const auto& sol : rankedUpperSolutions) {
        ParentCandidate candidate;
        candidate.chromosome = sol->get_chromosome();
        candidate.upperCost = sol->get_upper_cost();
        candidate.signature = build_adjacency_signature(candidate.chromosome);
        parentPool.push_back(std::move(candidate));
    }

    // Current S1 has been selected and local search.
    // For the ablation, trigger lower-level charging only for upper-level solutions
    // whose upper cost is within gammaTrigger * globalBestUpper.
    vector<shared_ptr<Individual>> S2 = S1;
    double v3;
    shared_ptr<Individual> outstandingUpper = select_best_individual_by_upper_cost(S1);
    if (outstandingUpper->get_upper_cost() < globalBestUpper) {
        globalBestUpper = outstandingUpper->get_upper_cost();
    }

    S2.clear();
    const double triggerUpperBound = globalBestUpper * gammaTrigger;
    for (auto& ind : S1) {
        if (ind->get_upper_cost() <= triggerUpperBound) {
            S2.push_back(ind);
        }
    }

    auto it = std::find(S2.begin(), S2.end(), outstandingUpper);
    // outstandingUpper is repaired separately below and should not be duplicated in S2
    if (it != S2.end()) {
        S2.erase(it);
    }

    // Current S2 has been selected and ready for recharging, make recharging on S2
    vector<shared_ptr<Individual>> S3;
    S3.push_back(outstandingUpper); //  *** switch off ***
    const double oldOutstandingUpperCost = outstandingUpper->get_upper_cost();
    const double newOutstandingLowerCost = fix_one_solution(*outstandingUpper, *instance);
    v3 = newOutstandingLowerCost - oldOutstandingUpperCost;
    for (auto& ind:S2) {
        double oldUpperCost = ind->get_upper_cost();
        fix_one_solution(*ind, *instance);
        double newLowerCost = ind->get_lower_cost();
        S3.push_back(ind);
        if (v3 > newLowerCost - oldUpperCost)
            v3 = newLowerCost - oldUpperCost;
    }
    if (r == 0 || r > v3) {
        r = v3;
    }

    S3_stats = calculate_population_metrics(get_lower_cost_vector_from_group(S3));


    // statistics
    iterBest = make_unique<Individual>(*select_best_individual_by_lower_cost(S3));
    if (globalBest->get_lower_cost() > iterBest->get_lower_cost()) {
        globalBest = make_unique<Individual>(*iterBest);
    }


    vector<vector<int>> chromosomes;
    chromosomes.reserve(popSize - 1);

    const int offspringTarget = popSize - 1;
    const bool hasVerifiedBest = globalBest->get_lower_cost() < INFEASIBLE;
    const int verifiedImmigrantTarget = hasVerifiedBest ? static_cast<int>(std::lround(offspringTarget * 0.05)) : 0;
    const int pureImmigrantTarget = static_cast<int>(std::lround(offspringTarget * 0.10));
    const int upperUpperTarget = std::max(0, offspringTarget - verifiedImmigrantTarget - pureImmigrantTarget);

    auto append_child = [&](vector<int>& child, int phaseTarget) {
        if (static_cast<int>(chromosomes.size()) < phaseTarget) {
            chromosomes.push_back(child);
        }
    };

    auto select_upper_parent_index = [&]() {
        const int actualTournamentSize = std::max(1, std::min(tournamentSize, static_cast<int>(parentPool.size())));
        std::uniform_int_distribution<size_t> distribution(0, parentPool.size() - 1);
        size_t bestIndex = distribution(randomEngine);
        for (int i = 1; i < actualTournamentSize; ++i) {
            const size_t challengerIndex = distribution(randomEngine);
            if (parentPool[challengerIndex].upperCost < parentPool[bestIndex].upperCost) {
                bestIndex = challengerIndex;
            }
        }
        return bestIndex;
    };

    auto select_diverse_upper_parent_index = [&](size_t anchorIndex) {
        if (parentPool.size() <= 1) {
            return anchorIndex;
        }

        vector<size_t> candidateIndices;
        candidateIndices.reserve(parentPool.size() - 1);
        for (size_t i = 0; i < parentPool.size(); ++i) {
            if (i != anchorIndex) {
                candidateIndices.push_back(i);
            }
        }
        std::shuffle(candidateIndices.begin(), candidateIndices.end(), randomEngine);
        const size_t sampleSize = std::min<size_t>(8, candidateIndices.size());
        candidateIndices.resize(sampleSize);

        size_t bestIndex = candidateIndices.front();
        double bestDistance = -1.0;
        for (size_t candidateIndex : candidateIndices) {
            const double distance = adjacency_distance(parentPool[anchorIndex], parentPool[candidateIndex]);
            if (distance > bestDistance + 1e-12 ||
                (std::fabs(distance - bestDistance) <= 1e-12 &&
                 parentPool[candidateIndex].upperCost < parentPool[bestIndex].upperCost)) {
                bestDistance = distance;
                bestIndex = candidateIndex;
            }
        }
        return bestIndex;
    };

    while (static_cast<int>(chromosomes.size()) < upperUpperTarget) {
        const size_t parent1Index = select_upper_parent_index();
        const size_t parent2Index = select_diverse_upper_parent_index(parent1Index);
        vector<int> child1 = parentPool[parent1Index].chromosome;
        vector<int> child2 = parentPool[parent2Index].chromosome;
        cxPartiallyMatched(child1, child2, randomEngine);
        append_child(child1, upperUpperTarget);
        append_child(child2, upperUpperTarget);
    }

    const int verifiedPhaseTarget = upperUpperTarget + verifiedImmigrantTarget;
    if (hasVerifiedBest) {
        const vector<int> verifiedChromosome = globalBest->get_chromosome();
        while (static_cast<int>(chromosomes.size()) < verifiedPhaseTarget) {
            vector<int> child1 = verifiedChromosome;
            vector<int> child2 = make_random_immigrant(instance->customers, randomEngine);
            cxPartiallyMatched(child1, child2, randomEngine);
            append_child(child1, verifiedPhaseTarget);
            append_child(child2, verifiedPhaseTarget);
        }
    }

    while (static_cast<int>(chromosomes.size()) < offspringTarget) {
        chromosomes.push_back(make_random_immigrant(instance->customers, randomEngine));
    }

    for (auto& chromosome: chromosomes) {
        if (uniformRealDis(randomEngine) < mutationProb) {
            mutShuffleIndexes(chromosome, mutationIndProb, randomEngine);
        }
    }

    // destroy all the individual objects
    S3.clear();
    S2.clear();
    S1.clear();
    population.clear();
    population.shrink_to_fit();


    // update population
    population.reserve(popSize);
    population.push_back(make_shared<Individual>(*iterBest));
    for (int i = 0; i < popSize - 1; ++i) {
        vector<int> a_giant_tour = {instance->depot};
        a_giant_tour.insert(a_giant_tour.end(), chromosomes[i].begin(), chromosomes[i].end());

        vector<vector<int>> dumb_routes = prins_split(a_giant_tour, *instance);

        for (auto& route : dumb_routes) {
            route.insert(route.begin(), instance->depot);
            route.push_back(instance->depot);
        }

        population.push_back(make_shared<Individual>(routeCapacity, nodeCapacity, dumb_routes,
                                                     instance->fitness_evaluation(dumb_routes),
                                                     instance->compute_demand_sum(dumb_routes))
                                                     );
    }
}
