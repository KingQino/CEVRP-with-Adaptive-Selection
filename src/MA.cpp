//
// Created by Yinghao Qin on 19/12/2023.
//

#include "../include/MA.hpp"

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
    this->nodeCapacity = this->instance->customerNumber + 1;
    this->gen = 0;
    this->gammaL = 1.2;
    this->gammaR = 0.8;
    this->delta = 30;
    this->r = 0.0;
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

vector<double> MA::get_fitness_vector_from_group(const vector<shared_ptr<Individual>>& group) {
    std::vector<double> ans;
    ans.reserve(group.size());  // Reserve space to avoid unnecessary reallocation

    // Use transform along with a lambda to extract fitness values
    std::transform(group.begin(), group.end(), std::back_inserter(ans),
                   [](const auto& ind) { return ind->get_fit(); });

    return ans;
}

void MA::open_log_for_evolution() {
    string directoryPath = "../" + statsPath + "/" + instance->instanceName + "/" + to_string(seed);
    create_directories_if_not_exists(directoryPath);

    string filename = "evols." + instance->instanceName + ".csv";
    logEvolution.open(directoryPath + "/" + filename);
    logEvolution << "generation,pop_size,"
                    "offspring_size,S_min_fit,S_avg_fit,S_max_fit,S_std_fit,"
                    "upper_pop_size,S1_min_fit,S1_avg_fit,S1_max_fit,S1_std_fit,"
                    "lower_pop_size,S3_min_fit,S3_avg_fit,S3_max_fit,S3_std_fit,S3_infeasible_size,"
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
    string directoryPath = "../" + statsPath + "/" + instance->instanceName + "/" + to_string(seed);
    create_directories_if_not_exists(directoryPath);
    string filename = "solution." + instance->instanceName + ".txt";

    logSolution.open(directoryPath + "/" + filename);
    logSolution << fixed << setprecision(5) << globalBest->get_fit() << endl;
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
}

void MA::run_heuristic() {
    gen++;

    S_stats = calculate_population_metrics(get_fitness_vector_from_group(population));

    vector<shared_ptr<Individual>> S1 = population;
    double v1 = 0;
    double v2;
    shared_ptr<Individual> talentedInd = select_best_individual(population);
    if (gen > delta) { //  switch off - False
        // when the generations are greater than the threshold, part of the upper-level sub-solutions S1 will be selected for local search
        double old_fit = talentedInd->get_fit();

        two_opt_for_individual(*talentedInd, *instance);
        two_opt_star_for_individual(*talentedInd, *instance);
        node_shift_for_individual(*talentedInd, *instance);

        double new_fit = talentedInd->get_fit();
        v1 = old_fit - new_fit;
        v2 = v1 * 1.2;

        S1.clear();
        for (auto& ind:population) {
            if (ind->get_fit() - v2 <= new_fit) S1.push_back(ind);
        }

        auto it = std::find(S1.begin(), S1.end(), talentedInd);
        // If genius_upper is found, remove it from S2
        if (it != S1.end()) {
            S1.erase(it);
        }
    }


    // make local search on S1
    for(auto& ind : S1) {
        two_opt_for_individual(*ind, *instance); // 2-opt
        two_opt_star_for_individual(*ind, *instance);
        node_shift_for_individual(*ind, *instance);
    }
    if (gen > delta) S1.push_back(talentedInd); //  *** switch off ***

    S1_stats = calculate_population_metrics(get_fitness_vector_from_group(S1));

    // Current S1 has been selected and local search.
    // Pick a portion of the upper sub-solutions to go for recharging process, by the difference between before and after charging of the best solution in S1
    vector<shared_ptr<Individual>> S2 = S1;
    double v3;
    shared_ptr<Individual> outstandingUpper = select_best_individual(S1);
    if (gen > 0) { // Switch = off False
        // 开关 此处只是设计了一个总是为真的虚拟条件，需要具体实现
        double old_fit = outstandingUpper->get_fit(); // fitness without recharging f
        double new_fit = fix_one_solution(*outstandingUpper, *instance); // // fitness with recharging F
        v3 = new_fit - old_fit;
        r = v3 * 0.8;

        S2.clear();
        for (auto& ind:S1) {
            if (ind->get_fit() + r <= new_fit)
                S2.push_back(ind);
        }

        auto it = std::find(S2.begin(), S2.end(), outstandingUpper);
        // If genius_upper is found, remove it from S2
        if (it != S2.end()) {
            S2.erase(it);
        }
    }

    // Current S2 has been selected and ready for recharging, make recharging on S2
    vector<shared_ptr<Individual>> S3;
    S3.push_back(outstandingUpper); //  *** switch off ***
    for (auto& ind:S2) {
        double old_fit = ind->get_fit();
        fix_one_solution(*ind, *instance);
        double new_fit = ind->get_fit();
        S3.push_back(ind);
        if (v3 > new_fit - old_fit)
            v3 = new_fit - old_fit;
    }
    if (r == 0 || r > v3) {
        r = v3;
    }

    S3_stats = calculate_population_metrics(get_fitness_vector_from_group(S3));

    // statistics
    iterBest = make_unique<Individual>(*select_best_individual(S3));
    if (globalBest->get_fit() > iterBest->get_fit()) {
        globalBest = make_unique<Individual>(*iterBest);
    }


    // Selection
    vector<vector<int>> promising_seqs;
    promising_seqs.reserve(S3.size());
    for(auto& sol : S3) {
        promising_seqs.push_back(sol->get_chromosome()); // encoding
    }

    vector<vector<int>> average_seqs;
    for(auto& sol : population) {
        // judge whether sol in S3 or not
        auto it = std::find(S3.begin(), S3.end(), sol);
        if (it != S3.end()) continue;
        average_seqs.push_back(sol->get_chromosome()); // encoding
    }


    vector<vector<int>> chromosomes;
    if (promising_seqs.size() == 1) {
        vector<int> father = promising_seqs[0];
        // 90% - elite x non-elites
        for (int i = 0; i < int (0.45 * popSize); ++i) {
            vector<int> _father(father);
            vector<int> mother = selRandom(average_seqs, 1, randomEngine)[0];
            cxPartiallyMatched(_father, mother, randomEngine);
            chromosomes.push_back(_father);
            chromosomes.push_back(mother);
        }
        // 9%  - elite x immigrants
        for (int i = 0; i < int(0.05 * popSize); ++i) {
            vector<int> _father(father);
            vector<int> mother(instance->customers);
            shuffle(mother.begin(), mother.end(), randomEngine);
            cxPartiallyMatched(_father, mother, randomEngine);
            chromosomes.push_back(_father);
            chromosomes.push_back(mother);
        }
//        chromosomes.pop_back();
        // free 1 space  - best ind
    } else {
        // part of elites x elites
        int num_promising_seqs = promising_seqs.size();
        int loop_num = int(num_promising_seqs / 2.0) <= (popSize/2) ? int(num_promising_seqs / 2.0) : int(popSize/4);
        for (int i = 0; i < loop_num; ++i) {
            vector<vector<int>> parents = selRandom(promising_seqs, 2, randomEngine);
            cxPartiallyMatched(parents[0], parents[1], randomEngine);
            chromosomes.push_back(parents[0]);
            chromosomes.push_back(parents[1]);
        }
        // portion of elites x non-elites
        int num_promising_x_average = popSize - chromosomes.size();
        for (int i = 0; i < int(num_promising_x_average / 2.0); ++i) {
            vector<int> parent1 = selRandom(promising_seqs, 1, randomEngine)[0];
            vector<int> parent2 = selRandom(average_seqs, 1, randomEngine)[0];
            cxPartiallyMatched(parent1, parent2, randomEngine);
            chromosomes.push_back(parent1);
            chromosomes.push_back(parent2);
        }
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

