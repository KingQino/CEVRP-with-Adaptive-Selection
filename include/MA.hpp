//
// Created by Yinghao Qin on 19/12/2023.
//

#ifndef CEVRP_YINGHAO_MA_HPP
#define CEVRP_YINGHAO_MA_HPP

#include <random>
#include <algorithm>
#include <iterator>
#include <deque>
#include <queue>

#include "case.hpp"
#include "stats.hpp"
#include "utils.hpp"
#include "individual.hpp"

using namespace std;

class MA : public StatsInterface{
public:
    static vector<double> get_fitness_vector_from_group(const vector<shared_ptr<Individual>>& group) ;

    MA(Case* instance, int seed, int isMaxEvals = 1, int popSize = 100, double eliteRatio = 0.01, double immigrantRatio = 0.05,
       double crossoverProb = 1.0, double mutationProb = 0.5, double mutationIndProb = 0.2, int tournamentSize = 2);
    ~MA() override;
    void run();
    void initialize_heuristic();
    void run_heuristic();
    bool termination_criteria_1() const;
    bool termination_criteria_2(const std::chrono::duration<double>& runningTime) const;
    void pop_init_with_clustering(); // hien clustering
    void pop_init_with_order_split(); // random order first, split second
    void pop_init_with_direct_encoding(); // direct encoding approach
    void open_log_for_evolution() override;
    void flush_row_into_evol_log() override;
    void close_log_for_evolution() override;
    void save_log_for_solution() override;

    std::ostringstream ossRowEvol;
    Case* instance;
    std::default_random_engine randomEngine;
    uniform_real_distribution<double> uniformRealDis;
    std::vector<std::shared_ptr<Individual>> population;
    std::unique_ptr<Individual> globalBest;
    double globalUpperBest;
    std::unique_ptr<Individual> iterBest;
    PopulationMetrics S1_stats;
    PopulationMetrics S3_stats;
    PopulationMetrics S_stats; // statistics
    queue<double> global_upper_best_in_past_two_gens;
    double recharging_threshold_ratio_last_gen;

    int seed;
    int isMaxEvals; // stop criteria, 1 for max-evals, others for max-exec-time
    int popSize;
    double eliteRatio;
    double immigrantRatio;
    double crossoverProb;
    double mutationProb;
    double mutationIndProb;
    int tournamentSize;

    int routeCapacity;
    int nodeCapacity;
    int gen; // iteration num
    double gammaL; // confidence ratio of local search: 调大可以增加local search的解的个数
    double gammaR; // confidence ratio of recharging: 调小可以增加recharging的解的个数
    int delta;  // confidence interval
    deque<double> P; // list for confidence intervals of local search
    double r; // confidence interval is used to judge whether an upper-level sub-solution should make the charging process
};
#endif //CEVRP_YINGHAO_MA_HPP
