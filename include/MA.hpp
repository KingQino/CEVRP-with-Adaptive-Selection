//
// Created by Yinghao Qin on 19/12/2023.
//

#ifndef CEVRP_YINGHAO_MA_HPP
#define CEVRP_YINGHAO_MA_HPP

#include <random>

#include "case.hpp"
#include "stats.hpp"
#include "individual.hpp"
#include "follower.hpp"
#include "initializer.hpp"
#include "leader.hpp"
#include "local_search_intensity_learner.hpp"
#include "parameters.hpp"
#include "reproduction.hpp"

class MA : public StatsInterface{
public:
    static constexpr const char* EVOLUTION_LOG_HEADER =
        "iter,evals,best_upper_cost,best_lower_cost,progress,duration";
    static constexpr const char* LOCAL_SEARCH_LOG_HEADER =
        "iter\tquality_gap\tdistance_before\tdistance_after\tmove_limit\t"
        "accepted_moves\tneighborhood_calls\tls_evals\trelative_upper_improvement\t"
        "reached_local_optimum\tcrossed_gamma\tlower_evaluated\t"
        "verified_lower_improvement";
    static constexpr const char* LOCAL_SEARCH_OPERATOR_LOG_HEADER =
        "iter\toperator\tcalls\taccepts\tevals\tupper_gain\tgamma_crosses";
    static constexpr const char* LOCAL_SEARCH_LEARNING_LOG_HEADER =
        "iter\tquality_gap\tdistance\tprogress\tgamma_margin\taction\t"
        "ls_evals\tbenefit\tbudget_price\treward";

    MA(Case* instance, const Parameters& parameters);
    ~MA() override;
    void run();
    void initialize_search();
    void run_generation();
    bool reached_evaluation_limit() const;
    bool reached_time_limit(const std::chrono::duration<double>& runningTime) const;
    void initialize_population_with_clustering();
    void initialize_population_with_random_split();
    void initialize_population_with_direct_encoding();
    void open_log_for_evolution() override;
    void flush_row_into_evol_log() override;
    void close_log_for_evolution() override;
    void save_log_for_solution() override;
    void open_log_for_local_search();
    void flush_local_search_log();
    void close_log_for_local_search();

    std::ostringstream evolutionRows;
    std::ostringstream localSearchRows;
    std::ostringstream localSearchOperatorRows;
    std::ostringstream localSearchLearningRows;
    std::ofstream logLocalSearch;
    std::ofstream logLocalSearchOperators;
    std::ofstream logLocalSearchLearning;
    Case* instance;
    std::mt19937 randomEngine;
    std::mt19937 localSearchEngine;
    std::mt19937 localSearchLearningEngine;
    std::uniform_real_distribution<double> uniformRealDis;
    std::vector<std::shared_ptr<Individual>> population;
    std::vector<std::shared_ptr<Individual>> populationBuffer;
    std::unique_ptr<Individual> verifiedBest;
    std::shared_ptr<Individual> retainedLowerElite;
    FollowerWorkspace followerWorkspace;
    SplitWorkspace splitWorkspace;
    ReproductionWorkspace reproductionWorkspace;
    LocalSearchWorkspace localSearchWorkspace;
    LocalSearchIntensityLearner localSearchIntensityLearner;
    int seed;
    int isMaxEvals; // stop criteria, 1 for max-evals, others for max-exec-time
    bool enableLogging;
    std::string statsDirectory;
    int popSize;
    double mutationProb;
    double mutationIndProb;
    int tournamentSize;
    LocalSearchIntensity localSearchIntensity;
    bool enableLocalSearchLearning;
    double parentPoolRatio;
    double qualityRatio;
    double verifiedUpperRatio;
    double pureImmigrantRatio;

    int routeCapacity;
    int nodeCapacity;
    int generation;
    double lowerLevelTriggerRatio;
    double globalBestUpperCost;
};
#endif //CEVRP_YINGHAO_MA_HPP
