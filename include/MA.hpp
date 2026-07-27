//
// Created by Yinghao Qin on 19/12/2023.
//

#ifndef CEVRP_YINGHAO_MA_HPP
#define CEVRP_YINGHAO_MA_HPP

#include <array>
#include <cstdint>
#include <random>

#include "case.hpp"
#include "stats.hpp"
#include "individual.hpp"
#include "follower.hpp"
#include "initializer.hpp"
#include "leader.hpp"
#include "local_search_allocation.hpp"
#include "parameters.hpp"
#include "reproduction.hpp"

class MA : public StatsInterface{
public:
    static constexpr int LOCAL_SEARCH_ALLOCATION_LOG_INTERVAL = 10;
    static constexpr const char* EVOLUTION_LOG_HEADER =
        "iter,evals,best_upper_cost,best_lower_cost,progress,duration";
    static constexpr const char* LOCAL_SEARCH_OPERATOR_LOG_HEADER =
        "iter\toperator\tcalls\taccepts\tevals\tupper_gain\tgamma_crosses";
    static constexpr const char* LOCAL_SEARCH_ALLOCATION_LOG_HEADER =
        "iter\tpolicy\taction\tselections\tforced_local_optima\t"
        "exploratory_selections\treached_local_optimum\t"
        "hit_move_limit\thit_distance_limit\taccepted_moves\t"
        "neighborhood_calls\tevals\tupper_gain\tgamma_crosses\tparent_uses\t"
        "lower_archive_entries\tparent_reward\tlower_reward\t"
        "gamma_reward\tcontinuation_gain_signal\treward\t"
        "avg_incremental_cost_units\tavg_score\t"
        "avg_productive_neighborhood_fraction";

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
    void accumulate_local_search_allocation_stats(
        const std::array<LocalSearchAllocationStats, 3>& stats);
    void write_local_search_allocation_snapshot();

    std::ostringstream evolutionRows;
    std::ostringstream localSearchOperatorRows;
    std::ostringstream localSearchAllocationRows;
    std::ofstream logLocalSearchOperators;
    std::ofstream logLocalSearchAllocation;
    std::array<LocalSearchAllocationStats, 3>
        pendingLocalSearchAllocationStats{};
    int pendingLocalSearchAllocationGenerations{};
    Case* instance;
    std::mt19937 randomEngine;
    std::mt19937 localSearchEngine;
    std::mt19937 localSearchAllocationEngine;
    std::uniform_real_distribution<double> uniformRealDis;
    std::vector<std::shared_ptr<Individual>> population;
    std::vector<std::shared_ptr<Individual>> populationBuffer;
    std::unique_ptr<Individual> verifiedBest;
    std::unique_ptr<Individual> upperBestIndividual;
    std::shared_ptr<Individual> retainedLowerElite;
    FollowerWorkspace followerWorkspace;
    SplitWorkspace splitWorkspace;
    ReproductionWorkspace reproductionWorkspace;
    LocalSearchWorkspace localSearchWorkspace;
    std::vector<LocalSearchWorkspace> mixedLocalSearchWorkspaces;
    OnlineIntensityLearner localSearchAllocator;
    int seed;
    int isMaxEvals; // stop criteria, 1 for max-evals, others for max-exec-time
    bool enableLogging;
    std::string statsDirectory;
    int popSize;
    double mutationProb;
    double mutationIndProb;
    int tournamentSize;
    LocalSearchIntensity localSearchIntensity;
    LocalSearchPolicy localSearchPolicy;
    double parentPoolRatio;
    double qualityRatio;
    double verifiedUpperRatio;
    double pureImmigrantRatio;

    int routeCapacity;
    int nodeCapacity;
    int generation;
    double lowerLevelTriggerRatio;
    double globalBestUpperCost;
    std::uint64_t lastUpperImprovementDistanceCalls{};
    std::uint64_t lastLowerImprovementDistanceCalls{};
    double recentGammaEntryRate{};
};
#endif //CEVRP_YINGHAO_MA_HPP
