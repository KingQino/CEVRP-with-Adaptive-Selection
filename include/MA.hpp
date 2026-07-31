//
// Created by Yinghao Qin on 19/12/2023.
//

#ifndef CEVRP_YINGHAO_MA_HPP
#define CEVRP_YINGHAO_MA_HPP

#include <array>
#include <cstdint>
#include <random>

#include "case.hpp"
#include "elite_unlimited.hpp"
#include "stats.hpp"
#include "individual.hpp"
#include "follower.hpp"
#include "initializer.hpp"
#include "leader.hpp"
#include "local_search_allocation.hpp"
#include "parameters.hpp"
#include "reproduction.hpp"

struct SearchBudgetStats {
    std::uint64_t normalLocalSearchDistanceCalls{};
    std::uint64_t eliteLocalSearchDistanceCalls{};
    std::uint64_t followerDistanceCalls{};
    std::uint64_t otherDistanceCalls{};
    std::uint64_t totalDistanceCalls{};
    int followerCandidates{};
    int followerRuns{};
};

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
        "neighborhood_calls\tevals\tcontinuation_evals\tupper_gain\t"
        "gamma_crosses\tparent_uses\t"
        "lower_archive_entries\tparent_reward\tlower_reward\t"
        "gamma_reward\tcontinuation_gain_signal\treward\t"
        "avg_incremental_cost_units\tavg_score";
    static constexpr const char* ELITE_UNLIMITED_LOG_HEADER =
        "iter\teligible_candidates\ttriggers\tnormal_ls_evals\t"
        "elite_evals\taccepted_moves\tneighborhood_calls\tupper_gain\t"
        "gamma_crosses\tparent_uses\tlower_archive_entries\t"
        "verified_improvements\tbudget_credit_evals";
    static constexpr const char* BUDGET_ALLOCATION_LOG_HEADER =
        "iter\tnormal_ls_evals\telite_ls_evals\tfollower_evals\t"
        "other_evals\ttotal_evals\tnormal_ls_share\telite_ls_share\t"
        "follower_share\tother_share\tfollower_candidates\tfollower_runs";
    static constexpr const char* RUN_CONFIGURATION_LOG_HEADER =
        "seed\tgamma\telite_rho\tls_depth\tweak_move_fraction\t"
        "medium_move_fraction\tbounded_strong_move_fraction\t"
        "bounded_strong_weak_call_multiplier\tls_cost_penalty";

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
    void accumulate_elite_unlimited_stats(
        const EliteUnlimitedStats& stats);
    void write_elite_unlimited_snapshot();
    void accumulate_search_budget_stats(const SearchBudgetStats& stats);
    void write_search_budget_snapshot();
    [[nodiscard]] bool elite_unlimited_enabled() const;

    std::ostringstream evolutionRows;
    std::ostringstream localSearchOperatorRows;
    std::ostringstream localSearchAllocationRows;
    std::ostringstream eliteUnlimitedRows;
    std::ostringstream searchBudgetRows;
    std::ofstream logLocalSearchOperators;
    std::ofstream logLocalSearchAllocation;
    std::ofstream logEliteUnlimited;
    std::ofstream logSearchBudget;
    std::ofstream logRunConfiguration;
    std::array<LocalSearchAllocationStats, 3>
        pendingLocalSearchAllocationStats{};
    int pendingLocalSearchAllocationGenerations{};
    EliteUnlimitedStats pendingEliteUnlimitedStats{};
    int pendingEliteUnlimitedGenerations{};
    SearchBudgetStats pendingSearchBudgetStats{};
    int pendingSearchBudgetGenerations{};
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
    LocalSearchWorkspace eliteUnlimitedWorkspace;
    std::vector<LocalSearchWorkspace> mixedLocalSearchWorkspaces;
    OnlineIntensityLearner localSearchAllocator;
    EliteUnlimitedController eliteUnlimitedController;
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
    LocalSearchDepthProfile localSearchDepthProfile;
    LocalSearchDepthConfig localSearchDepthConfig;
    double localSearchCostPenalty;
    double eliteBudgetRatio;
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
