//
// Created by Yinghao Qin on 19/12/2023.
//

#ifndef CEVRP_YINGHAO_MA_HPP
#define CEVRP_YINGHAO_MA_HPP

#include <random>
#include <algorithm>
#include <iterator>
#include <deque>

#include "case.hpp"
#include "stats.hpp"
#include "individual.hpp"

class MA : public StatsInterface{
public:
    static std::vector<double> collect_upper_costs(
        const std::vector<std::shared_ptr<Individual>>& group);
    static std::vector<double> collect_lower_costs(
        const std::vector<std::shared_ptr<Individual>>& group);

    MA(Case* instance, int seed, int isMaxEvals = 1, int popSize = 100, double eliteRatio = 0.01, double immigrantRatio = 0.05,
       double crossoverProb = 1.0, double mutationProb = 0.5, double mutationIndProb = 0.2, int tournamentSize = 2);
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

    std::ostringstream ossRowEvol;
    Case* instance;
    std::default_random_engine randomEngine;
    std::uniform_real_distribution<double> uniformRealDis;
    std::vector<std::shared_ptr<Individual>> population;
    std::unique_ptr<Individual> verifiedBest;
    std::unique_ptr<Individual> generationBestComplete;
    PopulationMetrics populationMetrics;
    PopulationMetrics upperCandidateMetrics;
    PopulationMetrics followerEvaluatedMetrics;

    int seed;
    int isMaxEvals; // stop criteria, 1 for max-evals, others for max-exec-time
    int popSize;
    double eliteRatio; // Legacy API option; reproduction currently uses fixed phase ratios.
    double immigrantRatio; // Legacy API option; reproduction currently injects a fixed 10%.
    double crossoverProb; // Legacy API option; PMX is currently always applied.
    double mutationProb;
    double mutationIndProb;
    int tournamentSize;

    int routeCapacity;
    int nodeCapacity;
    int generation;
    double localSearchConfidenceMultiplier;
    double chargingConfidenceMultiplier; // Retained from the former confidence filter.
    double lowerLevelTriggerRatio;
    int confidenceWindowSize;
    std::deque<double> recentUpperImprovements;
    double bestObservedChargingPenalty; // Monitoring state retained from the former filter.
    double globalBestUpperCost;
};
#endif //CEVRP_YINGHAO_MA_HPP
