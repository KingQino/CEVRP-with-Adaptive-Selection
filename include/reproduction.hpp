#ifndef REPRODUCTION_HPP
#define REPRODUCTION_HPP

#include <cstdint>
#include <memory>
#include <random>
#include <vector>

class Individual;

struct ParentCandidate {
    std::vector<int> chromosome;
    double upperCost;
    std::vector<std::uint64_t> adjacencySignature;
};

class Reproduction {
public:
    static std::shared_ptr<Individual> best_by_upper_cost(
        const std::vector<std::shared_ptr<Individual>>& population);
    static std::shared_ptr<Individual> best_by_lower_cost(
        const std::vector<std::shared_ptr<Individual>>& population);

    static std::vector<ParentCandidate> build_quality_diversity_parent_pool(
        const std::vector<std::shared_ptr<Individual>>& rankedUpperSolutions,
        std::size_t desiredPoolSize,
        double qualityRatio);
    static ParentCandidate make_parent_candidate(const Individual& individual);
    static double adjacency_distance(
        const ParentCandidate& first,
        const ParentCandidate& second);

    static std::vector<int> make_random_immigrant(
        const std::vector<int>& customers,
        std::default_random_engine& randomEngine);
    static void partially_matched_crossover(
        std::vector<int>& firstParent,
        std::vector<int>& secondParent,
        std::default_random_engine& randomEngine);
    static void mutate_by_index_shuffle(
        std::vector<int>& chromosome,
        double mutationProbability,
        std::default_random_engine& randomEngine);

    static std::vector<std::vector<int>> create_offspring(
        const std::vector<ParentCandidate>& parentPool,
        const Individual* verifiedBest,
        bool hasVerifiedBest,
        const std::vector<int>& customers,
        int offspringCount,
        int tournamentSize,
        double mutationProbability,
        double geneMutationProbability,
        double verifiedUpperRatio,
        double pureImmigrantRatio,
        std::default_random_engine& randomEngine,
        std::uniform_real_distribution<double>& probabilityDistribution);
};

#endif
