#include "../include/reproduction.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <unordered_map>

#include "../include/individual.hpp"

namespace {

using AdjacencySignature = std::vector<std::uint64_t>;

std::uint64_t encode_undirected_edge(int firstNode, int secondNode) {
    if (firstNode > secondNode) {
        std::swap(firstNode, secondNode);
    }
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(firstNode)) << 32)
        | static_cast<std::uint32_t>(secondNode);
}

AdjacencySignature build_depot_aware_signature(const Individual& individual) {
    AdjacencySignature signature;
    std::size_t edgeCount = 0;
    for (int routeIndex = 0; routeIndex < individual.route_num; ++routeIndex) {
        if (individual.node_num[routeIndex] > 1) {
            edgeCount += static_cast<std::size_t>(individual.node_num[routeIndex] - 1);
        }
    }
    signature.reserve(edgeCount);

    // Route boundary edges are included and multiplicity is preserved.
    for (int routeIndex = 0; routeIndex < individual.route_num; ++routeIndex) {
        for (int nodeIndex = 1; nodeIndex < individual.node_num[routeIndex]; ++nodeIndex) {
            signature.push_back(encode_undirected_edge(
                individual.routes[routeIndex][nodeIndex - 1],
                individual.routes[routeIndex][nodeIndex]));
        }
    }
    std::sort(signature.begin(), signature.end());
    return signature;
}

double multiset_jaccard_similarity(
    const AdjacencySignature& first,
    const AdjacencySignature& second) {
    if (first.empty() && second.empty()) {
        return 1.0;
    }

    std::size_t intersectionSize = 0;
    std::size_t unionSize = 0;
    std::size_t firstIndex = 0;
    std::size_t secondIndex = 0;
    while (firstIndex < first.size() && secondIndex < second.size()) {
        if (first[firstIndex] == second[secondIndex]) {
            ++intersectionSize;
            ++unionSize;
            ++firstIndex;
            ++secondIndex;
        } else if (first[firstIndex] < second[secondIndex]) {
            ++unionSize;
            ++firstIndex;
        } else {
            ++unionSize;
            ++secondIndex;
        }
    }
    unionSize += first.size() - firstIndex;
    unionSize += second.size() - secondIndex;
    return static_cast<double>(intersectionSize) / static_cast<double>(unionSize);
}

}  // namespace

std::shared_ptr<Individual> Reproduction::best_by_upper_cost(
    const std::vector<std::shared_ptr<Individual>>& population) {
    if (population.empty()) {
        return nullptr;
    }
    return *std::min_element(
        population.begin(),
        population.end(),
        [](const std::shared_ptr<Individual>& first, const std::shared_ptr<Individual>& second) {
            return first->get_upper_cost() < second->get_upper_cost();
        });
}

std::shared_ptr<Individual> Reproduction::best_by_lower_cost(
    const std::vector<std::shared_ptr<Individual>>& population) {
    if (population.empty()) {
        return nullptr;
    }
    return *std::min_element(
        population.begin(),
        population.end(),
        [](const std::shared_ptr<Individual>& first, const std::shared_ptr<Individual>& second) {
            return first->get_lower_cost() < second->get_lower_cost();
        });
}

double Reproduction::adjacency_distance(
    const ParentCandidate& first,
    const ParentCandidate& second) {
    return 1.0 - multiset_jaccard_similarity(
        first.adjacencySignature,
        second.adjacencySignature);
}

ParentCandidate Reproduction::make_parent_candidate(const Individual& individual) {
    ParentCandidate candidate;
    candidate.chromosome = individual.get_chromosome();
    candidate.upperCost = individual.get_upper_cost();
    candidate.adjacencySignature = build_depot_aware_signature(individual);
    return candidate;
}

std::vector<ParentCandidate> Reproduction::build_quality_diversity_parent_pool(
    const std::vector<std::shared_ptr<Individual>>& rankedUpperSolutions,
    std::size_t desiredPoolSize) {
    if (rankedUpperSolutions.empty() || desiredPoolSize == 0) {
        return {};
    }

    const std::size_t candidateLimit = std::min(
        rankedUpperSolutions.size(),
        desiredPoolSize * 3);
    std::set<AdjacencySignature> seenSignatures;
    std::vector<ParentCandidate> candidates;
    candidates.reserve(candidateLimit);

    for (const auto& solution : rankedUpperSolutions) {
        ParentCandidate candidate = make_parent_candidate(*solution);
        if (seenSignatures.insert(candidate.adjacencySignature).second) {
            candidates.push_back(std::move(candidate));
            if (candidates.size() == candidateLimit) {
                break;
            }
        }
    }

    const std::size_t poolSize = std::min(desiredPoolSize, candidates.size());
    const std::size_t qualitySlots = (poolSize + 1) / 2;
    std::vector<bool> selected(candidates.size(), false);
    std::vector<ParentCandidate> parentPool;
    parentPool.reserve(poolSize);

    for (std::size_t i = 0; i < qualitySlots; ++i) {
        parentPool.push_back(candidates[i]);
        selected[i] = true;
    }

    while (parentPool.size() < poolSize) {
        std::size_t bestIndex = candidates.size();
        double bestMinimumDistance = -1.0;

        for (std::size_t i = 0; i < candidates.size(); ++i) {
            if (selected[i]) {
                continue;
            }

            double minimumDistance = std::numeric_limits<double>::infinity();
            for (const auto& selectedParent : parentPool) {
                minimumDistance = std::min(
                    minimumDistance,
                    adjacency_distance(candidates[i], selectedParent));
            }

            const bool betterDiversity = minimumDistance > bestMinimumDistance + 1e-12;
            const bool sameDiversity = std::fabs(minimumDistance - bestMinimumDistance) <= 1e-12;
            const bool betterQuality = bestIndex == candidates.size()
                || candidates[i].upperCost < candidates[bestIndex].upperCost - 1e-12;
            const bool sameQuality = bestIndex != candidates.size()
                && std::fabs(candidates[i].upperCost - candidates[bestIndex].upperCost) <= 1e-12;
            const bool smallerChromosome = bestIndex != candidates.size()
                && candidates[i].chromosome < candidates[bestIndex].chromosome;

            if (betterDiversity
                || (sameDiversity && betterQuality)
                || (sameDiversity && sameQuality && smallerChromosome)) {
                bestIndex = i;
                bestMinimumDistance = minimumDistance;
            }
        }

        if (bestIndex == candidates.size()) {
            break;
        }
        parentPool.push_back(candidates[bestIndex]);
        selected[bestIndex] = true;
    }
    return parentPool;
}

std::vector<int> Reproduction::make_random_immigrant(
    const std::vector<int>& customers,
    std::default_random_engine& randomEngine) {
    std::vector<int> immigrant(customers);
    std::shuffle(immigrant.begin(), immigrant.end(), randomEngine);
    return immigrant;
}

void Reproduction::partially_matched_crossover(
    std::vector<int>& firstParent,
    std::vector<int>& secondParent,
    std::default_random_engine& randomEngine) {
    const int chromosomeSize = static_cast<int>(firstParent.size());
    std::uniform_int_distribution<> distribution(0, chromosomeSize - 1);

    int firstCut = distribution(randomEngine);
    int secondCut = distribution(randomEngine);
    if (firstCut > secondCut) {
        std::swap(firstCut, secondCut);
    }

    std::vector<int> firstChild(firstParent.begin() + firstCut, firstParent.begin() + secondCut);
    std::vector<int> secondChild(secondParent.begin() + firstCut, secondParent.begin() + secondCut);
    std::unordered_map<int, int> firstMapping;
    std::unordered_map<int, int> secondMapping;
    for (int i = 0; i < secondCut - firstCut; ++i) {
        firstMapping[secondChild[i]] = firstChild[i];
        secondMapping[firstChild[i]] = secondChild[i];
    }

    for (int i = 0; i < chromosomeSize; ++i) {
        if (i < firstCut || i >= secondCut) {
            int firstGene = firstParent[i];
            int secondGene = secondParent[i];
            while (firstMapping.find(firstGene) != firstMapping.end()) {
                firstGene = firstMapping[firstGene];
            }
            while (secondMapping.find(secondGene) != secondMapping.end()) {
                secondGene = secondMapping[secondGene];
            }
            firstChild.push_back(secondGene);
            secondChild.push_back(firstGene);
        }
    }

    firstParent = firstChild;
    secondParent = secondChild;
}

void Reproduction::mutate_by_index_shuffle(
    std::vector<int>& chromosome,
    double mutationProbability,
    std::default_random_engine& randomEngine) {
    const int chromosomeSize = static_cast<int>(chromosome.size());
    std::uniform_real_distribution<double> probabilityDistribution(0.0, 1.0);
    std::uniform_int_distribution<int> swapDistribution(0, chromosomeSize - 2);

    for (int i = 0; i < chromosomeSize; ++i) {
        if (probabilityDistribution(randomEngine) < mutationProbability) {
            int swapIndex = swapDistribution(randomEngine);
            if (swapIndex >= i) {
                ++swapIndex;
            }
            std::swap(chromosome[i], chromosome[swapIndex]);
        }
    }
}

std::vector<std::vector<int>> Reproduction::create_offspring(
    const std::vector<ParentCandidate>& parentPool,
    const Individual* verifiedBest,
    bool hasVerifiedBest,
    const std::vector<int>& customers,
    int offspringCount,
    int tournamentSize,
    double mutationProbability,
    double geneMutationProbability,
    std::default_random_engine& randomEngine,
    std::uniform_real_distribution<double>& probabilityDistribution) {
    std::vector<std::vector<int>> offspring;
    offspring.reserve(offspringCount);

    const int verifiedImmigrantCount = hasVerifiedBest
        ? static_cast<int>(std::lround(offspringCount * 0.05))
        : 0;
    const int pureImmigrantCount = static_cast<int>(std::lround(offspringCount * 0.10));
    const int upperParentOffspringCount = std::max(
        0,
        offspringCount - verifiedImmigrantCount - pureImmigrantCount);

    auto appendChild = [&](std::vector<int>& child, int phaseTarget) {
        if (static_cast<int>(offspring.size()) < phaseTarget) {
            offspring.push_back(child);
        }
    };

    auto selectUpperParentIndex = [&]() {
        const int actualTournamentSize = std::max(
            1,
            std::min(tournamentSize, static_cast<int>(parentPool.size())));
        std::uniform_int_distribution<std::size_t> distribution(0, parentPool.size() - 1);
        std::size_t bestIndex = distribution(randomEngine);
        for (int i = 1; i < actualTournamentSize; ++i) {
            const std::size_t challengerIndex = distribution(randomEngine);
            if (parentPool[challengerIndex].upperCost < parentPool[bestIndex].upperCost) {
                bestIndex = challengerIndex;
            }
        }
        return bestIndex;
    };

    auto selectDiverseMateIndex = [&](std::size_t anchorIndex) {
        if (parentPool.size() <= 1) {
            return anchorIndex;
        }

        std::vector<std::size_t> candidateIndices;
        candidateIndices.reserve(parentPool.size() - 1);
        for (std::size_t i = 0; i < parentPool.size(); ++i) {
            if (i != anchorIndex) {
                candidateIndices.push_back(i);
            }
        }
        std::shuffle(candidateIndices.begin(), candidateIndices.end(), randomEngine);
        const std::size_t sampleSize = std::min<std::size_t>(8, candidateIndices.size());
        candidateIndices.resize(sampleSize);

        std::size_t bestIndex = candidateIndices.front();
        double bestDistance = -1.0;
        for (std::size_t candidateIndex : candidateIndices) {
            const double distance = adjacency_distance(
                parentPool[anchorIndex],
                parentPool[candidateIndex]);
            if (distance > bestDistance + 1e-12
                || (std::fabs(distance - bestDistance) <= 1e-12
                    && parentPool[candidateIndex].upperCost < parentPool[bestIndex].upperCost)) {
                bestDistance = distance;
                bestIndex = candidateIndex;
            }
        }
        return bestIndex;
    };

    while (static_cast<int>(offspring.size()) < upperParentOffspringCount) {
        const std::size_t firstParentIndex = selectUpperParentIndex();
        std::vector<int> firstChild = parentPool[firstParentIndex].chromosome;
        std::vector<int> secondChild;
        if (parentPool.size() == 1) {
            secondChild = make_random_immigrant(customers, randomEngine);
        } else {
            const std::size_t secondParentIndex = selectDiverseMateIndex(firstParentIndex);
            secondChild = parentPool[secondParentIndex].chromosome;
        }
        partially_matched_crossover(firstChild, secondChild, randomEngine);
        appendChild(firstChild, upperParentOffspringCount);
        appendChild(secondChild, upperParentOffspringCount);
    }

    const int verifiedPhaseTarget = upperParentOffspringCount + verifiedImmigrantCount;
    if (hasVerifiedBest) {
        const std::vector<int> verifiedChromosome = verifiedBest->get_chromosome();
        while (static_cast<int>(offspring.size()) < verifiedPhaseTarget) {
            std::vector<int> firstChild = verifiedChromosome;
            std::vector<int> secondChild = make_random_immigrant(
                customers,
                randomEngine);
            partially_matched_crossover(firstChild, secondChild, randomEngine);
            appendChild(firstChild, verifiedPhaseTarget);
            appendChild(secondChild, verifiedPhaseTarget);
        }
    }

    while (static_cast<int>(offspring.size()) < offspringCount) {
        offspring.push_back(make_random_immigrant(customers, randomEngine));
    }

    for (auto& chromosome : offspring) {
        if (probabilityDistribution(randomEngine) < mutationProbability) {
            mutate_by_index_shuffle(
                chromosome,
                geneMutationProbability,
                randomEngine);
        }
    }
    return offspring;
}
