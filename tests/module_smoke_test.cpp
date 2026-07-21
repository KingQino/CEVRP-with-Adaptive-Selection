#include <algorithm>
#include <cassert>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "MA.hpp"
#include "follower.hpp"
#include "initializer.hpp"
#include "leader.hpp"
#include "reproduction.hpp"

namespace {

void assert_routes_cover_customers(
    const std::vector<std::vector<int>>& routes,
    const Case& instance) {
    std::vector<int> routedCustomers;
    for (const auto& route : routes) {
        assert(route.size() >= 2);
        assert(route.front() == instance.depot);
        assert(route.back() == instance.depot);
        routedCustomers.insert(routedCustomers.end(), route.begin() + 1, route.end() - 1);
    }

    std::vector<int> expectedCustomers = instance.customers;
    std::sort(routedCustomers.begin(), routedCustomers.end());
    std::sort(expectedCustomers.begin(), expectedCustomers.end());
    assert(routedCustomers == expectedCustomers);
}

void assert_is_customer_permutation(
    const std::vector<int>& chromosome,
    const Case& instance) {
    std::vector<int> sortedChromosome = chromosome;
    std::vector<int> expectedCustomers = instance.customers;
    std::sort(sortedChromosome.begin(), sortedChromosome.end());
    std::sort(expectedCustomers.begin(), expectedCustomers.end());
    assert(sortedChromosome == expectedCustomers);
}

}  // namespace

int main(int argc, char* argv[]) {
    const std::string instanceName = argc > 1 ? argv[1] : "E-n22-k4.evrp";
    const std::string instancePath = std::string(TEST_DATA_DIRECTORY) + "/" + instanceName;
    Case instance(instancePath, 1);
    std::default_random_engine randomEngine(1);

    auto clusteredRoutes = Initializer::build_with_clustering(instance, randomEngine);
    auto splitRoutes = Initializer::build_with_random_split(instance, randomEngine);
    auto directRoutes = Initializer::build_with_direct_encoding(instance, randomEngine);
    assert_routes_cover_customers(clusteredRoutes, instance);
    assert_routes_cover_customers(splitRoutes, instance);
    assert_routes_cover_customers(directRoutes, instance);

    Individual individual(
        instance.vehicleNumber * 3,
        instance.customerNumber + 2,
        clusteredRoutes,
        instance.fitness_evaluation(clusteredRoutes),
        instance.compute_demand_sum(clusteredRoutes));
    const double upperCostBeforeSearch = individual.get_upper_cost();
    Individual vndBaseline(individual);
    Leader::improve_with_three_neighborhood_vnd(vndBaseline, instance);
    assert(vndBaseline.get_upper_cost() <= upperCostBeforeSearch + 1e-8);

    Individual repeatedRvnd(individual);
    std::default_random_engine firstRvndEngine(7);
    std::default_random_engine secondRvndEngine(7);
    Leader::improve_with_three_neighborhood_rvnd(individual, instance, firstRvndEngine);
    Leader::improve_with_three_neighborhood_rvnd(repeatedRvnd, instance, secondRvndEngine);
    assert(individual.get_upper_cost() <= upperCostBeforeSearch + 1e-8);
    assert(std::fabs(individual.get_upper_cost() - repeatedRvnd.get_upper_cost()) <= 1e-8);
    assert(individual.get_routes() == repeatedRvnd.get_routes());

    Follower::optimize_charging(individual, instance);
    assert(std::isfinite(individual.get_lower_cost()));
    if (instance.customerNumber <= 30) {
        Follower::refine_charging_by_enumeration(individual, instance);
        assert(std::isfinite(individual.get_lower_cost()));
    }

    std::vector<std::shared_ptr<Individual>> rankedSolutions;
    rankedSolutions.push_back(std::make_shared<Individual>(individual));
    rankedSolutions.push_back(std::make_shared<Individual>(
        instance.vehicleNumber * 3,
        instance.customerNumber + 2,
        splitRoutes,
        instance.fitness_evaluation(splitRoutes),
        instance.compute_demand_sum(splitRoutes)));
    std::sort(
        rankedSolutions.begin(),
        rankedSolutions.end(),
        [](const auto& first, const auto& second) {
            return first->get_upper_cost() < second->get_upper_cost();
        });
    const auto parentPool = Reproduction::build_quality_diversity_parent_pool(rankedSolutions, 2);
    assert(!parentPool.empty());
    if (parentPool.size() == 2) {
        const double distance = Reproduction::adjacency_distance(parentPool[0], parentPool[1]);
        assert(distance >= 0.0 && distance <= 1.0);
    }

    auto firstChild = Reproduction::make_random_immigrant(instance.customers, randomEngine);
    auto secondChild = Reproduction::make_random_immigrant(instance.customers, randomEngine);
    Reproduction::partially_matched_crossover(firstChild, secondChild, randomEngine);
    Reproduction::mutate_by_index_shuffle(firstChild, 0.2, randomEngine);
    assert_is_customer_permutation(firstChild, instance);
    assert_is_customer_permutation(secondChild, instance);

    MA algorithm(&instance, 1, 1, 10);
    algorithm.initialize_search();
    algorithm.run_generation();
    assert(algorithm.population.size() == 10);
    assert(algorithm.verifiedBest != nullptr);
    assert(std::isfinite(algorithm.verifiedBest->get_lower_cost()));
    assert(std::string(MA::EVOLUTION_LOG_HEADER)
           == "iter,evals,best_upper_cost,best_lower_cost,progress,duration");
    algorithm.flush_row_into_evol_log();
    const std::string evolutionRow = algorithm.evolutionRows.str();
    assert(std::count(evolutionRow.begin(), evolutionRow.end(), ',') == 5);
    return 0;
}
