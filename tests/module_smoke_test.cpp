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

void assert_individual_is_consistent(
    const Individual& individual,
    Case& instance) {
    const auto routes = individual.get_routes();
    assert(static_cast<int>(routes.size()) == individual.route_num);
    assert(individual.route_num <= individual.route_cap);
    assert_routes_cover_customers(routes, instance);

    double recomputedCost = 0.0;
    for (int routeIndex = 0; routeIndex < individual.route_num; ++routeIndex) {
        const auto& route = routes[routeIndex];
        assert(static_cast<int>(route.size()) == individual.node_num[routeIndex]);

        int recomputedDemand = 0;
        for (std::size_t nodeIndex = 1; nodeIndex + 1 < route.size(); ++nodeIndex) {
            recomputedDemand += instance.get_customer_demand(route[nodeIndex]);
        }
        assert(recomputedDemand == individual.demand_sum[routeIndex]);
        assert(recomputedDemand <= instance.maxC);
        recomputedCost += instance.fitness_evaluation(route);
    }

    const double tolerance = 1e-7 * std::max(1.0, std::fabs(recomputedCost));
    assert(std::fabs(recomputedCost - individual.get_upper_cost()) <= tolerance);
}

void assert_inter_route_relocate_removes_empty_route(Case& instance) {
    int firstCustomer = -1;
    int secondCustomer = -1;
    for (std::size_t first = 0;
         first < instance.customers.size() && firstCustomer < 0;
         ++first) {
        for (std::size_t second = first + 1;
             second < instance.customers.size();
             ++second) {
            const int firstCandidate = instance.customers[first];
            const int secondCandidate = instance.customers[second];
            if (instance.get_customer_demand(firstCandidate)
                    + instance.get_customer_demand(secondCandidate) > instance.maxC) {
                continue;
            }

            const std::vector<int> firstRoute = {
                instance.depot,
                firstCandidate,
                instance.depot};
            const std::vector<int> secondRoute = {
                instance.depot,
                secondCandidate,
                instance.depot};
            const std::vector<int> mergedRoute = {
                instance.depot,
                firstCandidate,
                secondCandidate,
                instance.depot};
            if (instance.fitness_evaluation(mergedRoute) + 1e-8
                < instance.fitness_evaluation(firstRoute)
                    + instance.fitness_evaluation(secondRoute)) {
                firstCustomer = firstCandidate;
                secondCustomer = secondCandidate;
                break;
            }
        }
    }
    assert(firstCustomer >= 0 && secondCustomer >= 0);

    const std::vector<std::vector<int>> routes = {
        {instance.depot, firstCustomer, instance.depot},
        {instance.depot, secondCustomer, instance.depot},
    };
    Individual individual(
        2,
        instance.customerNumber + 2,
        routes,
        instance.fitness_evaluation(routes),
        instance.compute_demand_sum(routes));
    // Seed 2 selects inter-route relocate first from the five-neighborhood list.
    std::default_random_engine randomEngine(2);
    Leader::improve_with_five_neighborhood_rvnd(individual, instance, randomEngine);

    assert(individual.route_num == 1);
    std::vector<int> actualCustomers = individual.get_chromosome();
    std::vector<int> expectedCustomers = {firstCustomer, secondCustomer};
    std::sort(actualCustomers.begin(), actualCustomers.end());
    std::sort(expectedCustomers.begin(), expectedCustomers.end());
    assert(actualCustomers == expectedCustomers);
    assert(individual.demand_sum[0]
           == instance.get_customer_demand(firstCustomer)
               + instance.get_customer_demand(secondCustomer));
    const auto mergedRoutes = individual.get_routes();
    assert(mergedRoutes.size() == 1);
    assert(std::fabs(
        instance.fitness_evaluation(mergedRoutes[0])
        - individual.get_upper_cost()) <= 1e-8);
}

}  // namespace

int main(int argc, char* argv[]) {
    const std::string instanceName = argc > 1 ? argv[1] : "E-n22-k4.evrp";
    const std::string instancePath = std::string(TEST_DATA_DIRECTORY) + "/" + instanceName;
    Case instance(instancePath, 1);
    std::default_random_engine randomEngine(1);
    assert_inter_route_relocate_removes_empty_route(instance);

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
    assert_individual_is_consistent(individual, instance);

    Individual fiveNeighborhoodRvnd(vndBaseline);
    Individual repeatedFiveNeighborhoodRvnd(vndBaseline);
    const double upperCostBeforeFiveNeighborhoodSearch = fiveNeighborhoodRvnd.get_upper_cost();
    std::default_random_engine firstFiveNeighborhoodEngine(11);
    std::default_random_engine secondFiveNeighborhoodEngine(11);
    Leader::improve_with_five_neighborhood_rvnd(
        fiveNeighborhoodRvnd,
        instance,
        firstFiveNeighborhoodEngine);
    Leader::improve_with_five_neighborhood_rvnd(
        repeatedFiveNeighborhoodRvnd,
        instance,
        secondFiveNeighborhoodEngine);
    assert(fiveNeighborhoodRvnd.get_upper_cost()
           <= upperCostBeforeFiveNeighborhoodSearch + 1e-8);
    assert(std::fabs(
        fiveNeighborhoodRvnd.get_upper_cost()
        - repeatedFiveNeighborhoodRvnd.get_upper_cost()) <= 1e-8);
    assert(fiveNeighborhoodRvnd.get_routes()
           == repeatedFiveNeighborhoodRvnd.get_routes());
    assert_individual_is_consistent(fiveNeighborhoodRvnd, instance);

    Individual sevenNeighborhoodRvnd(fiveNeighborhoodRvnd);
    Individual repeatedSevenNeighborhoodRvnd(fiveNeighborhoodRvnd);
    const double upperCostBeforeSevenNeighborhoodSearch =
        sevenNeighborhoodRvnd.get_upper_cost();
    const int routesBeforeSevenNeighborhoodSearch = sevenNeighborhoodRvnd.route_num;
    std::default_random_engine firstSevenNeighborhoodEngine(13);
    std::default_random_engine secondSevenNeighborhoodEngine(13);
    Leader::improve_with_seven_neighborhood_rvnd(
        sevenNeighborhoodRvnd,
        instance,
        firstSevenNeighborhoodEngine);
    Leader::improve_with_seven_neighborhood_rvnd(
        repeatedSevenNeighborhoodRvnd,
        instance,
        secondSevenNeighborhoodEngine);
    assert(sevenNeighborhoodRvnd.get_upper_cost()
           <= upperCostBeforeSevenNeighborhoodSearch + 1e-8);
    assert(sevenNeighborhoodRvnd.route_num <= routesBeforeSevenNeighborhoodSearch);
    assert(std::fabs(
        sevenNeighborhoodRvnd.get_upper_cost()
        - repeatedSevenNeighborhoodRvnd.get_upper_cost()) <= 1e-8);
    assert(sevenNeighborhoodRvnd.get_routes()
           == repeatedSevenNeighborhoodRvnd.get_routes());
    assert_individual_is_consistent(sevenNeighborhoodRvnd, instance);

    Follower::optimize_charging(sevenNeighborhoodRvnd, instance);
    assert(std::isfinite(sevenNeighborhoodRvnd.get_lower_cost()));
    if (instance.customerNumber <= 30) {
        Follower::refine_charging_by_enumeration(sevenNeighborhoodRvnd, instance);
        assert(std::isfinite(sevenNeighborhoodRvnd.get_lower_cost()));
    }

    std::vector<std::shared_ptr<Individual>> rankedSolutions;
    rankedSolutions.push_back(std::make_shared<Individual>(sevenNeighborhoodRvnd));
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
