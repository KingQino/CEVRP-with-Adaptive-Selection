#include "initializer.hpp"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstring>
#include <numeric>

#include "case.hpp"

namespace {

std::vector<std::vector<int>> cluster_customers(
    const Case& instance,
    std::mt19937& randomEngine) {
    std::vector<int> customers(instance.customers);
    std::shuffle(customers.begin(), customers.end(), randomEngine);

    std::vector<std::vector<int>> routes;
    std::vector<int> route;
    while (!customers.empty()) {
        route.clear();

        const int anchor = customers.front();
        customers.erase(customers.begin());
        route.push_back(anchor);
        int load = instance.get_customer_demand(anchor);

        const std::vector<int>& nearbyCustomers = instance.customerClustersMap.at(anchor);
        for (int node : nearbyCustomers) {
            auto customerIt = std::find(customers.begin(), customers.end(), node);
            if (customerIt == customers.end()) {
                continue;
            }
            if (load + instance.get_customer_demand(node) <= instance.maxC) {
                route.push_back(node);
                load += instance.get_customer_demand(node);
                customers.erase(customerIt);
            } else {
                routes.push_back(route);
                break;
            }
        }
    }

    routes.push_back(route);
    return routes;
}

void balance_last_route(
    std::vector<std::vector<int>>& routes,
    const Case& instance,
    std::mt19937& randomEngine) {
    std::vector<int>& lastRoute = routes.back();

    std::uniform_int_distribution<> distribution(0, static_cast<int>(lastRoute.size()) - 1);
    const int customer = lastRoute[distribution(randomEngine)];

    int lastRouteLoad = 0;
    for (int node : lastRoute) {
        lastRouteLoad += instance.get_customer_demand(node);
    }

    for (int candidate : instance.customerClustersMap.at(customer)) {
        if (std::find(lastRoute.begin(), lastRoute.end(), candidate) != lastRoute.end()) {
            continue;
        }

        auto sourceRouteIt = std::find_if(
            routes.begin(),
            routes.end(),
            [candidate](const std::vector<int>& route) {
                return std::find(route.begin(), route.end(), candidate) != route.end();
            });

        if (sourceRouteIt != routes.end()) {
            std::vector<int>& sourceRoute = *sourceRouteIt;
            int sourceRouteLoad = 0;
            for (int node : sourceRoute) {
                sourceRouteLoad += instance.get_customer_demand(node);
            }

            const int candidateDemand = instance.get_customer_demand(candidate);
            const bool capacityFeasible = candidateDemand + lastRouteLoad <= instance.maxC;
            const bool improvesBalance =
                std::abs((lastRouteLoad + candidateDemand) - (sourceRouteLoad - candidateDemand))
                < std::abs(lastRouteLoad - sourceRouteLoad);
            if (capacityFeasible && improvesBalance) {
                sourceRoute.erase(
                    std::remove(sourceRoute.begin(), sourceRoute.end(), candidate),
                    sourceRoute.end());
                lastRoute.push_back(candidate);
                lastRouteLoad += candidateDemand;
            } else {
                break;
            }
        }
    }
}

}  // namespace

std::vector<std::vector<int>> Initializer::split_giant_tour(
    const std::vector<int>& giantTour,
    Case& instance) {
    const int arrayLength = instance.customerNumber + 1;
    auto* predecessors = new int[arrayLength];
    auto* costs = new double[arrayLength];
    std::memset(predecessors, 0, sizeof(int) * arrayLength);
    costs[0] = 0;
    for (int i = 1; i < arrayLength; ++i) {
        costs[i] = DBL_MAX;
    }

    for (int i = 1; i < static_cast<int>(giantTour.size()); ++i) {
        int load = 0;
        double routeCost = 0;
        int j = i;
        do {
            load += instance.get_customer_demand(giantTour[j]);
            if (i == j) {
                routeCost = instance.get_distance(instance.depot, giantTour[j]) * 2;
            } else {
                routeCost -= instance.get_distance(giantTour[j - 1], instance.depot);
                routeCost += instance.get_distance(giantTour[j - 1], giantTour[j]);
                routeCost += instance.get_distance(instance.depot, giantTour[j]);
            }

            if (load <= instance.maxC) {
                if (costs[i - 1] + routeCost < costs[j]) {
                    costs[j] = costs[i - 1] + routeCost;
                    predecessors[j] = i - 1;
                }
                ++j;
            }
        } while (!(j >= static_cast<int>(giantTour.size()) || load > instance.maxC));
    }

    std::vector<std::vector<int>> routes;
    int j = static_cast<int>(giantTour.size()) - 1;
    while (true) {
        const int i = predecessors[j];
        routes.emplace_back(giantTour.begin() + i + 1, giantTour.begin() + j + 1);
        j = i;
        if (i == 0) {
            break;
        }
    }

    delete[] predecessors;
    delete[] costs;
    return routes;
}

std::vector<std::vector<int>> Initializer::build_with_random_split(
    Case& instance,
    std::mt19937& randomEngine) {
    std::vector<int> giantTour(instance.customers);
    std::shuffle(giantTour.begin(), giantTour.end(), randomEngine);
    giantTour.insert(giantTour.begin(), instance.depot);

    std::vector<std::vector<int>> routes = split_giant_tour(giantTour, instance);
    for (auto& route : routes) {
        route.insert(route.begin(), instance.depot);
        route.push_back(instance.depot);
    }
    return routes;
}

std::vector<std::vector<int>> Initializer::build_with_clustering(
    const Case& instance,
    std::mt19937& randomEngine) {
    std::vector<std::vector<int>> routes = cluster_customers(instance, randomEngine);
    balance_last_route(routes, instance, randomEngine);

    for (auto& route : routes) {
        route.insert(route.begin(), instance.depot);
        route.push_back(instance.depot);
    }
    return routes;
}

std::vector<std::vector<int>> Initializer::build_with_direct_encoding(
    const Case& instance,
    std::mt19937& randomEngine) {
    std::vector<int> customers(instance.customers);
    int vehicleIndex = 0;
    int currentLoad = 0;
    std::vector<int> route = {instance.depot};

    std::vector<std::vector<int>> routes;
    while (!customers.empty()) {
        std::vector<int> feasibleNextNodes;
        for (int customer : customers) {
            if (instance.get_customer_demand(customer) <= instance.maxC - currentLoad) {
                feasibleNextNodes.push_back(customer);
            }
        }

        const int remainingDemand = std::accumulate(
            customers.begin(),
            customers.end(),
            0,
            [&](int total, int customer) {
                return total + instance.get_customer_demand(customer);
            });
        if (remainingDemand <= instance.maxC * (instance.vehicleNumber - vehicleIndex - 1)
            || feasibleNextNodes.empty()) {
            feasibleNextNodes.push_back(instance.depot);
        }

        std::uniform_int_distribution<> distribution(
            0,
            static_cast<int>(feasibleNextNodes.size()) - 1);
        const int nextNode = feasibleNextNodes[distribution(randomEngine)];
        route.push_back(nextNode);

        if (nextNode == instance.depot) {
            routes.push_back(route);
            ++vehicleIndex;
            route = {instance.depot};
            currentLoad = 0;
        } else {
            currentLoad += instance.get_customer_demand(nextNode);
            customers.erase(
                std::remove(customers.begin(), customers.end(), nextNode),
                customers.end());
        }
    }

    route.push_back(instance.depot);
    routes.push_back(route);
    return routes;
}
