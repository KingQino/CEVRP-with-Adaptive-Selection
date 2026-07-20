#include "../include/follower.hpp"

#include <cfloat>
#include <cmath>
#include <list>
#include <numeric>
#include <utility>
#include <vector>

#include "../include/algorithm_constants.hpp"
#include "../include/case.hpp"
#include "../include/individual.hpp"

namespace {

using RouteRepair = std::pair<double, std::vector<int>>;

RouteRepair enumerate_best_station_per_edge(int* route, int length, Case& instance);
RouteRepair insert_then_remove_redundant_stations(int* route, int length, Case& instance);
RouteRepair enumerate_all_station_choices(std::vector<int>& route, Case& instance);
std::pair<std::vector<int>, double> enumerate_exact_station_choices(
    std::vector<int>& route,
    Case& instance);

void enumerate_station_positions(
    int minimumPosition,
    int remainingPositions,
    int* chosenPositions,
    int* bestChosenPositions,
    double& bestCost,
    int stationCount,
    int* route,
    int length,
    std::vector<double>& cumulativeDistance,
    Case& instance);

void enumerate_stations_and_positions(
    int minimumPosition,
    int remainingPositions,
    int* chosenStations,
    int* chosenPositions,
    std::vector<int>& bestRoute,
    double& bestCost,
    int stationCount,
    std::vector<int>& route,
    std::vector<double>& cumulativeDistance,
    Case& instance);

RouteRepair enumerate_best_station_per_edge(int* route, int length, Case& instance) {
    std::vector<int> repairedRoute;
    std::vector<double> cumulativeDistance(length, 0);
    for (int i = 1; i < length; ++i) {
        cumulativeDistance[i] = cumulativeDistance[i - 1]
            + instance.get_distance(route[i], route[i - 1]);
    }
    if (cumulativeDistance.back() <= instance.maxDis) {
        repairedRoute.insert(repairedRoute.end(), route, route + length);
        return std::make_pair(cumulativeDistance.back(), repairedRoute);
    }

    const int upperBound = static_cast<int>(cumulativeDistance.back() / instance.maxDis + 1);
    const int lowerBound = static_cast<int>(cumulativeDistance.back() / instance.maxDis);
    int* chosenPositions = new int[length];
    int* bestChosenPositions = new int[length];
    double bestCostForCount = DBL_MAX;
    double bestCost = bestCostForCount;

    for (int stationCount = lowerBound; stationCount <= upperBound; ++stationCount) {
        enumerate_station_positions(
            0,
            stationCount,
            chosenPositions,
            bestChosenPositions,
            bestCostForCount,
            stationCount,
            route,
            length,
            cumulativeDistance,
            instance);

        if (bestCostForCount < bestCost) {
            repairedRoute.clear();
            int routeStart = 0;
            for (int j = 0; j < stationCount; ++j) {
                const int from = route[bestChosenPositions[j]];
                const int to = route[bestChosenPositions[j] + 1];
                const int station = instance.bestStation[from][to];
                repairedRoute.insert(
                    repairedRoute.end(),
                    route + routeStart,
                    route + bestChosenPositions[j] + 1);
                repairedRoute.push_back(station);
                routeStart = bestChosenPositions[j] + 1;
            }
            repairedRoute.insert(repairedRoute.end(), route + routeStart, route + length);
            bestCost = bestCostForCount;
        }
    }

    delete[] chosenPositions;
    delete[] bestChosenPositions;
    if (bestCostForCount != DBL_MAX) {
        return std::make_pair(bestCostForCount, repairedRoute);
    }
    return std::make_pair(-1, repairedRoute);
}

RouteRepair insert_then_remove_redundant_stations(int* route, int length, Case& instance) {
    std::vector<int> repairedRoute;
    std::list<std::pair<int, int>> insertedStations;

    for (int i = 0; i < length - 1; ++i) {
        double availableRange = instance.maxDis;
        if (i != 0) {
            availableRange = instance.maxDis
                - instance.get_distance(insertedStations.back().second, route[i]);
        }
        const int station = instance.find_best_station_feasible(
            route[i],
            route[i + 1],
            availableRange);
        if (station == -1) {
            return std::make_pair(-1, repairedRoute);
        }
        insertedStations.push_back(std::make_pair(i, station));
    }

    while (!insertedStations.empty()) {
        bool changed = false;
        auto stationToRemove = insertedStations.begin();
        double savedDistance = 0;
        auto stationIt = insertedStations.begin();
        auto nextStationIt = stationIt;
        ++nextStationIt;

        if (nextStationIt != insertedStations.end()) {
            const int endIndex = nextStationIt->first;
            const int endStation = nextStationIt->second;
            double segmentDistance = 0;
            for (int i = 0; i < endIndex; ++i) {
                segmentDistance += instance.get_distance(route[i], route[i + 1]);
            }
            segmentDistance += instance.get_distance(route[endIndex], endStation);
            if (segmentDistance <= instance.maxDis) {
                savedDistance = instance.get_distance(route[stationIt->first], stationIt->second)
                    + instance.get_distance(stationIt->second, route[stationIt->first + 1])
                    - instance.get_distance(route[stationIt->first], route[stationIt->first + 1]);
            }
        } else {
            double segmentDistance = 0;
            for (int i = 0; i < length - 1; ++i) {
                segmentDistance += instance.get_distance(route[i], route[i + 1]);
            }
            if (segmentDistance <= instance.maxDis) {
                savedDistance = instance.get_distance(route[stationIt->first], stationIt->second)
                    + instance.get_distance(stationIt->second, route[stationIt->first + 1])
                    - instance.get_distance(route[stationIt->first], route[stationIt->first + 1]);
            }
        }

        ++stationIt;
        while (stationIt != insertedStations.end()) {
            nextStationIt = stationIt;
            ++nextStationIt;
            auto previousStationIt = stationIt;
            --previousStationIt;
            double segmentDistance = 0;

            if (nextStationIt != insertedStations.end()) {
                const int startIndex = previousStationIt->first + 1;
                const int endIndex = nextStationIt->first;
                segmentDistance += instance.get_distance(previousStationIt->second, route[startIndex]);
                for (int i = startIndex; i < endIndex; ++i) {
                    segmentDistance += instance.get_distance(route[i], route[i + 1]);
                }
                segmentDistance += instance.get_distance(route[endIndex], nextStationIt->second);
                if (segmentDistance <= instance.maxDis) {
                    const double candidateSaving =
                        instance.get_distance(route[stationIt->first], stationIt->second)
                        + instance.get_distance(stationIt->second, route[stationIt->first + 1])
                        - instance.get_distance(route[stationIt->first], route[stationIt->first + 1]);
                    if (candidateSaving > savedDistance) {
                        savedDistance = candidateSaving;
                        stationToRemove = stationIt;
                    }
                }
            } else {
                const int startIndex = previousStationIt->first + 1;
                segmentDistance += instance.get_distance(previousStationIt->second, route[startIndex]);
                for (int i = startIndex; i < length - 1; ++i) {
                    segmentDistance += instance.get_distance(route[i], route[i + 1]);
                }
                if (segmentDistance <= instance.maxDis) {
                    const double candidateSaving =
                        instance.get_distance(route[stationIt->first], stationIt->second)
                        + instance.get_distance(stationIt->second, route[stationIt->first + 1])
                        - instance.get_distance(route[stationIt->first], route[stationIt->first + 1]);
                    if (candidateSaving > savedDistance) {
                        savedDistance = candidateSaving;
                        stationToRemove = stationIt;
                    }
                }
            }
            ++stationIt;
        }

        if (savedDistance != 0) {
            insertedStations.erase(stationToRemove);
            changed = true;
        }
        if (!changed) {
            break;
        }
    }

    double routeCost = 0;
    for (int i = 0; i < length - 1; ++i) {
        routeCost += instance.get_distance(route[i], route[i + 1]);
    }
    int routeStart = 0;
    for (const auto& insertedStation : insertedStations) {
        const int position = insertedStation.first;
        const int station = insertedStation.second;
        routeCost -= instance.get_distance(route[position], route[position + 1]);
        routeCost += instance.get_distance(route[position], station);
        routeCost += instance.get_distance(station, route[position + 1]);
        repairedRoute.insert(repairedRoute.end(), route + routeStart, route + position + 1);
        repairedRoute.push_back(station);
        routeStart = position + 1;
    }
    repairedRoute.insert(repairedRoute.end(), route + routeStart, route + length);
    return std::make_pair(routeCost, repairedRoute);
}

void enumerate_station_positions(
    int minimumPosition,
    int remainingPositions,
    int* chosenPositions,
    int* bestChosenPositions,
    double& bestCost,
    int stationCount,
    int* route,
    int length,
    std::vector<double>& cumulativeDistance,
    Case& instance) {
    for (int i = minimumPosition; i <= length - 1 - remainingPositions; ++i) {
        if (stationCount == remainingPositions) {
            const double distanceToStation =
                instance.get_distance(route[i], instance.bestStation[route[i]][route[i + 1]]);
            if (cumulativeDistance[i] + distanceToStation > instance.maxDis) {
                break;
            }
        } else {
            const int lastPosition = chosenPositions[stationCount - remainingPositions - 1];
            const double distanceFromLastStation = instance.get_distance(
                route[lastPosition + 1],
                instance.bestStation[route[lastPosition]][route[lastPosition + 1]]);
            const double distanceToStation =
                instance.get_distance(route[i], instance.bestStation[route[i]][route[i + 1]]);
            if (cumulativeDistance[i] - cumulativeDistance[lastPosition + 1]
                + distanceFromLastStation + distanceToStation > instance.maxDis) {
                break;
            }
        }
        if (remainingPositions == 1) {
            const double finalSegmentDistance = cumulativeDistance.back() - cumulativeDistance[i + 1]
                + instance.get_distance(
                    instance.bestStation[route[i]][route[i + 1]],
                    route[i + 1]);
            if (finalSegmentDistance > instance.maxDis) {
                continue;
            }
        }

        chosenPositions[stationCount - remainingPositions] = i;
        if (remainingPositions > 1) {
            enumerate_station_positions(
                i + 1,
                remainingPositions - 1,
                chosenPositions,
                bestChosenPositions,
                bestCost,
                stationCount,
                route,
                length,
                cumulativeDistance,
                instance);
        } else {
            double routeCost = cumulativeDistance.back();
            for (int j = 0; j < stationCount; ++j) {
                const int from = route[chosenPositions[j]];
                const int to = route[chosenPositions[j] + 1];
                const int station = instance.bestStation[from][to];
                routeCost -= instance.get_distance(from, to);
                routeCost += instance.get_distance(from, station);
                routeCost += instance.get_distance(to, station);
            }
            if (routeCost < bestCost) {
                bestCost = routeCost;
                for (int j = 0; j < length; ++j) {
                    bestChosenPositions[j] = chosenPositions[j];
                }
            }
        }
    }
}

RouteRepair enumerate_all_station_choices(std::vector<int>& route, Case& instance) {
    std::vector<double> cumulativeDistance(route.size(), 0);
    for (int i = 1; i < static_cast<int>(route.size()); ++i) {
        cumulativeDistance[i] = cumulativeDistance[i - 1]
            + instance.get_distance(route[i], route[i - 1]);
    }
    if (cumulativeDistance.back() <= instance.maxDis) {
        return std::make_pair(cumulativeDistance.back(), route);
    }

    const int originalLength = static_cast<int>(route.size());
    const int lowerBound = static_cast<int>(std::floor(cumulativeDistance.back() / instance.maxDis));
    const int upperBound = static_cast<int>(std::ceil(cumulativeDistance.back() / instance.maxDis));
    std::vector<int> chosenPositions(route.size(), 0);
    std::vector<int> chosenStations(route.size(), 0);
    std::vector<int> bestRoute;
    double bestCost = DBL_MAX;

    auto tryStationCount = [&](int stationCount) {
        std::vector<int> candidateRoute;
        double candidateCost = DBL_MAX;
        enumerate_stations_and_positions(
            0,
            stationCount,
            chosenStations.data(),
            chosenPositions.data(),
            candidateRoute,
            candidateCost,
            stationCount,
            route,
            cumulativeDistance,
            instance);
        if (candidateCost < bestCost) {
            bestCost = candidateCost;
            bestRoute = std::move(candidateRoute);
        }
        return candidateCost != DBL_MAX;
    };

    for (int stationCount = lowerBound; stationCount <= upperBound; ++stationCount) {
        tryStationCount(stationCount);
    }
    if (bestCost != DBL_MAX) {
        return std::make_pair(bestCost, bestRoute);
    }

    std::vector<int> removalRoute;
    removalRoute.reserve(route.size() + instance.stationNumber);
    auto removalResult = insert_then_remove_redundant_stations(route.data(), originalLength, instance);
    if (removalResult.first >= 0) {
        removalRoute = std::move(removalResult.second);
        const int feasibleUpperBound = static_cast<int>(removalRoute.size()) - originalLength;
        for (int stationCount = upperBound + 1; stationCount <= feasibleUpperBound; ++stationCount) {
            tryStationCount(stationCount);
        }
        if (bestCost != DBL_MAX) {
            return std::make_pair(bestCost, bestRoute);
        }
    }

    std::vector<int> routeCopy(route);
    auto exactResult = enumerate_exact_station_choices(routeCopy, instance);
    if (exactResult.second >= 0) {
        return std::make_pair(exactResult.second, exactResult.first);
    }

    auto simpleResult = enumerate_best_station_per_edge(route.data(), originalLength, instance);
    if (simpleResult.first >= 0) {
        return simpleResult;
    }
    if (removalResult.first >= 0) {
        return removalResult;
    }
    return std::make_pair(-1.0, std::vector<int>());
}

std::pair<std::vector<int>, double> enumerate_exact_station_choices(
    std::vector<int>& route,
    Case& instance) {
    std::vector<double> cumulativeDistance(route.size(), 0);
    for (int i = 1; i < static_cast<int>(route.size()); ++i) {
        cumulativeDistance[i] = cumulativeDistance[i - 1]
            + instance.get_distance(route[i], route[i - 1]);
    }
    if (cumulativeDistance.back() <= instance.maxDis) {
        return std::make_pair(route, cumulativeDistance.back());
    }

    const int upperBound = static_cast<int>(std::ceil(cumulativeDistance.back() / instance.maxDis));
    const int lowerBound = static_cast<int>(std::floor(cumulativeDistance.back() / instance.maxDis));
    int* chosenPositions = new int[route.size()];
    int* chosenStations = new int[route.size()];
    std::vector<int> bestRoute;
    double bestCost = DBL_MAX;
    for (int stationCount = lowerBound; stationCount <= upperBound; ++stationCount) {
        enumerate_stations_and_positions(
            0,
            stationCount,
            chosenStations,
            chosenPositions,
            bestRoute,
            bestCost,
            stationCount,
            route,
            cumulativeDistance,
            instance);
    }
    delete[] chosenPositions;
    delete[] chosenStations;
    if (bestCost != DBL_MAX) {
        return std::make_pair(bestRoute, bestCost);
    }
    return std::make_pair(route, -1);
}

void enumerate_stations_and_positions(
    int minimumPosition,
    int remainingPositions,
    int* chosenStations,
    int* chosenPositions,
    std::vector<int>& bestRoute,
    double& bestCost,
    int stationCount,
    std::vector<int>& route,
    std::vector<double>& cumulativeDistance,
    Case& instance) {
    for (int i = minimumPosition;
         i <= static_cast<int>(route.size()) - 1 - remainingPositions;
         ++i) {
        if (stationCount == remainingPositions) {
            if (cumulativeDistance[i] >= instance.maxDis) {
                break;
            }
        } else if (cumulativeDistance[i]
                   - cumulativeDistance[chosenPositions[stationCount - remainingPositions - 1] + 1]
                   >= instance.maxDis) {
            break;
        }

        if (remainingPositions == 1
            && cumulativeDistance.back() - cumulativeDistance[i + 1] >= instance.maxDis) {
            continue;
        }

        for (int stationIndex = 0; stationIndex < instance.stationNumber; ++stationIndex) {
            chosenStations[stationCount - remainingPositions] = instance.stations[stationIndex];
            chosenPositions[stationCount - remainingPositions] = i;
            if (remainingPositions > 1) {
                enumerate_stations_and_positions(
                    i + 1,
                    remainingPositions - 1,
                    chosenStations,
                    chosenPositions,
                    bestRoute,
                    bestCost,
                    stationCount,
                    route,
                    cumulativeDistance,
                    instance);
            } else {
                bool feasible = true;
                double segmentDistance = cumulativeDistance[chosenPositions[0]]
                    + instance.get_distance(route[chosenPositions[0]], chosenStations[0]);
                if (segmentDistance > instance.maxDis) {
                    feasible = false;
                }
                for (int k = 1; feasible && k < stationCount; ++k) {
                    segmentDistance = cumulativeDistance[chosenPositions[k]]
                        - cumulativeDistance[chosenPositions[k - 1] + 1];
                    segmentDistance += instance.get_distance(
                        chosenStations[k - 1],
                        route[chosenPositions[k - 1] + 1]);
                    segmentDistance += instance.get_distance(
                        chosenStations[k],
                        route[chosenPositions[k]]);
                    if (segmentDistance > instance.maxDis) {
                        feasible = false;
                    }
                }
                segmentDistance = cumulativeDistance.back()
                    - cumulativeDistance[chosenPositions[stationCount - 1] + 1];
                segmentDistance += instance.get_distance(
                    route[chosenPositions[stationCount - 1] + 1],
                    chosenStations[stationCount - 1]);
                if (segmentDistance > instance.maxDis) {
                    feasible = false;
                }

                if (feasible) {
                    double routeCost = cumulativeDistance.back();
                    for (int k = 0; k < stationCount; ++k) {
                        const int from = route[chosenPositions[k]];
                        const int to = route[chosenPositions[k] + 1];
                        routeCost -= instance.get_distance(from, to);
                        routeCost += instance.get_distance(from, chosenStations[k]);
                        routeCost += instance.get_distance(chosenStations[k], to);
                    }
                    if (routeCost < bestCost) {
                        bestCost = routeCost;
                        bestRoute = route;
                        for (int k = stationCount - 1; k >= 0; --k) {
                            bestRoute.insert(
                                bestRoute.begin() + chosenPositions[k] + 1,
                                chosenStations[k]);
                        }
                    }
                }
            }
        }
    }
}

}  // namespace

void Follower::optimize_charging(Individual& individual, Case& instance) {
    individual.invalidate_lower_cost();
    double lowerCost = 0.0;
    std::vector<std::vector<int>> repairedRoutes;
    bool feasible = true;

    for (int routeIndex = 0; routeIndex < individual.route_num; ++routeIndex) {
        auto result = enumerate_best_station_per_edge(
            individual.routes[routeIndex],
            individual.node_num[routeIndex],
            instance);
        if (result.first == -1) {
            result = insert_then_remove_redundant_stations(
                individual.routes[routeIndex],
                individual.node_num[routeIndex],
                instance);
            if (result.first == -1) {
                lowerCost += INFEASIBLE_COST;
                feasible = false;
            } else {
                lowerCost += result.first;
                repairedRoutes.push_back(std::move(result.second));
            }
        } else {
            lowerCost += result.first;
            repairedRoutes.push_back(std::move(result.second));
        }
    }

    individual.set_lower_cost(lowerCost);
    if (feasible) {
        individual.set_tour(repairedRoutes);
    }
}

void Follower::refine_charging_by_enumeration(Individual& individual, Case& instance) {
    individual.invalidate_lower_cost();
    double lowerCost = 0.0;
    std::vector<std::vector<int>> repairedRoutes;
    bool feasible = true;

    for (int routeIndex = 0; routeIndex < individual.route_num; ++routeIndex) {
        std::vector<int> route(
            individual.routes[routeIndex],
            individual.routes[routeIndex] + individual.node_num[routeIndex]);
        auto result = enumerate_all_station_choices(route, instance);
        if (result.first < 0) {
            lowerCost += INFEASIBLE_COST;
            feasible = false;
            continue;
        }
        lowerCost += result.first;
        repairedRoutes.push_back(std::move(result.second));
    }

    individual.set_lower_cost(lowerCost);
    if (feasible) {
        individual.set_tour(repairedRoutes);
    }
}
