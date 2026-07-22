//
// Created by Yinghao Qin on 16/11/2023.
//

#include "individual.hpp"

#include <algorithm>
#include <stdexcept>

using namespace std;

const int Individual::TOUR_SIZE = 1500;

Individual::Individual(const Individual& ind)
    : Individual(ind.route_cap, ind.node_cap) {
    copy_from(ind);
}

Individual::Individual(int route_cap, int node_cap) {
    this->route_cap = route_cap;
    this->node_cap = node_cap;
    this->routes = new int *[route_cap];
    for (int i = 0; i < route_cap; ++i) {
        this->routes[i] = new int[node_cap];
    }
    this->route_num = 0;
    this->node_num = new int[route_cap]();
    this->demand_sum = new int [route_cap]();
    this->upper_cost = 0.0;
    this->lower_cost = std::numeric_limits<double>::infinity();
    this->upper_locally_optimal = false;
    this->tour_capacity = TOUR_SIZE;
    this->tour = new int[this->tour_capacity];
    this->steps = 0;
}

Individual::Individual(int route_cap, int node_cap, const vector<vector<int>>& _routes, double upper_cost, const vector<int>& demand_sum)
:Individual(route_cap, node_cap) {
    load_upper_solution(_routes, upper_cost, demand_sum);
}

Individual::~Individual() {
    for (int i = 0; i < this->route_cap; ++i) {
        delete[] this->routes[i];
    }
    delete[] this->routes;
    delete[] this->node_num;
    delete[] this->demand_sum;
    delete[] this->tour;
}


void Individual::reset() {
    memset(this->node_num, 0, sizeof(int) * this->route_cap);
    memset(this->demand_sum, 0, sizeof(int) * this->route_cap);
    this->upper_cost = 0.0;
    this->lower_cost = std::numeric_limits<double>::infinity();
    this->upper_locally_optimal = false;
    this->route_num = 0;
    this->steps = 0;
}

vector<vector<int>> Individual::get_routes() const {
    vector<vector<int>> all_routes(route_num);

    for (int i = 0; i < route_num; ++i) {
        all_routes[i].resize(node_num[i]);
        for (int j = 0; j < node_num[i]; ++j) {
            all_routes[i][j] = routes[i][j];
        }
    }

    return all_routes;
}


vector<int> Individual::get_chromosome() const {
    vector<int> chromosome; // num of customers
    int chromosomeSize = 0;
    for (int i = 0; i < route_num; ++i) {
        chromosomeSize += std::max(0, node_num[i] - 2);
    }
    chromosome.reserve(chromosomeSize);
    for (int i = 0; i < route_num; ++i) {
        for (int j = 1; j < node_num[i] - 1; ++j) {
            chromosome.push_back(routes[i][j]);
        }
    }
    return chromosome;
}


double Individual::get_upper_cost() const {
    return upper_cost;
}

double Individual::get_lower_cost() const {
    return lower_cost;
}

bool Individual::is_upper_locally_optimal() const {
    return upper_locally_optimal;
}

void Individual::set_upper_cost(double cost) {
    this->upper_cost = cost;
    this->upper_locally_optimal = false;
    invalidate_lower_cost();
}

void Individual::set_lower_cost(double cost) {
    this->lower_cost = cost;
}

void Individual::set_upper_locally_optimal(bool locally_optimal) {
    this->upper_locally_optimal = locally_optimal;
}

void Individual::invalidate_lower_cost() {
    this->lower_cost = std::numeric_limits<double>::infinity();
    this->steps = 0;
}

void Individual::load_upper_solution(
    const vector<vector<int>>& new_routes,
    double cost,
    const vector<int>& route_demand_sum) {
    if (new_routes.size() > static_cast<size_t>(route_cap)) {
        throw std::length_error("upper solution exceeds route capacity");
    }
    if (route_demand_sum.size() != new_routes.size()) {
        throw std::invalid_argument("route demand count does not match route count");
    }
    for (const auto& route : new_routes) {
        if (route.size() > static_cast<size_t>(node_cap)) {
            throw std::length_error("upper route exceeds node capacity");
        }
    }

    reset();
    upper_cost = cost;
    route_num = static_cast<int>(new_routes.size());
    for (int routeIndex = 0; routeIndex < route_num; ++routeIndex) {
        node_num[routeIndex] = static_cast<int>(new_routes[routeIndex].size());
        demand_sum[routeIndex] = route_demand_sum[routeIndex];
        if (node_num[routeIndex] > 0) {
            memcpy(
                routes[routeIndex],
                new_routes[routeIndex].data(),
                sizeof(int) * node_num[routeIndex]);
        }
    }
}

void Individual::copy_from(const Individual& other) {
    if (this == &other) {
        return;
    }
    if (route_cap != other.route_cap || node_cap != other.node_cap) {
        throw std::invalid_argument("individual capacities do not match");
    }

    reset();
    route_num = other.route_num;
    upper_cost = other.upper_cost;
    lower_cost = other.lower_cost;
    upper_locally_optimal = other.upper_locally_optimal;
    for (int routeIndex = 0; routeIndex < route_num; ++routeIndex) {
        node_num[routeIndex] = other.node_num[routeIndex];
        demand_sum[routeIndex] = other.demand_sum[routeIndex];
        if (node_num[routeIndex] > 0) {
            memcpy(
                routes[routeIndex],
                other.routes[routeIndex],
                sizeof(int) * node_num[routeIndex]);
        }
    }

    ensure_tour_capacity(other.steps);
    steps = other.steps;
    if (steps > 0) {
        memcpy(tour, other.tour, sizeof(int) * steps);
    }
}

void Individual::set_routes(const vector<vector<int>>& _routes) {
    for (size_t i = 0; i < _routes.size(); ++i) {
        for (size_t j = 0; j < _routes[i].size(); ++j) {
            this->routes[i][j] = _routes[i][j];
        }
    }
    this->upper_locally_optimal = false;
    invalidate_lower_cost();
}


pair<int*, int> Individual::get_tour() {
    return make_pair(this->tour, this->steps);
}

void Individual::set_tour(const vector<vector<int>>& repaired_routes) {
    std::size_t requiredCapacity = 1;
    for (const auto& route : repaired_routes) {
        if (!route.empty()) {
            requiredCapacity += route.size() - 1;
        }
    }
    if (requiredCapacity > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        throw std::length_error("repaired tour is too large");
    }
    ensure_tour_capacity(static_cast<int>(requiredCapacity));

    int index = 0;
    for (const auto& route : repaired_routes) {
        for (size_t i = 0; i + 1 < route.size(); ++i) {
            this->tour[index++] = route[i];
        }
    }
    this->tour[index++] = 0; // DEPOT
    this->steps = index;
}

void Individual::ensure_tour_capacity(int required_capacity) {
    if (required_capacity <= this->tour_capacity) {
        return;
    }

    const int newCapacity = std::max(required_capacity, this->tour_capacity * 2);
    int* expandedTour = new int[newCapacity];
    if (this->steps > 0) {
        memcpy(expandedTour, this->tour, sizeof(int) * this->steps);
    }
    delete[] this->tour;
    this->tour = expandedTour;
    this->tour_capacity = newCapacity;
}


std::ostream& operator<<(std::ostream& os, const Individual& individual) {
    os << "Route Capacity: " << individual.route_cap << "\n";
    os << "Node Capacity: " << individual.node_cap << "\n";
    os << "Number of Routes: " << individual.route_num << "\n";
    os << "Upper cost: " << individual.upper_cost << "\n";
    os << "Lower cost: " << individual.lower_cost << "\n";

    os << "Number of Nodes per route: ";
    for (int i = 0; i < individual.route_cap; ++i) {
        os << individual.node_num[i] << " ";
    }
    os << "\n";

    os << "Demand sum per route: ";
    for (int i = 0; i < individual.route_cap; ++i) {
        os << individual.demand_sum[i] << " ";
    }
    os << "\n";

    for (int i = 0; i < individual.route_num; ++i) {
        os << "Route " << i + 1 << ": ";
        for (int j = 0; j < individual.node_num[i]; ++j) {
            os << individual.routes[i][j] << " ";
        }
        os << "\n";
    }

    os << "Tour: ";
    for (int i = 0; i < individual.steps; ++i) {
        os << individual.tour[i] << " ";
    }
    os << "\n";

    return os;
}
