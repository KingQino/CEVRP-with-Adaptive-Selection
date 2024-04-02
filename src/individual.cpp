//
// Created by Yinghao Qin on 16/11/2023.
//

#include "../include/individual.hpp"

const int Individual::TOUR_SIZE = 1500;

Individual::Individual(const Individual& ind) {
    this->route_cap = ind.route_cap;
    this->node_cap = ind.node_cap;
    this->route_num = ind.route_num;
    this->fit = ind.fit;
    this->routes = new int *[ind.route_cap];
    for (int i = 0; i < ind.route_cap; ++i) {
        this->routes[i] = new int[ind.node_cap];
        memcpy(this->routes[i], ind.routes[i], sizeof(int) * ind.node_cap);
    }
    this->node_num = new int[ind.route_cap];
    memcpy(this->node_num, ind.node_num, sizeof(int) * ind.route_cap);
    this->demand_sum = new int[ind.route_cap];
    memcpy(this->demand_sum, ind.demand_sum, sizeof(int) * ind.route_cap);
    this->tour = new int [TOUR_SIZE];
    memcpy(this->tour, ind.tour, sizeof(int) * (ind.steps));
    this->steps = ind.steps;
}

Individual::Individual(int route_cap, int node_cap) {
    this->route_cap = route_cap;
    this->node_cap = node_cap;
    this->routes = new int *[route_cap];
    for (int i = 0; i < route_cap; ++i) {
        this->routes[i] = new int[node_cap];
        memset(this->routes[i], 0, sizeof(int) * node_cap);
    }
    this->route_num = 0;
    this->node_num = new int[route_cap];
    memset(this->node_num, 0, sizeof(int) * route_cap);
    this->demand_sum = new int [route_cap];
    memset(this->demand_sum, 0, sizeof(int) * route_cap);
    this->fit = 0;
    this->tour = new int[TOUR_SIZE];
    memset(this->tour, 0, sizeof(int) * TOUR_SIZE);
    this->steps = 0;
}

Individual::Individual(int route_cap, int node_cap, const vector<vector<int>>& _routes, double fit, const vector<int>& demand_sum)
:Individual(route_cap, node_cap) {
    this->fit = fit;
    this->route_num = _routes.size();
    for (int i = 0; i < this->route_num; ++i) {
        this->node_num[i] = _routes[i].size();
        for (int j = 0; j < this->node_num[i]; ++j) {
            this->routes[i][j] = _routes[i][j];
        }
    }
    for (int i = 0; i < demand_sum.size(); ++i) {
        this->demand_sum[i] = demand_sum[i];
    }
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
    this->fit = 0;
    this->route_num = 0;
    memset(this->tour, 0, sizeof(int) * TOUR_SIZE);
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
    for (int i = 0; i < route_num; ++i) {
        for (int j = 1; j < node_num[i] - 1; ++j) {
            chromosome.push_back(routes[i][j]);
        }
    }
    return chromosome;
}


double Individual::get_fit() const {
    return fit;
}

void Individual::set_fit(double _fit) {
    this->fit = _fit;
}


void Individual::set_routes(const vector<vector<int>>& _routes) const {
    for (int i = 0; i < _routes.size(); ++i) {
        for (int j = 0; j < _routes[i].size(); ++j) {
            this->routes[i][j] = _routes[i][j];
        }
    }
}


pair<int*, int> Individual::get_tour() {
    return make_pair(this->tour, this->steps);
}

void Individual::set_tour(const vector<vector<int>>& repaired_routes) {
    int index = 0;
    for (const auto& route : repaired_routes) {
        for (int i = 0; i < route.size() - 1; ++i) {
            this->tour[index++] = route[i];
        }
    }
    this->tour[index++] = 0; // DEPOT
    this->steps = index;
}


std::ostream& operator<<(std::ostream& os, const Individual& individual) {
    os << "Route Capacity: " << individual.route_cap << "\n";
    os << "Node Capacity: " << individual.node_cap << "\n";
    os << "Number of Routes: " << individual.route_num << "\n";
    os << "Fitness: " << individual.fit << "\n";

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

    for (int i = 0; i < individual.route_cap; ++i) {
        os << "Route " << i + 1 << ": ";
        for (int j = 0; j < individual.node_cap; ++j) {
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


