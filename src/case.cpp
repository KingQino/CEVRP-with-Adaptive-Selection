//
// Created by Yinghao Qin on 19/12/2023.
//

#include "case.hpp"
#include <stdexcept>

using namespace std;

const int Case::MAX_EVALUATION_FACTOR = 25000;

Case::Case(const string& filepath, int id) {
    this->ID = id;
    size_t lastSeparatorPos = filepath.find_last_of('/');
    this->fileName = filepath.substr(lastSeparatorPos + 1);
    size_t lastDot = this->fileName.find_last_of('.');
    this->instanceName = this->fileName.substr(0, lastDot);

    read_problem(filepath);
}

Case::~Case() {
    for (int i = 0; i < actualProblemSize; i++) {
        delete[] this->distances[i];
    }
    delete[] this->distances;
    for (int i = 0; i < actualProblemSize; i++) {
        delete[] this->bestStation[i];
    }
    delete[] this->bestStation;
}

void Case::read_problem(const string& filepath) {
    enum class Section {
        Header,
        NodeCoords,
        Demands,
        Stations,
        Depots
    };

    auto trim = [](const string& value) {
        const auto begin = value.find_first_not_of(" \t\r\n");
        if (begin == string::npos) {
            return string();
        }
        const auto end = value.find_last_not_of(" \t\r\n");
        return value.substr(begin, end - begin + 1);
    };

    auto parse_int_after_colon = [&](const string& text, int& target) {
        const auto pos = text.find(':');
        if (pos == string::npos) {
            return false;
        }
        istringstream iss(trim(text.substr(pos + 1)));
        iss >> target;
        return !iss.fail();
    };

    auto parse_double_after_colon = [&](const string& text, double& target) {
        const auto pos = text.find(':');
        if (pos == string::npos) {
            return false;
        }
        istringstream iss(trim(text.substr(pos + 1)));
        iss >> target;
        return !iss.fail();
    };

    this->positions.clear();
    this->demand.clear();
    this->customers.clear();
    this->stations.clear();
    this->stationSet.clear();
    this->customerClustersMap.clear();
    this->customerNearestStationMap.clear();
    this->depotNumber = 1;
    this->depot = 0;

    ifstream infile(filepath.c_str());
    if (!infile.is_open()) {
        throw runtime_error("Failed to open instance file: " + filepath);
    }

    Section section = Section::Header;
    vector<tuple<int, double, double>> nodeCoords;
    unordered_map<int, int> demandByNode;
    vector<int> stationNodes;
    vector<int> depotNodes;
    int maxNodeId = -1;

    string line;
    while (getline(infile, line)) {
        const string trimmed = trim(line);
        if (trimmed.empty()) {
            continue;
        }
        if (trimmed == "EOF") {
            break;
        }
        if (trimmed == "NODE_COORD_SECTION") {
            section = Section::NodeCoords;
            continue;
        }
        if (trimmed == "DEMAND_SECTION") {
            section = Section::Demands;
            continue;
        }
        if (trimmed == "STATIONS_COORD_SECTION") {
            section = Section::Stations;
            continue;
        }
        if (trimmed == "DEPOT_SECTION") {
            section = Section::Depots;
            continue;
        }

        switch (section) {
            case Section::Header:
                if (trimmed.find("VEHICLES:") != string::npos) {
                    parse_int_after_colon(trimmed, this->vehicleNumber);
                } else if (trimmed.find("CAPACITY:") != string::npos && trimmed.find("ENERGY") == string::npos) {
                    parse_int_after_colon(trimmed, this->maxC);
                } else if (trimmed.find("ENERGY_CAPACITY:") != string::npos) {
                    parse_double_after_colon(trimmed, this->maxQ);
                } else if (trimmed.find("ENERGY_CONSUMPTION:") != string::npos) {
                    parse_double_after_colon(trimmed, this->conR);
                } else if (trimmed.find("OPTIMAL_VALUE:") != string::npos) {
                    parse_double_after_colon(trimmed, this->optimum);
                }
                break;
            case Section::NodeCoords: {
                istringstream iss(trimmed);
                int nodeId = -1;
                double x = 0.0;
                double y = 0.0;
                if (iss >> nodeId >> x >> y) {
                    nodeCoords.emplace_back(nodeId - 1, x, y);
                    maxNodeId = std::max(maxNodeId, nodeId - 1);
                }
                break;
            }
            case Section::Demands: {
                istringstream iss(trimmed);
                int nodeId = -1;
                int nodeDemand = 0;
                if (iss >> nodeId >> nodeDemand) {
                    demandByNode[nodeId - 1] = nodeDemand;
                    maxNodeId = std::max(maxNodeId, nodeId - 1);
                }
                break;
            }
            case Section::Stations: {
                istringstream iss(trimmed);
                int nodeId = -1;
                if (iss >> nodeId && nodeId > 0) {
                    stationNodes.push_back(nodeId - 1);
                    maxNodeId = std::max(maxNodeId, nodeId - 1);
                }
                break;
            }
            case Section::Depots: {
                istringstream iss(trimmed);
                int nodeId = -1;
                if (iss >> nodeId && nodeId > 0) {
                    depotNodes.push_back(nodeId - 1);
                    maxNodeId = std::max(maxNodeId, nodeId - 1);
                }
                break;
            }
        }
    }
    infile.close();

    if (nodeCoords.empty()) {
        throw runtime_error("Instance file has no node coordinates: " + filepath);
    }
    if (depotNodes.size() != 1) {
        throw runtime_error("Only single-depot instances are supported: " + filepath);
    }

    sort(stationNodes.begin(), stationNodes.end());
    stationNodes.erase(unique(stationNodes.begin(), stationNodes.end()), stationNodes.end());

    this->actualProblemSize = maxNodeId + 1;
    this->positions.assign(actualProblemSize, make_pair(0.0, 0.0));
    this->demand.assign(actualProblemSize, 0);
    this->depot = depotNodes.front();
    this->depotNumber = 1;
    this->stations = stationNodes;
    this->stationSet.insert(stations.begin(), stations.end());
    this->stationNumber = static_cast<int>(stations.size());

    for (const auto& [nodeId, x, y] : nodeCoords) {
        this->positions[nodeId] = make_pair(x, y);
    }

    this->customers.clear();
    this->totalDem = 0;
    for (const auto& [nodeId, nodeDemand] : demandByNode) {
        this->demand[nodeId] = nodeDemand;
        if (nodeId != depot) {
            this->customers.push_back(nodeId);
            this->totalDem += nodeDemand;
        }
    }
    sort(this->customers.begin(), this->customers.end());
    this->customerNumber = static_cast<int>(customers.size());

    this->maxDis = maxQ / conR;

    this->distances = generate_2D_matrix_double(actualProblemSize, actualProblemSize);
    for (int i = 0; i < actualProblemSize; i++) {
        for (int j = 0; j < actualProblemSize; j++) {
            distances[i][j] = euclidean_distance(i, j);
        }
    }

    this->bestStation = new int* [actualProblemSize];
    for (int i = 0; i < actualProblemSize; i++) {
        this->bestStation[i] = new int[actualProblemSize];
        memset(this->bestStation[i], 0, sizeof(int) * actualProblemSize);
    }

    vector<int> routeNodes = customers;
    routeNodes.insert(routeNodes.begin(), depot);
    for (int a = 0; a < static_cast<int>(routeNodes.size()) - 1; ++a) {
        for (int b = a + 1; b < static_cast<int>(routeNodes.size()); ++b) {
            const int from = routeNodes[a];
            const int to = routeNodes[b];
            this->bestStation[from][to] = this->bestStation[to][from] = find_best_station(from, to);
        }
    }

    init_customer_clusters_map();
    init_customer_nearest_station_map();

    this->evals = 0.0;
    this->maxEvals = actualProblemSize * MAX_EVALUATION_FACTOR;
    if (customerNumber <= 100) {
        maxExecTime = int (1 * (actualProblemSize / 100.0) * 60 * 60);
    } else if (customerNumber <= 915) {
        maxExecTime = int (2 * (actualProblemSize / 100.0) * 60 * 60);
    } else {
        maxExecTime = int (3 * (actualProblemSize / 100.0) * 60 * 60);
    }
}


double Case::euclidean_distance(int i, int j) {
    double xd, yd;
    double r = 0.0;
    xd = positions[i].first - positions[j].first;
    yd = positions[i].second - positions[j].second;
    r = sqrt(xd * xd + yd * yd);
    return r;
}

void Case::init_customer_clusters_map() {
    for (int node : customers) {
        vector<int> other_customers;
        for (int x : customers) {
            if (x != node) {
                other_customers.push_back(x);
            }
        }

        sort(other_customers.begin(), other_customers.end(), [&](int i, int j) {
            return distances[node][i]  < distances[node][j];
        });

        customerClustersMap[node] = other_customers;
    }
}

void Case::init_customer_nearest_station_map() {
    for (int customer : customers) {
        int nearestStation = -1;
        double minDis = DBL_MAX;
        for (int station : stations) {
            double dis = distances[customer][station];
            if (minDis > dis) {
                nearestStation = station;
                minDis = dis;
            }
        }
        customerNearestStationMap[customer] = make_pair(nearestStation, minDis);
    }
}

double **Case::generate_2D_matrix_double(int n, int m) {
    double **matrix;

    matrix = new double *[n];
    for (int i = 0; i < n; i++) {
        matrix[i] = new double[m];
    }
    //initialize the 2-d array
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            matrix[i][j] = 0.0;
        }
    }
    return matrix;
}

int Case::get_customer_demand(int customer) const {
    return demand[customer];
}

double Case::get_distance(int from, int to) {
    //adds partial evaluation to the overall fitness evaluation count
    //It can be used when local search is used and a whole evaluation is not necessary
    evals += (1.0 / actualProblemSize);

    return distances[from][to];
}

double Case::get_evals() const {
    return evals;
}

double Case::fitness_evaluation(const vector<vector<int>>& routes) {
    double tour_length = 0.0;
    for (auto& route : routes) {
        for (size_t j = 0; j < route.size() - 1; ++j) {
            tour_length += distances[route[j]][route[j + 1]];
        }
    }

    evals++;

    return tour_length;
}

double Case::fitness_evaluation(const vector<int>& route) const {
    double tour_length = 0.0;
    for (size_t j = 0; j < route.size() - 1; ++j) {
        tour_length += distances[route[j]][route[j + 1]];
    }

    return tour_length;
}

vector<int> Case::compute_demand_sum(const vector<vector<int>>& routes) {
    vector<int> demand_sum;
    for (auto & route : routes) {
        int temp = 0;
        for (int node : route) {
            temp += get_customer_demand(node);
        }
        demand_sum.push_back(temp);
    }

    return demand_sum;
}

int Case::find_best_station(int from, int to) const {
    int theStation = -1;
    double bigDis = DBL_MAX;

    for (int station : stations) {
        double dis = distances[from][station] + distances[to][station];

        if (bigDis > dis && from != station && to != station) {
            theStation = station;
            bigDis = dis;
        }
    }

    return theStation;
}

int Case::find_best_station_feasible(int from, int to, double max_dis) const {
    int theStation = -1;
    double bigDis = DBL_MAX;

    for (int station : stations) {
        if (distances[from][station] < max_dis &&
            bigDis > distances[from][station] + distances[to][station] &&
            from != station && to != station &&
            distances[station][to] < maxDis) {

            theStation = station;
            bigDis = distances[from][station] + distances[to][station];
        }
    }

    return theStation;
}

int Case::find_nearest_station_to_y_feasible(int x, int y, double max_dis) {
    int targetedStation = -1;
    double minDis = DBL_MAX;

    for (int station : stations) {
        double x2station = get_distance(x, station);
        double station2y = get_distance(station, y);
        if (x2station <= max_dis && station2y < minDis) {
            targetedStation = station;
            minDis = station2y;
        }
    }

    return targetedStation;
}

bool Case::is_charging_station(int node) const {
    return node == depot || stationSet.count(node) > 0;
}
