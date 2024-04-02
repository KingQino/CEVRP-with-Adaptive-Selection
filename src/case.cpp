//
// Created by Yinghao Qin on 19/12/2023.
//

#include "../include/case.hpp"

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
    for (int i = 0; i < depotNumber + customerNumber; i++) {
        delete[] this->bestStation[i];
    }
    delete[] this->bestStation;
}

void Case::read_problem(const string& filepath) {
    stringstream ss;
    this->depotNumber = 1;
    this->depot = 0;
    ifstream infile(filepath.c_str());
    char line[250];
    while (infile.getline(line, 249)) {
        string templine(line);
        if (templine.find("DIMENSION:") != string::npos) {
            string substr = templine.substr(templine.find(':') + 1);
            ss << substr;
            ss >> this->customerNumber;
            ss.clear();
            this->customerNumber--;
        }
        else if (templine.find("STATIONS:") != string::npos) {
            string substr = templine.substr(templine.find(':') + 1);
            ss << substr;
            ss >> this->stationNumber;
            ss.clear();
        }
        else if (templine.find("VEHICLES:") != string::npos) {
            string substr = templine.substr(templine.find(':') + 1);
            ss << substr;
            ss >> this->vehicleNumber;
            ss.clear();
        }
        else if (templine.find("CAPACITY:") != string::npos && templine.find("ENERGY") == string::npos) {
            string substr = templine.substr(templine.find(':') + 1);
            ss << substr;
            ss >> this->maxC;
            ss.clear();
        }
        else if (templine.find("ENERGY_CAPACITY:") != string::npos) {
            string substr = templine.substr(templine.find(':') + 1);
            ss << substr;
            ss >> this->maxQ;
            ss.clear();
        }
        else if (templine.find("ENERGY_CONSUMPTION:") != string::npos) {
            string substr = templine.substr(templine.find(':') + 1);
            ss << substr;
            ss >> this->conR;
            ss.clear();
        }
        else if (templine.find("OPTIMAL_VALUE:") != string::npos) {
            string substr = templine.substr(templine.find(':') + 1);
            ss << substr;
            ss >> this->optimum;
            ss.clear();
        }
        else if (templine.find("NODE_COORD_SECTION") != string::npos) {
            this->actualProblemSize = depotNumber + customerNumber + stationNumber;
            for (int i = 0; i < actualProblemSize; i++) {
                positions.push_back(make_pair(0, 0));
            }
            for (int i = 0; i < actualProblemSize; i++) {
                infile.getline(line, 249);
                templine = line;
                ss << templine;
                int ind;
                double x, y;
                ss >> ind >> x >> y;
                ss.clear();
                positions[ind - 1].first = x;
                positions[ind - 1].second = y;
            }
        }
        else if (templine.find("DEMAND_SECTION") != string::npos) {
            int totalNumber = depotNumber + customerNumber;
            for (int i = 0; i < totalNumber; i++) {
                demand.push_back(0);
            }
            for (int i = 0; i < totalNumber; i++) {
                infile.getline(line, 249);
                templine = line;
                ss << templine;
                int ind;
                int c;
                ss >> ind >> c;
                ss.clear();
                demand[ind - 1] = c;
                if (c == 0) {
                    depot = ind - 1;
                }
            }
        }
    }
    infile.close();

    // processing variables
    for (int i = 1; i < depotNumber + customerNumber; ++i) {
        customers.push_back(i);
    }

    for (int i = depotNumber + customerNumber; i < actualProblemSize; ++i) {
        stations.push_back(i);
    }

    this->maxDis = maxQ / conR;

    this->totalDem = 0;
    for (auto& e : demand) {
        this->totalDem += e;
    }

    this->distances = generate_2D_matrix_double(actualProblemSize, actualProblemSize);
    int i, j;
    for (i = 0; i < actualProblemSize; i++) {
        for (j = 0; j < actualProblemSize; j++) {
            distances[i][j] = euclidean_distance(i, j);
        }
    }

    this->bestStation = new int* [depotNumber + customerNumber];
    for (int i = 0; i < depotNumber + customerNumber; i++) {
        this->bestStation[i] = new int[depotNumber + customerNumber];
        memset(this->bestStation[i], 0, sizeof(int)* (depotNumber + customerNumber));
    }
    for (int i = 0; i < depotNumber + customerNumber - 1; i++) {
        for (int j = i + 1; j < depotNumber + customerNumber; j++) {
            this->bestStation[i][j] = this->bestStation[j][i] = find_best_station(i, j);
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
    for (int i = 1; i <= customerNumber; ++i) {
        int nearestStation = -1;
        double minDis = DBL_MAX;
        for (int j = customerNumber + 1; j < actualProblemSize; ++j) {
            double dis = distances[i][j];
            if (minDis > dis) {
                nearestStation = j;
                minDis = dis;
            }
        }
        customerNearestStationMap[i] = make_pair(nearestStation, minDis);
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
        for (int j = 0; j < route.size() - 1; ++j) {
            tour_length += distances[route[j]][route[j + 1]];
        }
    }

    evals++;

    return tour_length;
}

double Case::fitness_evaluation(const vector<int>& route) const {
    double tour_length = 0.0;
    for (int j = 0; j < route.size() - 1; ++j) {
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

    for (int i = customerNumber + 1 ; i < actualProblemSize; ++i) {
        double dis = distances[from][i] + distances[to][i];

        if (bigDis > dis && from != i && to != i) {
            theStation = i;
            bigDis = dis;
        }
    }

    return theStation;
}

int Case::find_best_station_feasible(int from, int to, double max_dis) const {
    int theStation = -1;
    double bigDis = DBL_MAX;

    for (int i = customerNumber + 1; i < actualProblemSize; ++i) {
        if (distances[from][i] < max_dis &&
            bigDis > distances[from][i]  + distances[to][i]  &&
            from != i && to != i &&
            distances[i][to] < maxDis) {

            theStation = i;
            bigDis = distances[from][i] + distances[to][i];
        }
    }

    return theStation;
}

int Case::find_nearest_station_to_y_feasible(int x, int y, double max_dis) {
    int targetedStation = -1;
    double minDis = DBL_MAX;

    for (int s = customerNumber + 1; s < actualProblemSize; ++s) {
        double x2station = get_distance(x, s);
        double station2y = get_distance(s, y);
        if (x2station <= max_dis && station2y < minDis) {
            targetedStation = s;
            minDis = station2y;
        }
    }

    return targetedStation;
}

bool Case::is_charging_station(int node) const {

    bool flag;
    if (node == depot || ( node >= depotNumber + customerNumber && node < actualProblemSize))
        flag = true;
    else
        flag = false;
    return flag;
}
