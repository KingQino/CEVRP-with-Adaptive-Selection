//
// Created by Yinghao Qin on 19/12/2023.
//

#ifndef CEVRP_YINGHAO_CASE_HPP
#define CEVRP_YINGHAO_CASE_HPP

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <set>
#include <cstring>
#include <string>
#include <map>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <cstdio>



using namespace std;

class Case {
public:
    static const int MAX_EVALUATION_FACTOR;


    Case(const string& filepath, int id);
    ~Case();
    void read_problem(const string& filepath);					//reads .evrp file
    double euclidean_distance(int i, int j);
    void init_customer_clusters_map();
    void init_customer_nearest_station_map();
    static double **generate_2D_matrix_double(int n, int m);
    [[nodiscard]] int get_customer_demand(int customer) const;				//returns the customer demand
    double get_distance(int from, int to);				//returns the distance
    [[nodiscard]] double get_evals() const;									//returns the number of evaluations
    double fitness_evaluation(const vector<vector<int>>& routes); // customized fitness function
    [[nodiscard]] double fitness_evaluation(const vector<int>& route) const; // used for testing TODO: DELETE on Release
    vector<int> compute_demand_sum(const vector<vector<int>>& routes); // compute the demand sum of all customers for each route.
    [[nodiscard]] int find_best_station(int from, int to) const;
    [[nodiscard]] int find_best_station_feasible(int from, int to, double max_dis) const; // the station within allowed max distance from "from", and min dis[from][s]+dis[to][s]
    int find_nearest_station_to_y_feasible(int x, int y, double max_dis); // find the nearest station to y, and meanwhile the station is reachable for x
    bool is_charging_station(int node) const;					//returns true if node is a charging station


    int ID;
    string fileName;
    string instanceName;

    int depotNumber;
    int customerNumber;
    int stationNumber;
    int vehicleNumber;
    int actualProblemSize; //Total number of customers, charging stations and depot
    int maxC;
    double maxQ;
    double conR; // energy consumption rate;
    vector<pair<double, double>> positions;
    vector<int> demand;
    int depot;  //depot id (usually 0)
    vector<int> customers;
    vector<int> stations;
    double maxDis;
    int totalDem;
    double** distances;
    double optimum;
    int** bestStation; // "bestStation" is designed for two customers, bringing the minimum extra cost.
    unordered_map<int, vector<int>> customerClustersMap; // For Hien's clustering usage only. For each customer, a list of customer nodes from near to far, e.g., {1: [5,3,2,6], 2: [], ...}
    unordered_map<int, pair<int, double>> customerNearestStationMap; // for each customer, find the nearest station and store the corresponding distance
    double evals;
    double maxEvals;
    int maxExecTime; // unit seconds
};


#endif //CEVRP_YINGHAO_CASE_HPP

