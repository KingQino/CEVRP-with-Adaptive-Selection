//
// Created by Yinghao Qin on 16/11/2023.
//

#ifndef SAMPLE_UTILS_HPP
#define SAMPLE_UTILS_HPP

#include <iostream>
#include <vector>
#include <random>
#include <cstring>
#include <numeric>
#include <memory>

#include "individual.hpp"
#include "case.hpp"

using namespace std;

#define INFEASIBLE 1000000000


// used in 2-opt*, define the hash relationship to make sure one-to-one mapping between route pairs
struct pair_hash
{
    size_t operator() (pair<int, int> const & apair) const {
        return apair.first * 256 + apair.second;
    }
};


// population initialization
vector<vector<int>> prins_split(const vector<int>& x, Case& instance);
vector<vector<int>> hien_clustering(const Case& instance, std::default_random_engine& rng);
void hien_balancing(vector<vector<int>>& routes, const Case& instance, std::default_random_engine& rng);
vector<vector<int>> routes_constructor_with_split(Case& instance, std::default_random_engine& rng);
vector<vector<int>> routes_constructor_with_hien_method(const Case& instance, std::default_random_engine& rng);
vector<vector<int>> routes_construct_with_direct_encoding(const Case& instance, std::default_random_engine& rng);

// local search operators
double two_opt_for_single_route(vector<int>& route, Case& instance);
bool two_opt_for_individual(Individual& individual, Case& instance);
bool two_opt_star_for_individual(Individual& individual, Case& instance);
bool node_shift(int* route, int length, double& fitv, Case& instance);
void moveItoJ(int* route, int a, int b);
void node_shift_for_individual(Individual& individual, Case& instance);

// recharging optimization
double fix_one_solution(Individual& individual, Case& instance);
pair<double, vector<int>> insert_station_by_simple_enumeration_array(int* route, int length, Case& instance);
pair<double, vector<int>> insert_station_by_remove_array(int* route, int length, Case& instance);
void tryACertainNArray(int mlen, int nlen, int* chosenPos, int* bestChosenPos, double& finalfit, int curub, int* route, int length, vector<double>& accumulateDis, Case& instance);
pair<double, vector<int>> simple_repair_target_one_station(const int* route, int length, Case& instance); // O(n) - designed for route need only one station - before using, calculate how many stations are needed,
pair<double, vector<int>> station_reallocate_one(vector<int>& repairedForwardRoute, double fit, Case& instance); // O(n) - designed for simple repaired route with one station - potentially improve it

// Refine
pair<vector<int>, double> insert_station_by_enumeration(vector<int>& route, Case& instance);
void tryACertainN(int mlen, int nlen, int* chosenSta, int* chosenPos, vector<int>& finalRoute, double& finalfit, int curub, vector<int>& route, vector<double>& accumulateDis, Case& instance);

// GA operators
vector<vector<int>> selRandom(const vector<vector<int>>& chromosomes, int k, std::default_random_engine& rng);
vector<std::shared_ptr<Individual>> selRandom(const vector<std::shared_ptr<Individual>>& individuals, int k, std::default_random_engine& rng);
vector<std::shared_ptr<Individual>> selTournament(const vector<std::shared_ptr<Individual>>& individuals, int k, int tournamentSize, std::default_random_engine& rng);
void cxPartiallyMatched(vector<int>& parent1, vector<int>& parent2, std::default_random_engine& rng);
void mutShuffleIndexes(vector<int>& chromosome, double indpb, std::default_random_engine& rng);


// tools
std::shared_ptr<Individual> select_best_individual(const vector<std::shared_ptr<Individual>>& population);
std::shared_ptr<Individual> select_worst_individual(const vector<std::shared_ptr<Individual>>& population);


#endif //SAMPLE_UTILS_HPP
