
#include "individual.hpp"
#include "utils.hpp"

struct solution{
  int *tour;	//this is what the fitness_evaluation function in EVRP.hpp will evaluate
  int id;
  double tour_length; //quality of the solution
  int steps; //size of the solution
  //the format of the solution is as follows:
  //*tour:  0 - 5 - 6 - 8 - 0 - 1 - 2 - 3 - 4 - 0 - 7 - 0
  //*steps: 12
  //this solution consists of three routes: 
  //Route 1: 0 - 5 - 6 - 8 - 0
  //Route 2: 0 - 1 - 2 - 3 - 4 - 0
  //Route 3: 0 - 7 - 0
};


extern solution *best_sol;

extern int pop_size;
extern double cx_prob;
extern double mut_prob;
extern double mut_indpb;
extern int tournament_size;

extern int route_capacity;
extern int node_capacity;
extern vector<Individual*> population;
extern Individual* best_ind;
extern int gen;

extern int pop_size_monitor; // population

void initialize_heuristic();
void run_heuristic();
void run_heuristic_with_confidence_based_selection(long long duration);
void run_heuristic_with_confidence_based_selection_flow(long long duration);
void run_heuristic_with_confidence_based_selection_switch(long long duration);
void refine();



void free_heuristic();