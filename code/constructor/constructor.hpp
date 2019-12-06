#ifndef __H_CONSTRUCTOR_H__
#define __H_CONSTRUCTOR_H__

#include "../common/defs.hpp"
#include "../common/data.hpp"
#include "../common/solution.hpp"
#include "../common/pairmovementtuple.hpp"
#include <iostream>
#include <cmath>
#include <limits>
#include <random>

class Constructor{
public:
	Constructor(const Data& data_, std::mt19937_64 &rng_) : data(data_), rng(rng_) {};
    Solution greedy_heuristic(int k=1);
	Solution trivial_heuristic();
	
private:
	const Data& data;
	std::mt19937_64 &rng;
	
    /*Check if inserting u after node i in route is a valid movement*/
    bool is_valid_insertion(const Route& route, int i, int u, double rtime_i);

    /*Cost type 1 to insert node u between i and j*/
    double c1(int i, int u, int j);

    /*Insert node u after node i in route, updating information accordingly*/
    Route insert_in_route(Route route, int i, int u);
    
    /*Get the PD pair that minimizes cost cost when inserting in route*/
    PairMovementTuple get_best_pd_pair(Route route, const std::vector<bool>& routed, int k=1);

    /*Get the first node to insert in route*/
    int get_route_first_node(const std::vector<bool>& routed, int type=1);
};

#endif //__H_CONSTRUCTOR_H__
