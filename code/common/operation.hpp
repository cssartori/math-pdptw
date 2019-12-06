#ifndef __H_OPERATION_H__
#define __H_OPERATION_H__

#include "solution.hpp"
#include "data.hpp"
#include "route.hpp"
#include "pairmovementtuple.hpp"

//this is really the core engine of the whole algorithm
//as in, the MOST BASIC operations are defined here
namespace Operation{
	void insert_in_route(const Data& data, Route& route, int i, int u);
	void remove_pair_from_route(const Data& data, Route& route, int u);
	void calculate_route_fts(const Data& data, Route& route);
	void calculate_solution_fts(const Data& data, Solution& s);
	double c_in(const Data& data, int i, int u, int j);
	PairMovementTuple search_insertion_location(const Data& data, Route& route, int u, std::mt19937_64& rng, bool random=false);
	PairMovementTuple search_all_insertion_locations(const Data& data, Route& route, int u, int rindex, std::mt19937_64& rng, bool random=true);
	void remove_pairs_from_solution(const Data& data, const size_t q, const std::vector<int>& pairs, Solution& s);
	void remove_pairs_from_solution(const Data& data, const size_t q, const std::vector<int>& pairs, const std::vector<size_t>& nr, Solution& s);
	size_t get_node_route(const Solution& s, int u);
	void get_all_nodes_routes(const Data& data, const Solution& s, std::vector<size_t>& nr);
	void compute_route_values(const Data& data, Route& route);
	PairMovementTuple sampled_search_insertion_location(const Data& data, Route& route, int u, size_t& tries, std::mt19937_64& rng, double beta = 0.99, PairMovementTuple pm = PairMovementTuple());

};

#endif //__H_OPERATION_H__
