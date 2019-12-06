#ifndef __H_SPPM_H__
#define __H_SPPM_H__

#include "../common/data.hpp"
#include "../common/defs.hpp"
#include "../common/route.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations" //avoid CPLEX warnings of deprecated declarations
#include <ilcplex/ilocplex.h>
#pragma GCC diagnostic pop

#include <boost/dynamic_bitset.hpp>
#include <map>
#include <iostream>
#include <cstdio>

class SPPModel{
public:
	IloNumVarArray x;
	IloModel model;
	std::vector<std::reference_wrapper<Route> > ridx;
	double solve(const Data& data, std::map< boost::dynamic_bitset<>, Route >& route_pool, Solution& s, Solution& sbest);

private:
	Solution get_solution_set(const Data& data, const IloCplex& cplex);
	void build_model(const Data& data, IloEnv& env, std::map< boost::dynamic_bitset<>, Route >& route_pool, std::map< boost::dynamic_bitset<>, size_t >& route_index);
	void warm_start(IloEnv& env, IloCplex& cplex, Solution& s, std::map< boost::dynamic_bitset<>, size_t >& route_index);

};

#endif //__H_SPPM_H__
