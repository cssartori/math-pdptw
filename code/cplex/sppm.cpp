#include "sppm.hpp"

ILOSTLBEGIN

void SPPModel::build_model(const Data& data, IloEnv& env, std::map< boost::dynamic_bitset<>, Route >& route_pool, std::map< boost::dynamic_bitset<>, size_t >& route_index){
	x = IloNumVarArray(env, route_pool.size(), 0.0, 1.0, ILOINT);

	model = IloModel(env);
	IloExpr obj(env);
	ridx.reserve(route_pool.size());
	std::map<boost::dynamic_bitset<>, Route>::iterator it;
	int i=0;
	for(it = route_pool.begin(); it != route_pool.end(); it++){
		obj += x[i++]*(it->second.cost);
		ridx.push_back(it->second);
		route_index[it->second.node_set] = i-1;
	}
	model.add(IloMinimize(env, obj));
	obj.end();
	
	for(int j=1;j<data.nnod-1;j++){
		IloExpr e(env);
		for(size_t i=0;i<ridx.size();i++){
			e += (ridx[i].get().has_node(j) ? 1 : 0)*x[i];
		}
		model.add(e == 1);
	}
	
}

Solution SPPModel::get_solution_set(const Data& data, const IloCplex& cplex){
	Solution s(data.nnod);
	for(size_t i=0;i<ridx.size();i++){
		if(cplex.isExtracted(x[i]) && round(cplex.getValue(x[i])) == 1.0){
			s.insert_route(ridx[i].get());
			s.cost += ridx[i].get().cost;
		}
	}
	return s;
}


void SPPModel::warm_start(IloEnv& env, IloCplex& cplex, Solution& s, std::map< boost::dynamic_bitset<>, size_t >& route_index){
	IloNumArray x_val(env, route_index.size());
	for(Route& route : s.routes){
		x_val[route_index[route.node_set]] = 1.0;
	}
	cplex.addMIPStart(x, x_val);
	x_val.end();
}

double SPPModel::solve(const Data& data, std::map< boost::dynamic_bitset<>, Route >& route_pool, Solution& s, Solution& sbest){
	std::map< boost::dynamic_bitset<>, size_t > route_index;
	IloEnv env;
	build_model(data, env, route_pool, route_index);
	IloCplex cplex(model);
	warm_start(env, cplex, sbest, route_index);
	cplex.setParam(IloCplex::Param::Threads, 1);
	cplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, 0.0000001);
	cplex.setParam(IloCplex::Param::TimeLimit, 60.0);
		
#ifndef DEBUG
	cplex.setOut(env.getNullStream());
	cplex.setWarning(env.getNullStream());
#endif
	cplex.solve();

	double cpx_value = Def::INF;
	//cout << "STATUS " << cplex.getStatus() << endl;
	if(cplex.getStatus() == IloAlgorithm::Feasible || cplex.getStatus() == IloAlgorithm::Optimal){
		cpx_value = cplex.getObjValue();
		//printd(("CPLEX: %.2f | pool = %li | improve = %i\n", cpx_value, route_pool.size(), spp_improve));
		if((cpx_value - sbest.cost) < -Def::EPSILON){
			s = get_solution_set(data, cplex);
			s.node_set.set();
		}
	}

	//we need to end the env of CPLEX to avoid memory leaks (and overflows!)
	cplex.end();
	env.end();
	return cpx_value;
}
