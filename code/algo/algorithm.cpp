#include "algorithm.hpp"
#include "../common/defs.hpp"
#include "../common/timer.hpp"
#include "../constructor/constructor.hpp"
#include "../common/operation.hpp"
#include "../common/perturb.hpp"
#include "../common/solution.hpp"
#include "../validator/validator.hpp"
#include "../ages/ages.hpp"
#include "../lns/lns.hpp"
#include "../cplex/sppm.hpp"
#include <boost/dynamic_bitset.hpp>
#include <map>
#include <random>

using namespace std;

ResultInfo Algorithm::solve(){
	std::uniform_real_distribution<double> prob(0, 1);
	std::mt19937_64 rng(par.seed);
	Timer t;
	t.start();
	
	ResultInfo rinfo;

	Constructor con(data, rng);
	Solution s(data.nnod);

	//Create Initial Solution
	if(par.initial_constructor == Def::CONSTRUCTOR_GREEDY){ //greedy
		s = con.greedy_heuristic();
		Operation::calculate_solution_fts(data, s); //greedy constructor does not keep FTS updated
	}else{ //trivial
		s = con.trivial_heuristic();
		Operation::calculate_solution_fts(data, s); //trivial constructor does not keep FTS updated
	}

	printd(("Cost = %.2f | %li | %i\n", s.cost, s.size(), Validator::validate_solution(data,s)));
	
	//save initial solution
	rinfo.si = s;
	rinfo.initial_time = t.elapsed_seconds();
	
	Solution sb = s;
	size_t iter = 0, inop = 0;

	string logstr = "";
	string slicestr = "";
	string solstr = "";

	map< boost::dynamic_bitset<>, Route > route_pool;	
	if(par.use_spp) extract_routes(s, route_pool);

	size_t ages_pert_size;
	if(par.alg_ages == Def::AGES_ORIGINAL_TYPE){
		ages_pert_size = par.ges_psize;
	}else{
		ages_pert_size = static_cast<size_t>(par.ges_ppsize*data.nreq);
	}
	
	AGES ages(data, rng, par.ges_max_iter, std::max(ages_pert_size, (size_t)1), par.prob_eval);
	LNS lns(data, par, rng, par.lns_min_q, std::max(static_cast<size_t>(par.lns_max_mult*data.nreq), (size_t)1), par.lns_min_k, par.lns_max_k, par.lns_lsize, par.lns_max_iter);
	
	double elap_sec = t.elapsed_seconds();
	double obest = s.cost;
	size_t perturb_count = 0;
	
	while(elap_sec < par.max_time){
			
		if( (elap_sec <= par.max_time)
			|| (par.save_log && (obest - sb.cost) >= Def::EPSILON) ){
			
			//update stored values 
			rinfo.total_time = elap_sec;
			rinfo.iter = iter;
			rinfo.spp_pool_size = route_pool.size();

			if((par.save_log && (obest - sb.cost) >= Def::EPSILON)){
				logstr += to_string(rinfo.nimp)+string(";")+rinfo.to_string(data);
				obest = sb.cost;
			}
		}
		
		double tb_ges = elap_sec; //time before GES
		printd(("%li - Best Solution: %.2f\n", iter, sb.cost));
		size_t prev_sol_size = s.size();
		printd(("GES\n"));
		if(par.alg_ages == Def::AGES_NEW_TYPE){
			ages.solve(s, t, par.max_time);
		}else{
			ages.original_solve(s, t, par.max_time, par.prob_eval);
		}
			
		printd(("*Cost = %.2f | %li | %i | %i\n", s.cost, s.size(), Validator::validate_solution(data,s), s.node_set.all()));

		rinfo.ges_total_time += (t.elapsed_seconds() - tb_ges);
		if(prev_sol_size > s.size()) rinfo.ges_num_reduced++;
			
		double prev_cost = s.cost;		
		double tb_lns = t.elapsed_seconds(); //time before LNS
		
		printd(("LNS\n"));
		lns.solve(t, s);
		printd(("Cost = %.2f | %li | %i\n", s.cost, s.size(), Validator::validate_solution(data,s)));

		rinfo.lns_total_time += (t.elapsed_seconds() - tb_lns);
		
		double tb_spp = t.elapsed_seconds();		
		if(par.use_spp && tb_spp < par.max_time){
			bool pool_modified = false;
			//if solution has been modified
			if(std::abs(s.cost - prev_cost) > Def::EPSILON){
				printd(("Adding %li routes to the pool\n", s.size()));
				pool_modified = extract_routes(s,route_pool);
			}

			if(pool_modified){
				SPPModel spm;
				double prev_cost = s.cost;
				double cpx_value = spm.solve(data, route_pool, s, s.cost < sb.cost ? s : sb);		
				printd(("CPLEX: %.2f | pool = %li | improve = %li\n", cpx_value, route_pool.size(), rinfo.spp_num_improve));
				if((cpx_value - sb.cost) < -Def::EPSILON){
					if((cpx_value - prev_cost) < -Def::EPSILON){
						rinfo.spp_num_improve++;
					}
					printd(("Improved: %f < %f\n", cpx_value, sb.cost));
				}
				rinfo.spp_total_time += t.elapsed_seconds()-tb_spp;
			}
		}

		double tb_pert = t.elapsed_seconds();
		// if the cost has been improved
		if(sb.cost-s.cost >= Def::EPSILON && tb_pert < par.max_time){
			inop=0;
			sb=s;
			rinfo.sb = sb;
			rinfo.nimp++; //count an improvement
		}else if(tb_pert < par.max_time){ // otherwise perturb
			if(prob(rng) < (double)(iter-inop)/iter)
				s = sb;
			
			inop++;
			// perturb solution
			vector<size_t> nodes_routes(data.nnod, Def::NO_ROUTE);
			//TODO: this can probably be improved by keeping the information as we go
			Operation::get_all_nodes_routes(data, s, nodes_routes);
			if(par.alg_ages == Def::AGES_NEW_TYPE){
				Perturb::sampled_perturb(data, par.ppsize*data.nreq, s, nodes_routes, perturb_count, par.prob_eval, rng);
			}else{
				Perturb::original_perturb(data, par.ppsize*data.nreq, par.prob_eval, s, perturb_count, nodes_routes, rng);
			}
		}
		
		iter++;
		elap_sec = t.elapsed_seconds();
	}

	//TODO: is this still necessary?
	if(sb.cost-s.cost >= Def::EPSILON){
		sb = s;
		rinfo.nimp++; //count an improvement
	}
	
	rinfo.total_time = t.elapsed_seconds();
	rinfo.iter = iter;
	rinfo.sb = sb;
	rinfo.spp_pool_size = route_pool.size();
	
	if(par.save_log){
		ofstream logfile;
		logfile.open(par.log_filename);
		if(not logfile.is_open()){
			cout << "Error opening file " << par.log_filename << endl;
			exit(-1);
		}
		logfile << "improvements:\n";
		logfile << logstr;
		logfile << "\nsolutions:\n";
		solstr += rinfo.to_string(data);
		logfile << solstr;
		logfile.close();
	}
	
	return rinfo;
}


//extract routes from solution s and add them to the pool of routes
bool Algorithm::extract_routes(const Solution& s, map< boost::dynamic_bitset<>, Route >& route_pool){
	bool mod = false;
	for(const Route& route : s.routes){
		std::pair< std::map<boost::dynamic_bitset<>, Route>::iterator, bool> it = route_pool.insert(make_pair(route.node_set, route));
		if(not it.second && it.first->second.cost > route.cost){
			it.first->second = route;
			mod = true;
		}else if(it.second){
			mod = true;
		}
	}
	return mod;
}
