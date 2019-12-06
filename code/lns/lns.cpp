#include "lns.hpp"
#include "../common/operation.hpp"
#include "../validator/validator.hpp"
#include "../common/timer.hpp"
#include <random>
#include <boost/dynamic_bitset.hpp>
#include <algorithm>
#include <map>
#include <cmath>

using namespace std;
using namespace boost;

struct RelatePair{
	double cost;
	int u;

	RelatePair(){
		cost = Def::INF;
		u = Def::NO_NODE;
	}
};

//auxiliary stuff (it could be implemented as an overloaded operator in the above struct!)
bool comp_relate_pair(const RelatePair& rp1, const RelatePair& rp2){
	return rp1.cost < rp2.cost;
}
	
double compute_max_reach_time(const Data& data, const Solution& s);
void compute_relatedness(const Data& data, const Parameters& par, const Solution& s, const std::vector<size_t>& nr, std::vector<RelatePair>& rvec, int u, double maxrt);

void LNS::solve(Timer& t, Solution& s){
	std::uniform_real_distribution<double> prob(0, 1);
	std::uniform_int_distribution<size_t> rand_q(static_cast<size_t>(min_q), static_cast<size_t>(max_q));

	weight_removal[LNS::REM_SHAW_TYPE] = par.lns_wrem[LNS::REM_SHAW_TYPE];
	weight_removal[LNS::REM_RANDOM_TYPE] = par.lns_wrem[LNS::REM_RANDOM_TYPE]+par.lns_wrem[LNS::REM_SHAW_TYPE];
	weight_removal[LNS::REM_WORST_TYPE] = par.lns_wrem[LNS::REM_WORST_TYPE]+par.lns_wrem[LNS::REM_RANDOM_TYPE]+par.lns_wrem[LNS::REM_SHAW_TYPE];
	total_weight = par.lns_wrem[LNS::REM_WORST_TYPE]+par.lns_wrem[LNS::REM_RANDOM_TYPE]+par.lns_wrem[LNS::REM_SHAW_TYPE];
	
	vector<double> lhist(lsize,s.cost);
	Solution sbest = s;
	int iter = 0;
	int inop = 0;
	vector<int> removed(max_q, Def::NO_NODE);

	vector<int> pens(data.nnod, 0);
	
	while(inop < par.lns_max_iter && t.elapsed_seconds() < par.max_time){
		size_t q = rand_q(rng);
		q = q > static_cast<size_t>(data.nreq) ? static_cast<size_t>(data.nreq) : q;
		int removal_type = 0;
		
		Solution sl = s;
		double p = prob(rng)*total_weight;
		if(p <= weight_removal[LNS::REM_SHAW_TYPE]){
			shaw_removal(q, sl, removed);
			removal_type = LNS::REM_SHAW_TYPE;
		}else if(p <= weight_removal[LNS::REM_RANDOM_TYPE]){
			random_removal(q, sl, removed);
			removal_type = LNS::REM_RANDOM_TYPE;
		}else if(p <= weight_removal[LNS::REM_WORST_TYPE]){
		 	worst_removal(q, sl, removed);
			removal_type = LNS::REM_WORST_TYPE;
		}

		removal_called[removal_type] += 1;
		size_t num_rein = regret_heuristic(q, sl, removed);

		bool imp = false;
		if(num_rein == q){
			//only gets here if all requests were reinserted
			removal_reinsert[removal_type] += 1;

			//and now the LAHC acceptance part
			int x = iter % lsize;
			if(sl.cost <= lhist[x] || sl.cost <= sbest.cost){
				if(sl.cost - sbest.cost < -Def::EPSILON){
					removal_improve[removal_type] += 1;
					sbest = sl;
					inop=0;
					imp = true;
				}
				s = sl;
			}else{
				//reset solution to the best if not accepted (doing it or not does not change significantly the solution quality)
				s = sbest;
			}
			lhist[x] = min(lhist[x], s.cost);
		}
		
		if(imp){
		 	printd(("%i - New Solution: %.2f | %i | %i | %.2f\n", iter, sbest.cost, Validator::validate_solution(data, sbest), imp, ((double)q/data.nreq)*100.00));
		}
		
		iter++;
		inop++;
		total_iter++;
	}
	sbest.clear_routes();
	s = sbest;
}


size_t LNS::regret_heuristic(const size_t q, Solution &s, vector<int>& removed){
	//generator of the k in k-regret heuristic
	std::uniform_int_distribution<size_t> rand_k(static_cast<size_t>(par.lns_min_k), static_cast<size_t>(par.lns_max_k));	
	size_t k = par.lns_min_k == par.lns_max_k ? par.lns_min_k : rand_k(rng);
	k = (k >= s.size()) ? s.size() : k;
	
	dynamic_bitset<> added(removed.size()); //bitset to identify added requests
	vector< vector<PairMovementTuple> > moves(removed.size(), vector<PairMovementTuple>(s.size())); //movements available for each removed request
	vector< vector<PairMovementTuple> > moves_cache(removed.size(), vector<PairMovementTuple>(s.size())); //cache of already computed movements (used for speedup)
	dynamic_bitset<> route_modified(s.size()); //bit set indicating whether a route has been modified 
	route_modified.set(); //at first, all routes have been modified
	
	while(added.count() < q){
		//for each removed request compute the best insertion in each route
		for(size_t i=0;i<q;i++){ //for each ith-removed request 
			if(added[i]) continue;
			for(size_t r=0;r<s.size();r++){ //for each rth-route
				if(not route_modified[r]){ //if route r has not been modified its movement is the same
					moves[i][r] = moves_cache[i][r];
				}else{
					moves[i][r] = Operation::search_insertion_location(data, s.routes[r], removed[i], rng);
					moves[i][r].r1 = r;
					moves[i][r].r2 = r;
					moves_cache[i][r] = moves[i][r]; //update cache
				}
			}
			sort(moves[i].begin(), moves[i].end()); //sort moves of ith-request according to their cost
		}

		//reset because now all have been computed
		route_modified.reset();
		
		double max_reg = -Def::INF;
		size_t max_i = Def::NO_NODE;	
		for(size_t i=0;i<q;i++){ //for each removed request
			if(added[i]) continue;
			if(isinf(moves[i][0].cost)){ //if a request has NO insertion position now, it won't have in the future, we may stop earlier
				return 0;
			}

			//compute total regret for ith-request
			double reg = 0;
			for(size_t m=1;m<k;m++){ //for each m <= k
				reg += (moves[i][m].cost - moves[i][0].cost);
				if(isinf(reg)) break;
			}

			//update maximum regret and index
			if(reg > max_reg){
				max_i = i;
				max_reg = reg;
			}
		}

		if(max_i == (size_t)Def::NO_NODE){ //if no request could be inserted (does it even reach here with such state?)
			return 0;
		}

		//perform the insertion movement
		PairMovementTuple& pmt = moves[max_i][0];
		s.cost -= s.routes[pmt.r1].cost;
		Operation::insert_in_route(data, s.routes[pmt.r1], pmt.back_u, pmt.u);
		Operation::insert_in_route(data, s.routes[pmt.r1], pmt.back_v, pmt.v);
		Operation::calculate_route_fts(data, s.routes[pmt.r1]);
		s.cost += s.routes[pmt.r1].cost;

		//update bitsets accordingly
		route_modified[pmt.r1] = true;
		added[max_i] = true;
	}

	return added.count();
}

void LNS::worst_removal(const size_t q, Solution& s, vector<int>& removed){
	std::uniform_real_distribution<double> prob(0, 1);
	vector<RelatePair> rvec(data.nreq+1);
	size_t ridx = 0;

	vector<size_t> nr(data.nnod, Def::NO_ROUTE); //route of each node
	Operation::get_all_nodes_routes(data, s, nr);

	for(int i=1;i<data.nnod-1;i++){
		if(data.nodes[i].type == Def::PICKUP_NODE){
			int pi = i;
			int di = data.nodes[i].pair;
			int ri = nr[i];
			int fpi = s.routes[ri].forward[pi];
			int bpi = s.routes[ri].backward[pi];
			int fdi = s.routes[ri].forward[di];
			int bdi = s.routes[ri].backward[di];
			if(fpi == di)
				rvec[ridx].cost = -((data.times[bpi][pi]+data.times[pi][di]+data.times[di][fdi])-data.times[bpi][fdi]);
			else
				rvec[ridx].cost = -(((data.times[bpi][pi]+data.times[pi][fpi])-data.times[bpi][fpi])+((data.times[bdi][di]+data.times[di][fdi])-data.times[bdi][fdi]));
			rvec[ridx].u = i;
			ridx++;
		}
	}
	
	const double p = 6;
	size_t idx = 0;
	dynamic_bitset<> rm(data.nnod, false);
	while(idx < q){
		double y = prob(rng);
		size_t rk = pow(y,p)*(ridx-idx+1);
		std::nth_element(rvec.begin(), rvec.begin()+rk, rvec.end(), comp_relate_pair);

		if(rvec[rk].u == Def::NO_NODE || rm[rvec[rk].u]) continue;
		removed[idx++] = rvec[rk].u;
		rm[rvec[rk].u] = true;
		rvec[rk].cost = Def::INF;
		rvec[rk].u = Def::NO_NODE;
	}
	
	Operation::remove_pairs_from_solution(data, q, removed, nr, s);
}

void LNS::random_removal(const size_t q, Solution& s, vector<int>& removed){
	std::uniform_int_distribution<int> rand_req(1, data.nnod-2);
	dynamic_bitset<> rm(data.nnod, false);
	size_t idx = 0;
	while(idx < q){
		int ru = rand_req(rng);
		ru = data.nodes[ru].type == Def::PICKUP_NODE ? ru : data.nodes[ru].pair;
		if(not rm[ru]){
			rm[ru] = true;
			removed[idx++] = ru;
		}
	}

	Operation::remove_pairs_from_solution(data, q, removed, s);
}

void LNS::shaw_removal(const size_t q, Solution& s, vector<int>& removed){
	std::uniform_int_distribution<int> rand_req(1, data.nnod-2);
	std::uniform_real_distribution<double> prob(0, 1);

	vector<size_t> nr(data.nnod, Def::NO_ROUTE); //route of each node
	Operation::get_all_nodes_routes(data, s, nr);
   		
	vector< vector<RelatePair> > rmat(data.nnod);	

	double p = 6;
	int u = rand_req(rng);
	u = data.nodes[u].type == Def::PICKUP_NODE ? u : data.nodes[u].pair;
	size_t idx = 0;
	removed[idx++] = u;

	//TODO: keep the max reach time updated in the solution structure for more efficiency
	double max_reach_time = compute_max_reach_time(data, s);
	compute_relatedness(data, par, s, nr, rmat[u], u, max_reach_time);

	dynamic_bitset<> rm(data.nnod, false);
	rm[u] = true;
	while(idx < q){
		int ru = removed[rng()%idx];
		for(size_t i=0;i < rmat[ru].size(); i++){
			if(rmat[ru][i].u != Def::NO_NODE && rm[rmat[ru][i].u])
				rmat[ru][i].cost = Def::INF;
		}

		double y = prob(rng);
		size_t rk = pow(y,p)*(rmat[ru].size()-idx+1);
		std::nth_element(rmat[ru].begin(), rmat[ru].begin()+rk, rmat[ru].end(), comp_relate_pair);

		if(isinf(rmat[ru][rk].cost)) continue; //if, for some reason, the selected movement is not valid (this should NEVER be triggered though)

		removed[idx++] = rmat[ru][rk].u;
		rm[rmat[ru][rk].u] = true;
		if(idx < q){
			compute_relatedness(data, par, s, nr, rmat[rmat[ru][rk].u], rmat[ru][rk].u, max_reach_time);
		}
	}	

	Operation::remove_pairs_from_solution(data, q, removed, nr, s);
}

double compute_max_reach_time(const Data& data, const Solution& s){
	double maxrt = 0;
	for(size_t r=0;r<s.size();r++){
		if((s.routes[r].reach_time[data.final_depot()] - maxrt) > Def::EPSILON) maxrt = s.routes[r].reach_time[data.final_depot()];
	}
	return maxrt;
}

void compute_relatedness(const Data& data, const Parameters& par, const Solution& s, const vector<size_t>& nr, vector<RelatePair>& rvec, int u, double maxrt){
	int i=u;

	double maxct = data.max_time; //the maximum cost for normalization
	int maxde = data.max_demand; //max demand also for normalization
	//TODO: do we really need to normalize? Some works seem to point otherwise (e.g., Parragh Matheuristic)
	
	double a=par.lns_wshaw[LNS::WSHAW_ALPHA], b=par.lns_wshaw[LNS::WSHAW_BETA], c=par.lns_wshaw[LNS::WSHAW_GAMMA];
	for(int j=1;j<data.nnod-1;j++){
		if(i==j) continue;
		if(data.nodes[j].type == Def::DELIVERY_NODE) continue;
		int di = data.nodes[i].pair;
		int dj = data.nodes[j].pair;
		RelatePair rp;
		rp.cost =
			a*(data.times[i][j]/maxct + data.times[di][dj]/maxct)
			+ b*(std::abs(s.routes[nr[i]].reach_time[i]/maxrt - s.routes[nr[j]].reach_time[j]/maxrt)
				 + std::abs(s.routes[nr[di]].reach_time[di]/maxrt - s.routes[nr[dj]].reach_time[dj]/maxrt))
			+ c*(std::abs((double)data.nodes[i].demand/maxde-(double)data.nodes[j].demand/maxde));
		
		rp.u = j;
		rvec.push_back(rp);
	}	
}
