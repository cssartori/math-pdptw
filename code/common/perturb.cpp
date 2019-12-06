#include "perturb.hpp"
#include "operation.hpp"

using namespace std;

void Perturb::sampled_perturb(const Data& data, const size_t max_perturb, Solution& s, std::vector<size_t>& nr, size_t& ncount, double prob_eval, std::mt19937_64& rng, size_t forb_route){
	std::uniform_int_distribution<int> rand_req(1, data.nnod-2);
	double ptries = 100;
	size_t perturb = 0;
	while(perturb < max_perturb){
		int u = rand_req(rng);
		u = data.nodes[u].type == Def::PICKUP_NODE ? u : data.nodes[u].pair;
		size_t r = nr[u]; //route of node u 
		if(r != Def::NO_ROUTE && r != forb_route){
			int bu = s.routes[r].backward[u];
			int bdu = s.routes[r].backward[data.nodes[u].pair];
			s.cost -= s.routes[r].cost;
			Operation::remove_pair_from_route(data, s.routes[r], u);
			Operation::calculate_route_fts(data, s.routes[r]);
			size_t newr = Perturb::sampled_reinsert(data, prob_eval, ptries, u, s, rng, forb_route);
			if(newr == Def::NO_ROUTE){
				Operation::insert_in_route(data, s.routes[r], bu, u);
				Operation::insert_in_route(data, s.routes[r], bdu, data.nodes[u].pair);
				Operation::calculate_route_fts(data, s.routes[r]);
			}else{
				ncount++;
				nr[u] = newr; //update node route index
				nr[data.nodes[u].pair] = newr;
			}
			s.cost += s.routes[r].cost;
			perturb++;
		}
	}

	//TODO: check this update
	s.cost = 0;
	for(Route& route: s.routes){
		s.cost += route.cost;
	}
}

size_t Perturb::sampled_reinsert(const Data& data, const double prob_eval, const double ptries, const int u, Solution &s, std::mt19937_64& rng, size_t forb_route){
	PairMovementTuple pm;
	size_t first_route = rng()%s.size();
	size_t r = first_route;
	//size_t max_tries = ptries*(data.nnod*data.nnod);
	size_t tries = 0;
	do{
		if(r != forb_route){
			int oback_u = pm.back_u;
			int oback_v = pm.back_v;
			pm = Operation::sampled_search_insertion_location(data, s.routes[r], u, tries, rng, prob_eval, pm);
			pm.r1 = ((oback_u == pm.back_u) && (oback_v == pm.back_v)) ? pm.r1 : r;
		}
		r = (r+1)%s.size();
	}while(pm.back_u == Def::NO_NODE && r != first_route);

	if(pm.back_u == Def::NO_NODE){
		return Def::NO_ROUTE;
	}
	
	s.cost -= s.routes[pm.r1].cost;
	Operation::insert_in_route(data, s.routes[pm.r1], pm.back_u, pm.u);
	Operation::insert_in_route(data, s.routes[pm.r1], pm.back_v, pm.v);
	Operation::calculate_route_fts(data, s.routes[pm.r1]);
	s.cost += s.routes[pm.r1].cost;
	
	return pm.r1;
}


void Perturb::random_pd_shift(const Data& data, Solution& s, std::vector<size_t>& nr, std::mt19937_64& rng, size_t forb_route){
	std::uniform_int_distribution<int> rand_req(1, data.nnod-2);
	PairMovementTuple pm;
	size_t first_route = rng()%s.size();
	size_t r = first_route;

	int u = rand_req(rng);
	while(nr[u] == forb_route || nr[u] == Def::NO_ROUTE) u = rand_req(rng);
	u = data.nodes[u].type == Def::PICKUP_NODE ? u : data.nodes[u].pair;
	
	size_t ru = nr[u]; //route of node u
	do{
		if(r != forb_route && r != ru){
			int oback_u = pm.back_u;
			int oback_v = pm.back_v;
			pm = Operation::search_insertion_location(data, s.routes[r], u, rng, true);
			pm.r2 = ((oback_u == pm.back_u) && (oback_v == pm.back_v)) ? pm.r2 : r;
		}
		r = (r+1)%s.size();
	}while(pm.back_u == Def::NO_NODE && r != first_route);
	
	if(pm.back_u == Def::NO_NODE){
		return;
	}
	pm.r1 = ru;
	Route& route1 = s.routes[pm.r1];
	Route& route2 = s.routes[pm.r2];
		
	s.cost -= route1.cost + route2.cost;	
	Operation::remove_pair_from_route(data, route1, pm.u);
	Operation::insert_in_route(data, route2, pm.back_u, pm.u);
	Operation::insert_in_route(data, route2, pm.back_v, pm.v);
	nr[pm.u] = pm.r2;
	nr[pm.v] = pm.r2;
	
	Operation::calculate_route_fts(data, route1);
	Operation::calculate_route_fts(data, route2);

	s.cost += route1.cost + route2.cost;
}


void Perturb::random_pd_swap(const Data& data, Solution& s, std::vector<size_t>& nr, std::mt19937_64& rng, size_t forb_route){
	std::uniform_int_distribution<int> rand_req(1, data.nnod-2);

	int u = rand_req(rng);
	while(nr[u] == forb_route || nr[u] == Def::NO_ROUTE) u = rand_req(rng);
	int x = rand_req(rng);
	while(nr[u] == nr[x] || nr[x] == forb_route || nr[x] == Def::NO_ROUTE) x = rand_req(rng);

	u = data.nodes[u].type == Def::PICKUP_NODE ? u : data.nodes[u].pair;
	x = data.nodes[x].type == Def::PICKUP_NODE ? x : data.nodes[x].pair;

	Route route1 = s.routes[nr[u]]; //route of node u
	Route route2 = s.routes[nr[x]];

	Operation::remove_pair_from_route(data, route1, u);
	Operation::calculate_route_fts(data, route1);
	//insert in route 1
	PairMovementTuple rand_pair_1 = Operation::search_insertion_location(data, route1, x, rng);
	if(rand_pair_1.u == Def::NO_NODE) return;
					
	Operation::remove_pair_from_route(data, route2, x);
	Operation::calculate_route_fts(data, route2);
	//insert int route 2
	PairMovementTuple rand_pair_2 = Operation::search_insertion_location(data, route2, u, rng);
	if(rand_pair_2.u == Def::NO_NODE) return;
		
	rand_pair_1.r1 = nr[u]; rand_pair_1.r2 = nr[x];
	rand_pair_2.r1 = nr[u]; rand_pair_2.r2 = nr[x];
	
	if(rand_pair_1.u != Def::NO_NODE && rand_pair_2.u != Def::NO_NODE){
		Route& route1 = s.routes[rand_pair_1.r1];
		Route& route2 = s.routes[rand_pair_1.r2];

		s.cost -= route1.cost+route2.cost;
		
		Operation::remove_pair_from_route(data, route1, rand_pair_2.u);
		Operation::remove_pair_from_route(data, route2, rand_pair_1.u);
		
		Operation::insert_in_route(data, route1, rand_pair_1.back_u, rand_pair_1.u);
		Operation::insert_in_route(data, route1, rand_pair_1.back_v, rand_pair_1.v);
		Operation::insert_in_route(data, route2, rand_pair_2.back_u, rand_pair_2.u);
		Operation::insert_in_route(data, route2, rand_pair_2.back_v, rand_pair_2.v);

		nr[rand_pair_1.u] = rand_pair_1.r1;
		nr[rand_pair_1.v] = rand_pair_1.r1;
		nr[rand_pair_2.u] = rand_pair_2.r2;
		nr[rand_pair_2.v] = rand_pair_2.r2;
		
		Operation::calculate_route_fts(data, route1);
		Operation::calculate_route_fts(data, route2);		

		s.cost += route1.cost+route2.cost;
	}
	
}


void Perturb::original_perturb(const Data& data, const size_t psize, const double prob_shift, Solution& s, size_t& pcount, vector<size_t>& nr, std::mt19937_64& rng, size_t forb_route){
	if(s.size() <= 2) return; //all movements are based on two routes, can't move with only one route (note the other is virtually removed)
	
	std::uniform_real_distribution<double> prob(0, 1);
	size_t perturb = 0;
	double ocost = s.cost;
	//vector<size_t> nr(data.nnod, Def::NO_ROUTE);
	
	while(perturb < psize){
		if(prob(rng) <= prob_shift) Perturb::random_pd_shift(data, s, nr, rng, forb_route);
		else Perturb::random_pd_swap(data, s, nr, rng, forb_route);

		if(std::abs(ocost - s.cost) > Def::EPSILON){
			pcount++;
		}
		ocost = s.cost;
		perturb++;
	}
}
