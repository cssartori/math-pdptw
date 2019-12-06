#include "operation.hpp"

using namespace std;

/*Insert node u after node i in route, updating information accordingly*/
void Operation::insert_in_route(const Data& data, Route& route, int i, int u){
	/*set accumulated capacity for node u*/
	route.acc_cap[u] = route.acc_cap[i] + data.nodes[u].demand;
	
	double departure_time_i = 0.0;//time to leave node i
	if(i != data.initial_depot())
	 	departure_time_i = std::max(route.reach_time[i], data.nodes[i].etw) + data.nodes[i].stw;
	
	route.reach_time[u] = departure_time_i + data.times[i][u]; //time to reach node u
	double departure_time_u = std::max(route.reach_time[u], data.nodes[u].etw) + data.nodes[u].stw; //time to leave node u

	int node = route.forward[i]; //future forward of node u
	double rtime_n = departure_time_u + data.times[u][node]; //time to reach node
	//update all reach time and capacity information
	do{
		route.reach_time[node] = rtime_n;
		route.acc_cap[node] += data.nodes[u].demand;

		double departure_time_n = std::max(rtime_n , data.nodes[node].etw)+data.nodes[node].stw;
		rtime_n = departure_time_n + data.times[node][route.forward[node]];
		node = route.forward[node];	
	}while(node != route.forward[data.final_depot()]);
	
	//update route distance
	double add_cost = c_in(data, i, u, route.forward[i]);
	route.insert_node(i, u);	
	route.cost += add_cost;
}

/*Remove node u and its delivery pair from route, updating information accordingly*/
void Operation::remove_pair_from_route(const Data& data, Route& route, int u){
	int v = data.nodes[u].pair;
	int i = route.backward[u];
	double departure_time_i = 0.0;
	
	if(i != data.initial_depot())
	 	departure_time_i = std::max(route.reach_time[i], data.nodes[i].etw) + data.nodes[i].stw;

	int node = route.forward[u];
	int add_cap = -data.nodes[u].demand;
	if(node == v){
		node = route.forward[node];
		add_cap = 0;
	}
	double rtime_n = departure_time_i + data.times[i][node]; //time to reach node 

	//update all reach time and capacity information
	while(node != route.forward[data.final_depot()]){
		if(node == v){
			if(route.backward[node] == u){
				rtime_n -= data.times[i][node];
				rtime_n += data.times[i][route.forward[node]];
			}else{
				rtime_n -= data.times[route.backward[node]][node];
				rtime_n += data.times[route.backward[node]][route.forward[node]];
			}
			add_cap = 0;
		}else{
			route.reach_time[node] = rtime_n;
			route.acc_cap[node] += add_cap;

			double departure_time_n = std::max(rtime_n , data.nodes[node].etw)+data.nodes[node].stw;
			rtime_n = departure_time_n + data.times[node][route.forward[node]];
		}
		node = route.forward[node];	
	}
	
	//update route distance
	double add_cost = -c_in(data, route.backward[u], u, route.forward[u]);
	route.remove_node(u);
	add_cost += (-c_in(data, route.backward[v], v, route.forward[v]));	
	route.remove_node(v);
	route.cost += add_cost;
}

void Operation::calculate_route_fts(const Data& data, Route& route){
	int node = data.final_depot();
	double fts_n1 = data.nodes[node].ltw - std::max(route.reach_time[node], data.nodes[node].etw);
	route.fts[node] = fts_n1;
	node = route.backward[node];
	do{
		double start_serv_n = std::max(route.reach_time[node], data.nodes[node].etw);
		double fts_n = data.nodes[node].ltw - start_serv_n;
		fts_n = std::min(fts_n1, fts_n) + start_serv_n - route.reach_time[node];
		fts_n1 = fts_n;
		route.fts[node] = fts_n;
		node = route.backward[node];
	}while(node != data.initial_depot());
}

void Operation::calculate_solution_fts(const Data& data, Solution& s){
	for(size_t r=0;r<s.size();r++){
		calculate_route_fts(data, s.routes[r]);
	}
}

/*Cost to insert node u between i and j*/
double Operation::c_in(const Data& data, int i, int u, int j){
	return (data.costs[i][u] + data.costs[u][j] - data.costs[i][j]);
}


PairMovementTuple Operation::search_insertion_location(const Data& data, Route& route, int u, std::mt19937_64& rng, bool random){
	std::uniform_real_distribution<double> prob(0, 1);
	PairMovementTuple best_pair;
	best_pair.cost = Def::INF;
	
	int i=data.initial_depot(); //u will be inserted after i in route
	int n_sz = 0;
	do{
		int k = route.forward[i];
		double departure_time_i = 0.0; //time to leave node i (default is when i == 0)	
		if(i != data.initial_depot())
			departure_time_i = std::max(route.reach_time[i], data.nodes[i].etw) + data.nodes[i].stw; //time to leave node i
		
		double rtime_u = departure_time_i + data.times[i][u];
		double start_serv_u = std::max(rtime_u, data.nodes[u].etw);
		
		if(start_serv_u > data.nodes[u].ltw) //assuming triangular inequality, if i->u is invalid, any path i->j->u will also be invalid
			break;
		
		double departure_time_u = start_serv_u + data.nodes[u].stw;
		double new_rtime_k = departure_time_u + data.times[u][k];

		bool valid = true;
		if((new_rtime_k - route.reach_time[k]) > route.fts[k]){
			valid = false;
		}
		double icost = Operation::c_in(data, i, u, route.forward[i]);
		//checks if inserting u after i in route does not violate constraints
		if(valid && (random || icost <= best_pair.cost)){
			route.insert_node(i, u);
			int v = data.nodes[u].pair; //the delivery pair of pickup node u
			int j = u; //v will be inserted after j in route
			
			double rtime_j = departure_time_i + data.times[i][j]; //time to reach node j (in this case, u)
			
			route.acc_cap[u] = route.acc_cap[i];
			/*try to insert node v after every node j untill the depot*/
			while(j != data.final_depot()){
				/*If the vehicle is overloaded at node j, node v can't be inserted AFTER it (solution will be infeasible)'*/
				if(route.acc_cap[j] + data.nodes[u].demand > data.vcap)
					break;
								
				double departure_time_j = std::max(rtime_j, data.nodes[j].etw)+data.nodes[j].stw;
				double rtime_v = departure_time_j + data.times[j][v];
				double start_serv_v = std::max(rtime_v, data.nodes[v].etw);
				
				if(start_serv_v > data.nodes[v].ltw){ //assuming triangular inequality, if j->v is invalid, any path j->h->v will also be invalid
					break;
				}

				int m = route.forward[j];
				double departure_time_v = start_serv_v + data.nodes[v].stw;
				double new_rtime_m = departure_time_v + data.times[v][m];

				bool valid2 = true;
				if((new_rtime_m - route.reach_time[m]) > (route.fts[m])){
					valid2 = false;
				}
				
				/*Checks if inserting v after j in route 2 does not violate constraints*/
				if(valid2){
					double ci = Operation::c_in(data, i, u, route.forward[u]) + Operation::c_in(data, j, v, route.forward[j]);
					n_sz++;
					if((not random && ci < best_pair.cost) || (random && prob(rng) <= 1.0/n_sz)){
						//only set pair if solution gets better			
						best_pair.back_u = i; best_pair.u = u;
						best_pair.back_v = j; best_pair.v = v;
						best_pair.cost = ci;
					}
				}
				rtime_j = departure_time_j + data.times[j][route.forward[j]];
				j = route.forward[j];
			}
			route.remove_node(u);
		}		
		i = route.forward[i];
	}while(i != data.final_depot());
	
	return best_pair;	
}


PairMovementTuple Operation::search_all_insertion_locations(const Data& data, Route& route, int u, int rindex, std::mt19937_64& rng, bool random){
	PairMovementTuple pmt;
	pmt = search_insertion_location(data, route, u, rng, random);
	pmt.r1 = rindex;
	pmt.r2 = rindex;
	return pmt;
}

size_t Operation::get_node_route(const Solution& s, int u){
	for(size_t r=0;r<s.size();r++){
		if(s.routes[r].has_node(u)) return r;
	}
	return Def::NO_ROUTE;
}

void Operation::get_all_nodes_routes(const Data& data, const Solution& s, std::vector<size_t>& nr){
	for(size_t r=0;r<s.size();r++){
		int node = s.routes[r].forward[data.initial_depot()];
		while(node != data.final_depot()){
			nr[node] = r;
			node = s.routes[r].forward[node];
		}
	}
}

void Operation::compute_route_values(const Data& data, Route& route){
	int u = data.initial_depot();
	double rtime = 0.0;
	int cap = 0;
	double cost = 0.0;
	while(u != data.final_depot()){
		int fu = route.forward[u];
		route.reach_time[u] = rtime;
		route.acc_cap[u] = cap + data.nodes[u].demand;
		double serv_time_u = std::max(rtime, data.nodes[u].etw);
		double leave_time_u = serv_time_u + data.nodes[u].stw;
		rtime = leave_time_u + data.times[u][fu];
		cost += data.costs[u][fu];
		cap = route.acc_cap[u];
		u = fu;
	}
	
	route.reach_time[u] = rtime;
	route.acc_cap[u] = 0;
	route.cost = cost;
}

void Operation::remove_pairs_from_solution(const Data& data, const size_t q, const vector<int>& pairs, Solution& s){
	vector<bool> rmod(s.size(), false);
	for(size_t i=0;i<q;i++){
		size_t r = get_node_route(s, pairs[i]);
		rmod[r] = true;
		int u = pairs[i];
		int v = data.nodes[u].pair;
		s.routes[r].remove_node(u);
		s.routes[r].remove_node(v);
	}

	for(size_t i=0;i<rmod.size();i++){
		if(rmod[i]){
			s.cost -= s.routes[i].cost;
			compute_route_values(data, s.routes[i]);
			calculate_route_fts(data, s.routes[i]);
			s.cost += s.routes[i].cost;
		}
	}
}

//case where routes of each node are known in advance
void Operation::remove_pairs_from_solution(const Data& data, const size_t q, const vector<int>& pairs, const vector<size_t>& nr, Solution& s){
	for(size_t i=0;i<q;i++){
		size_t r = nr[pairs[i]];
		int u = pairs[i];
		int v = data.nodes[u].pair;
		s.routes[r].remove_node(u);
		s.routes[r].remove_node(v);
	}

	for(size_t i=0;i<q;i++){
		size_t r = nr[pairs[i]];
		s.cost -= s.routes[r].cost;
		compute_route_values(data, s.routes[r]);
		calculate_route_fts(data, s.routes[r]);
		s.cost += s.routes[r].cost;
	}
}

PairMovementTuple Operation::sampled_search_insertion_location(const Data& data, Route& route, int u, size_t& tries, std::mt19937_64& rng, double prob_eval, PairMovementTuple pm){
	std::uniform_real_distribution<double> prob(0, 1);
	PairMovementTuple best_pair = pm;
	
	int i=data.initial_depot(); //u will be inserted after i in route
	do{
		int k = route.forward[i];
		double departure_time_i = 0.0; //time to leave node i (default is when i == 0)	
		if(i != data.initial_depot())
			departure_time_i = std::max(route.reach_time[i], data.nodes[i].etw) + data.nodes[i].stw; //time to leave node i
		
		double rtime_u = departure_time_i + data.times[i][u];
		double start_serv_u = std::max(rtime_u, data.nodes[u].etw);
		
		if(start_serv_u > data.nodes[u].ltw) //assuming triangular inequality, if i->u is invalid, any path i->j->u will also be invalid
			return best_pair;
		
		double departure_time_u = start_serv_u + data.nodes[u].stw;
		double new_rtime_k = departure_time_u + data.times[u][k];

		bool valid = true;
		if((new_rtime_k - route.reach_time[k]) > route.fts[k]){
			valid = false;
		}
				
		//checks if inserting u after i in route does not violate constraints
		if(valid){
			route.insert_node(i, u);
			int v = data.nodes[u].pair; //the delivery pair of pickup node u
			int j = u; //v will be inserted after j in route
			
			double rtime_j = departure_time_i + data.times[i][j]; //time to reach node j (in this case, u)
			
			route.acc_cap[u] = route.acc_cap[i];
			/*try to insert node v after every node j untill the depot*/
			while(j != data.final_depot()){
				/*If the vehicle is overloaded at node j, node v can't be inserted AFTER it (solution will be infeasible)'*/
				if(route.acc_cap[j] + data.nodes[u].demand > data.vcap)
					break;
								
				double departure_time_j = std::max(rtime_j, data.nodes[j].etw)+data.nodes[j].stw;
				double rtime_v = departure_time_j + data.times[j][v];
				double start_serv_v = std::max(rtime_v, data.nodes[v].etw);
				
				if(start_serv_v > data.nodes[v].ltw){ //assuming triangular inequality, if j->v is invalid, any path j->h->v will also be invalid
					break;
				}

				int m = route.forward[j];
				double departure_time_v = start_serv_v + data.nodes[v].stw;
				double new_rtime_m = departure_time_v + data.times[v][m];

				bool valid2 = true;
				if((new_rtime_m - route.reach_time[m]) > (route.fts[m])){
					valid2 = false;
				}
				
				/*Checks if inserting v after j in route 2 does not violate constraints*/
				if(valid2){
					if(prob(rng) <= prob_eval){ //probabilistic acceptance
						tries++;
						double ci = Operation::c_in(data, i, u, route.forward[u]) + Operation::c_in(data, j, v, route.forward[j]);
						if(ci < best_pair.cost){
							//only set pair if solution gets better			
							best_pair.back_u = i; best_pair.u = u;
							best_pair.back_v = j; best_pair.v = v;
							best_pair.cost = ci;
						}
					}
				}
				
				rtime_j = departure_time_j + data.times[j][route.forward[j]];
				j = route.forward[j];
			}
			route.remove_node(u);
		}		
		i = route.forward[i];
	}while(i != data.final_depot());

	return best_pair;	
}
