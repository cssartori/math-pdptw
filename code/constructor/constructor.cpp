#include "constructor.hpp"
#include <random>

Solution Constructor::trivial_heuristic(){
	Solution s(data.nnod); //solution to be returned

	for(int i=0;i<data.nnod;i++){
		if(data.nodes[i].type != Def::PICKUP_NODE) continue;
		Route route(data.nnod);
		route = insert_in_route(route, data.initial_depot(), i);
		route = insert_in_route(route, i, data.nodes[i].pair);
		s.insert_route(route);
		s.cost += route.cost;
	}
	
	//return the constructed feasible solution
	return s;
}


/*Check if inserting u after node i in route is a valid movement*/
bool Constructor::is_valid_insertion(const Route& route, int i, int u, double rtime_i){
	bool valid = false;
	double departure_time_i = 0.0; //default case is when i == 0
		
	if(i != data.initial_depot())
		departure_time_i = std::max(rtime_i, data.nodes[i].etw) + data.nodes[i].stw; //time to leave node i

	double rtime_u = departure_time_i + data.times[i][u]; //time to reach node u from i
	double start_serv_u = std::max(rtime_u, data.nodes[u].etw); //time for service to start at node u
	//If service start at node u before its latest time window, and if the vehicle is not overloaded
	if(start_serv_u <= data.nodes[u].ltw && route.acc_cap[i] + data.nodes[u].demand <= data.vcap){
		double departure_time_u = start_serv_u + data.nodes[u].stw; //time to leave node u
		int node = route.forward[i]; //future forward of node u
		double rtime_n = departure_time_u + data.times[u][node]; //time to reach node

		valid = true;
		//Check each following time window so that there is no violation of it
		do{
			double departure_time_n = std::max(rtime_n, data.nodes[node].etw);
				
			if(departure_time_n > data.nodes[node].ltw){
				valid = false;
				break;
			}else if(std::abs(departure_time_n - std::max(route.reach_time[node], data.nodes[node].etw)) <= Def::EPSILON){
				//if the time to start service at node is the same as before, the rest of the route will have no modifications
				break;
			}
			
			departure_time_n +=  data.nodes[node].stw;
			rtime_n = departure_time_n + data.times[node][route.forward[node]];
			node = route.forward[node];
		}while(node != route.forward[data.final_depot()]);
	}

	return valid;
}


/*Cost type 1 to insert node u between i and j*/
double Constructor::c1(int i, int u, int j){
	return (data.costs[i][u] + data.costs[u][j] - data.costs[i][j]);
}

/*Insert node u after node i in route, updating information accordingly*/
Route Constructor::insert_in_route(Route route, int i, int u){
	/*set accumulated capacity for node u*/
	route.acc_cap[u] = route.acc_cap[i] + data.nodes[u].demand;
	
	double departure_time_i = 0.0;//time to leave node i
	if(i != data.initial_depot())
		departure_time_i = std::max(route.reach_time[i], data.nodes[i].etw) + data.nodes[i].stw;
	
	route.reach_time[u] = departure_time_i + data.times[i][u];  //time to reach node u
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
	
	//update route cost
	double add_dist = data.costs[i][u]+data.costs[u][route.forward[i]] - data.costs[i][route.forward[i]];	
	route.insert_node(i, u);	
	route.cost += add_dist;
		
	return route;
}


/*Get the PD pair that minimizes cost cost when inserting in route*/
PairMovementTuple Constructor::get_best_pd_pair(Route route, const std::vector<bool>& routed, int k){
	PairMovementTuple bp; //best pair for insertion
	std::vector<PairMovementTuple> vb;

	for(int u=1;u<data.nnod-1;u++){ //u is the node trying to be inserted
		if(routed[u] == true || data.nodes[u].type == Def::DELIVERY_NODE)
			continue;
		
		int i = 0;// u will be inserted after i in route
		do{
		
			if(is_valid_insertion(route, i, u, route.reach_time[i])){ //if inserting u after i in route is a valid insertion
				//Route or_route = route; //keep original route
				route.insert_node(i, u); //insert node u in route
				int v =  data.nodes[u].pair; //v is the delivery pair of node u
				int j = u; //v will be inserted after j in route
				double departure_time_i = 0.0; //time to leave node i (default is when i == 0)
				
				if(i != data.initial_depot())
					departure_time_i = std::max(route.reach_time[i], data.nodes[i].etw) + data.nodes[i].stw; //time to leave node i

				double rtime_j = departure_time_i + data.times[i][j]; //time to reach node j (in this case, u)
				/*try to insert node v after every node j untill the depot*/
				while(j != data.final_depot()){
					/*If the vehicle is overloaded at node j, node v can't be inserted AFTER it (solution will be infeasible)'*/
					if(route.acc_cap[j]+data.nodes[u].demand > data.vcap)
						break;
					if(is_valid_insertion(route, j, v, rtime_j)){
						PairMovementTuple mp(i, u, route.forward[u], j, v, route.forward[j], Def::NO_ROUTE, Def::NO_ROUTE, false, (c1(i, u, route.forward[u])+c1(j, v, route.forward[j])));
							
						if((int)vb.size() < k) vb.push_back(mp);
						else{
							int mini = -1;
							for(int p=0;p<k;p++){
								if(vb[p].cost > mp.cost)
									if((mini >= 0 && vb[p].cost > vb[mini].cost) || (mini < 0)) mini = p;
							}

							if(mini >= 0){
								vb[mini] = mp;
							}
						}
					}
					double departure_time_j = std::max(rtime_j, data.nodes[j].etw) + data.nodes[j].stw;
					rtime_j = departure_time_j + data.times[j][route.forward[j]];
					j = route.forward[j];
				}
				//Roll back route to its original state
				route.remove_node(u);
			}
			i = route.forward[i];
		}while(i != data.final_depot());
	}
	if (vb.size() >= 1) {
    std::uniform_int_distribution<size_t> chooser(0, vb.size() - 1);
		bp = vb[chooser(rng)];
  }
		
	return bp;
}


/*Get the first node to insert in route*/
int Constructor::get_route_first_node(const std::vector<bool>& routed, int type){
	int n = -1;
	if(type == 1){
		double d = Def::INF;
		double t = Def::INF;
		for(int i=1;i<data.nnod-1;i++){
			if(data.nodes[i].type == Def::PICKUP_NODE 
			   && routed[i] == false
			   && data.nodes[i].etw <= t){
				if(std::abs(t - data.nodes[i].etw) <= Def::EPSILON){
					if(d >= data.costs[0][i]){
						n = i;
						d = data.costs[0][i];
					}
				}else{
					n = i;
					t = data.nodes[i].etw;
					d = data.costs[0][i];
				}
			}
		}
	}else{
		double cost = Def::INF;
		for(int i=1;i<data.nnod-1;i++){
			if(data.nodes[i].type == Def::DELIVERY_NODE || routed[i]) continue;
			double icost = data.costs[data.initial_depot()][i] + data.costs[i][data.nodes[i].pair] + data.costs[data.nodes[i].pair][data.final_depot()];
			if(icost <= cost){
				cost = icost;
				n = i;
			}
		}
	}
	
	return n;
}

Solution Constructor::greedy_heuristic(int k){
	Solution s(data.nnod); //solution to be returned
	std::vector<bool> routed(data.nnod, false); //indicates wheter a node was already routed
	int req_attended = 0;	//indicates how many requests have been attended
	
	/*While there are requests to be attended and vehicles to do so*/
	while(req_attended < data.nreq){
		Route route(data.nnod);
		int u = get_route_first_node(routed, 1);	    
		int back_u = 0;
		int back_v = u;
		//while there are PD pairs to be inserted in this route
		while(u != Def::NO_NODE){
			req_attended++;
			//Insert PD pair in route
			route = insert_in_route(route, back_u, u); //pickup node
			route = insert_in_route(route, back_v, data.nodes[u].pair); //delivery node
			//mark it as routed
			routed[u] = true;
			routed[data.nodes[u].pair] = true;
			//Get another PD pair to insert
			PairMovementTuple best_pd = get_best_pd_pair(route, routed, k);
			back_u = best_pd.back_u;
			u = best_pd.u;
			//	printf("r.d = %.4f\n", route.cost);
			//route.print();
			back_v = best_pd.back_v;
		}
		//if no node could be inserted in this route, it is an error
		if(route.size() == 0)
			break;
		//insert the new route in solution	
		s.insert_route(route);
		s.cost += route.cost;
	}
	
	//if not all requests were attended, there is an error
	if(req_attended < data.nreq){
		std::cout << "Could not find feasible solution" << std::endl;
		exit(-1);
	}
	
	//return the constructed feasible solution
	return s;
}
