#include "ages.hpp"
#include "../validator/validator.hpp"
#include "../common/operation.hpp"
#include "../common/perturb.hpp"
#include <stack>
#include <queue>
#include <boost/dynamic_bitset.hpp>

using namespace std;

size_t random_route(const Solution& s, std::mt19937_64& rng);

void AGES::solve(Solution& sl, Timer& t, double max_time){
	if(sl.size() < 2) return;
	
	std::uniform_real_distribution<double> prob(0, 1);
	size_t iter = 0;
		
	while(sl.size() > 1 && iter < max_iter && t.elapsed_seconds() < max_time && sl.size() >= 2){ 
		printd(("%li - Size: %li\n", iter, sl.size()));
		Solution s = sl;
		vector<int> pen_count(data.nnod,1); //penalty counter
		vector<size_t> nodes_routes(data.nnod, Def::NO_ROUTE);
		Operation::get_all_nodes_routes(data, sl, nodes_routes);
		size_t r = random_route(s, rng);
		Route& route = s.routes[r];

		stack<int> nlist; //list of un-assigned nodes		
		//remove all nodes from route keeping LIFO order
		int node = route.forward[data.initial_depot()];
		while(node != data.final_depot()){
			if(data.nodes[node].type == Def::PICKUP_NODE){
				nlist.push(node);
				s.node_set[node] = false;
				nodes_routes[node] = Def::NO_ROUTE;
			}
			node = route.forward[node];
		}
		
		size_t best_unassign = nlist.size();
		while(sl.size() > 1 && iter < max_iter && not nlist.empty() && t.elapsed_seconds() < max_time){
			node = nlist.top(); nlist.pop();		
			PairMovementTuple move;
			size_t r2 = rng()%s.size();
			size_t first_route = r2;
			
			do{
				if(r2 != r){
					PairMovementTuple mv = Operation::search_all_insertion_locations(data, s.routes[r2], node, r2, rng, true);
					//get the first random movement that is valid! (this is still as random as it gets, and fast)
					if(mv.u != Def::NO_NODE){
						move = mv;
						break;
					}
				}
				r2=(r2+1)%s.size();
			}while(r2 != first_route);

			if(move.u != Def::NO_NODE){ //there is at least one movement
				//apply it to the route
				s.cost -= s.routes[move.r1].cost;
				Operation::insert_in_route(data, s.routes[move.r1], move.back_u, move.u);
				Operation::insert_in_route(data, s.routes[move.r1], move.back_v, move.v);
				s.node_set[move.u] = s.node_set[move.v] = true;
				s.cost += s.routes[move.r1].cost;
				Operation::calculate_route_fts(data, s.routes[move.r1]);
			}else{ //it could not be inserted in any route
				pen_count[node] += 1;
				
				int kmax=1;
				vector<PairMovementTuple> rms;
				rms = try_remove_pair(data, s, pen_count, node, r, kmax, true);
				
				if(rms[0].u == Def::NO_NODE){
					//remove with kmax=2
					rms = try_remove_pair(data, s, pen_count, node, r, ++kmax, true);
				}
			   
				if(rms[0].u != Def::NO_NODE){
					int t=0;
					s.cost -= s.routes[rms[t].r1].cost;
					Operation::remove_pair_from_route(data, s.routes[rms[t].r1], rms[t].u);
					s.node_set[rms[t].u] = s.node_set[data.nodes[rms[t].u].pair] = false;
					nlist.push(rms[t].u);
					nodes_routes[rms[t].u] = Def::NO_ROUTE;
					nodes_routes[data.nodes[rms[t].u].pair] = Def::NO_ROUTE;
					t++;
					
					if(kmax == 2){
						Operation::remove_pair_from_route(data, s.routes[rms[t].r1], rms[t].u);
						s.node_set[rms[t].u] = s.node_set[data.nodes[rms[t].u].pair] = false;
						nlist.push(rms[t].u);
						nodes_routes[rms[t].u] = Def::NO_ROUTE;
						nodes_routes[data.nodes[rms[t].u].pair] = Def::NO_ROUTE;
						t++;
					}
					Operation::insert_in_route(data, s.routes[rms[t].r1], rms[t].back_u, rms[t].u);
					Operation::insert_in_route(data, s.routes[rms[t].r1], rms[t].back_v, rms[t].v);
					s.node_set[rms[t].u] = s.node_set[data.nodes[rms[t].u].pair] = true;
					nodes_routes[rms[t].u] = rms[t].r1;
					nodes_routes[data.nodes[rms[t].u].pair] = rms[t].r1;
					s.cost += s.routes[rms[t].r1].cost;
					Operation::calculate_route_fts(data, s.routes[rms[t].r1]);
				}else{
					nlist.push(node);
				}

				Perturb::sampled_perturb(data, psize, s, nodes_routes, pcount, pprob_eval, rng, r);
				iter++;
			}

			if(nlist.size() < best_unassign){
				best_unassign = nlist.size();
				iter = 0;
			}
		}

		if(nlist.empty() && (s.cost-s.routes[r].cost) < sl.cost){
			s.cost -= s.routes[r].cost;
			s.remove_route(r);
			sl = s;
			//iter=0;
			s.node_set.set();
			s.clear_routes();
		}
	}
	
	sl.node_set.set();
}

std::vector<PairMovementTuple> AGES::try_remove_pair(const Data& data, const Solution& s, const std::vector<int>& pen_count, int u, size_t ru, int kmax, bool random_insertion){
	std::vector<PairMovementTuple> rms(kmax+1);
	Solution sl = s;
	int pbest = std::numeric_limits<int>::max();
	size_t first_route = rng()%s.size();
	size_t r = first_route;
	
	do{
		//avoid selecting the old route of u, it is, in practice, deleted
		if(r == ru){
			r=(r+1)%s.size();
			continue;
		}
		int i=s.routes[r].forward[data.initial_depot()];
		int j=s.routes[r].forward[i];
		
		while(i != data.final_depot()){
			int nd1 = i;
			int nd2 = Def::NO_NODE;
			
			if(data.nodes[nd1].type == Def::PICKUP_NODE && pen_count[nd1] < pbest){
				boost::dynamic_bitset<> ns = s.routes[r].node_set;
				ns[nd1] = false;
				ns[data.nodes[nd1].pair] = false;
				if(kmax == 2 && data.nodes[j].type == Def::PICKUP_NODE && pen_count[nd1]+pen_count[j] < pbest){
					nd2 = j;
					if(nd2 != Def::NO_NODE && data.nodes[nd2].type == Def::PICKUP_NODE){
						ns[nd2] = false;
						ns[data.nodes[nd2].pair] = false;
					}
				}
				
				if((kmax == 1 || (kmax == 2 && nd2 != Def::NO_NODE)) && (ns & data.conflicts[u]).none()){
					Route rr = s.routes[r];
					PairMovementTuple move;
					Operation::remove_pair_from_route(data, rr, nd1);
					if(nd2 != Def::NO_NODE) Operation::remove_pair_from_route(data, rr, nd2);
					Operation::calculate_route_fts(data, rr);
					move = Operation::search_all_insertion_locations(data, rr, u, r, rng, random_insertion);
					if(move.u != Def::NO_NODE){
						pbest = pen_count[nd1] + (kmax == 2 ? pen_count[nd2] : 0);
						PairMovementTuple pmt1, pmt2;
						pmt1.u = nd1; pmt1.r1 = r;
						pmt2.u = nd2; pmt2.r1 = r;
						int t=0;
						rms[t++] = pmt1;
						if(kmax == 2) rms[t++] = pmt2;
						rms[t] = move;
						if(pbest == kmax) return rms; //kmax is the lowest value possible for pbest
					}
				}			
			}
			
			if(kmax == 1)
				i=s.routes[r].forward[i];
			else{
				j=s.routes[r].forward[j];
				if(j == data.final_depot()){
					i=s.routes[r].forward[i];
					j=s.routes[r].forward[i];
				}
			}

		}
		r=(r+1)%s.size();
	}while(r != first_route);
	
	return rms;
}

size_t random_route(const Solution& s, std::mt19937_64& rng){
	std::uniform_int_distribution<size_t> rand_route(0, s.size() - 1);
	return rand_route(rng);
}

void AGES::original_solve(Solution& sl, Timer& t, double max_time, double prob_shift){
	std::uniform_real_distribution<double> prob(0, 1);
	size_t iter = 0;
	
	while(sl.size() > 1 && iter < max_iter && t.elapsed_seconds() < max_time){ 
		printd(("%li - Size: %li\n", iter, sl.size()));
		vector<int> pen_count(data.nnod,1); //penalty counter
		Solution s = sl;
		vector<size_t> nodes_routes(data.nnod, Def::NO_ROUTE);
		Operation::get_all_nodes_routes(data, sl, nodes_routes);
		size_t r = random_route(s, rng);
		Route& route = s.routes[r];

		stack<int> nlist; //list of un-assigned nodes
		//remove all nodes from route keeping LIFO order
		int node = route.forward[data.initial_depot()];
		while(node != data.final_depot()){
			if(data.nodes[node].type == Def::PICKUP_NODE){
				nlist.push(node);
				s.node_set[node] = false;
				nodes_routes[node] = Def::NO_ROUTE;
			}
			node = route.forward[node];
		}
		
		size_t best_unassign = nlist.size();
		while(sl.size() > 1 && iter < max_iter && not nlist.empty() && t.elapsed_seconds() < max_time){
			node = nlist.top(); nlist.pop();	
			PairMovementTuple move;
			size_t r2 = rng()%s.size();
			size_t first_route = r2;
			
			do{
				if(r2 != r){
					PairMovementTuple mv = Operation::search_all_insertion_locations(data, s.routes[r2], node, r2, rng, true);
					//get the first random movement that is valid! (this is still as random as it gets, and fast)
					if(mv.u != Def::NO_NODE){
						move = mv;
						break;
					}
				}
				r2=(r2+1)%s.size();
			}while(r2 != first_route);

			if(move.u != Def::NO_NODE){ //there is at least one movement
				//apply it to the route
				s.cost -= s.routes[move.r1].cost;
				Operation::insert_in_route(data, s.routes[move.r1], move.back_u, move.u);
				Operation::insert_in_route(data, s.routes[move.r1], move.back_v, move.v);
				s.node_set[move.u] = s.node_set[move.v] = true;
				s.cost += s.routes[move.r1].cost;
				Operation::calculate_route_fts(data, s.routes[move.r1]);
			}else{ //it could not be inserted in any route
				pen_count[node] += 1;
				
				int kmax=1;
				vector<PairMovementTuple> rms;
				rms = try_remove_pair(data, s, pen_count, node, r, kmax, true);
				if(rms[0].u == Def::NO_NODE){
					rms = try_remove_pair(data, s, pen_count, node, r, ++kmax, true);
				}
			   
				if(rms[0].u != Def::NO_NODE){
					int t=0;
					s.cost -= s.routes[rms[t].r1].cost;
					Operation::remove_pair_from_route(data, s.routes[rms[t].r1], rms[t].u);
					s.node_set[rms[t].u] = s.node_set[data.nodes[rms[t].u].pair] = false;
					nlist.push(rms[t].u);
					nodes_routes[rms[t].u] = Def::NO_ROUTE;
					nodes_routes[data.nodes[rms[t].u].pair] = Def::NO_ROUTE;
					t++;
					
					if(kmax == 2){
						Operation::remove_pair_from_route(data, s.routes[rms[t].r1], rms[t].u);
						s.node_set[rms[t].u] = s.node_set[data.nodes[rms[t].u].pair] = false;
						nlist.push(rms[t].u);
						nodes_routes[rms[t].u] = Def::NO_ROUTE;
						nodes_routes[data.nodes[rms[t].u].pair] = Def::NO_ROUTE;
						t++;
					}
					Operation::insert_in_route(data, s.routes[rms[t].r1], rms[t].back_u, rms[t].u);
					Operation::insert_in_route(data, s.routes[rms[t].r1], rms[t].back_v, rms[t].v);
					s.node_set[rms[t].u] = s.node_set[data.nodes[rms[t].u].pair] = true;
					nodes_routes[rms[t].u] = rms[t].r1;
					nodes_routes[data.nodes[rms[t].u].pair] = rms[t].r1;
					s.cost += s.routes[rms[t].r1].cost;
					Operation::calculate_route_fts(data, s.routes[rms[t].r1]);
				}else{
					nlist.push(node);
				}

				Perturb::original_perturb(data, psize, prob_shift, s, pcount, nodes_routes, rng, r);
				iter+=psize;
			}

			if(nlist.size() < best_unassign){
				best_unassign = nlist.size();
				iter = 0;
			}
		}

		if(nlist.empty()){
			s.cost -= s.routes[r].cost;
			s.remove_route(r);
			sl = s;
			//iter=0;
			s.node_set.set();
			s.clear_routes();
		}
	}
	
	sl.node_set.set();
}
