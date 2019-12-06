#include "validator.hpp"
#include "../common/defs.hpp"
#include <cstdlib>
#include <vector>
#include <cstdio>
#include <iostream>
#include <cmath>

using namespace std;

/*Validate the solution according to the PDPTW constraints. Return true when valid*/
bool Validator::validate_solution(const Data& data, const Solution& s){
	double total_dist = 0; //total solution distance
	vector<bool> visited(data.nnod, false); //check if a node was already visited
	visited[0] = true; visited[data.nnod-1] = true;
	
	for(int r=0;r<(int)s.routes.size();r++){
		/*For each route r in solution s*/
		Route route = s.routes[r];

		int node = 0; //first node is the depot (0)
		double rdist = 0; //total distance of route r
		double rtime = 0; //total time of route r
		int load = 0; //total load of vehicle in rotue r
			
		do{
			//update cost and time
			rdist += data.costs[node][route.forward[node]];
			rtime += data.times[node][route.forward[node]];
			//get next node in route
			node = route.forward[node];
			rtime = std::max(rtime, data.nodes[node].etw); 

			if(node != data.initial_depot() && node != data.final_depot() && visited[node]){
				printf("Node %i has already been visited!\n", node);
				return false;
			}

			//if node is delivery and the pickup pair has not been visited yet, the precedence constraint is violated
			if(data.nodes[node].type == Def::DELIVERY_NODE && visited[data.nodes[node].pair] == false){
				printf("Pickup was not visited before delivery! %i\n", node);
				return false;
			}
			
			//if the max time window constraint is violated: reach time > LTW
			if(rtime > data.nodes[node].ltw){
				printf("%i : Reached %i out of TW: %.4f < %.4f\n", r, node, data.nodes[node].ltw , rtime);
				return false;
			}
			
			//leave time of node: reach time + service
			rtime += data.nodes[node].stw;
			
			//update load
			load += data.nodes[node].demand;
			//if the load is smaller than the necessary to serve this delivery, the load constraint is violated
			if(load < 0){
				printf("load is invalid: %i < %i\n", load, data.nodes[node].demand);
				return false;
			}else if(load > data.vcap){ //load is bigger than the maximum allowed
				printf("load invalid: %i\n", load);
				return false;
			}
			
			//set this node as visited
			visited[node] = true;			
		}while(node != data.final_depot());

		//if the route cost calculated does not match the informed
		if(std::abs(route.cost - rdist) > Def::EPSILON){
			printf("wrong route %i cost : %.4f != %.4f\n", r, route.cost, rdist);
			return false;
		}
		//update the total cost of the solution
		total_dist += rdist;		
	}
	
	//if the solution cost does not match the informed
	if(std::abs((s.cost - total_dist)) > Def::EPSILON){
		printf("wrong sol dist: %.4f != %.4f\n", s.cost, total_dist);
		return false;
	}

	for(int i=0;i<data.nnod;i++){
		if(not visited[i]) return false;
	}
	
	return true;
}
