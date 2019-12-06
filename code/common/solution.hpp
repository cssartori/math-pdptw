#ifndef __H_SOLUTION_H__
#define __H_SOLUTION_H__

#include <vector>
#include "route.hpp"
#include <cstdio>
#include <boost/dynamic_bitset.hpp>

class Solution{
public:
    std::vector<Route> routes;
	boost::dynamic_bitset<> node_set;
    double cost;
	int n;

    Solution(int n_){
		node_set = boost::dynamic_bitset<>(n_);
		cost = 0;
		n = n_;
    }

	size_t size() const{
		return routes.size();
	}
	
    void insert_route(Route nroute){
		routes.push_back(nroute);
		node_set = node_set | nroute.node_set;
    }

    void update_route(int r, Route nroute){
		//TODO
    }

    void remove_route(int r){
		node_set = node_set ^ routes[r].node_set;
		routes.erase(routes.begin()+r);
    }

	void clear_routes(){
		for(size_t r=0;r<routes.size();r++){
			if(routes[r].size() == 0){
				remove_route(r);
				r--;
			}
		}
	}
	
	bool has_node(int i) const{
		return node_set[i];
	}

    void print() const{
		int i=0;
		for(auto& r : routes){
			printf("R%i: ", i++);
			r.print();
		}
    }
	
};

#endif //__H_SOLUTION_H__
