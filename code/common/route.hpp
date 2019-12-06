#ifndef __H_ROUTE_H__
#define __H_ROUTE_H__

#include "defs.hpp"
#include <vector>
#include <cstdio>
#include <boost/dynamic_bitset.hpp>

class Route{
public:
    std::vector<int> forward;
    std::vector<int> backward;
    std::vector<double> reach_time;
	std::vector<double> fts; //forward time slack
    std::vector<int>acc_cap;
	boost::dynamic_bitset<> node_set;
    int sz;
    double cost;

    Route(int n){
		forward = std::vector<int>(n, Def::NO_NODE);
		backward = std::vector<int>(n, Def::NO_NODE);
		forward[0] = backward[0] = n-1;
		backward[n-1] = forward[n-1] = 0;
		reach_time = std::vector<double>(n,0.0);
		fts = std::vector<double>(n,0.0);
		acc_cap = std::vector<int>(n,0);
		node_set = boost::dynamic_bitset<>(n);
		node_set[0] = true;
		node_set[n-1] = true;
		sz = 0;
		cost = 0;
    };

    void insert_node(int b, int i){
		forward[i] = forward[b];
		backward[forward[i]] = i;
		backward[i] = b;
		forward[b] = i;
		sz += 1;
		node_set[i] = true;
    };

    void remove_node(int i){
		forward[backward[i]] = forward[i];
		backward[forward[i]] = backward[i];
		forward[i] = backward[i] = Def::NO_NODE;
		acc_cap[i] = 0;
		reach_time[i] = 0.0;
		sz -= 1;
		node_set[i] = false;
    };

    bool has_node(int i) const{
		return (node_set[i]);
    };

	int size() const{
		return sz;
	}

    void print() const{
		int node = this->forward[0];
		printf("0");
		do{
			printf(" , %i", node);
			node = this->forward[node];
		}while(node != (int)forward.size()-1);
		printf(" , %i\n", node);
    }

	int get_by_index(int index){
		int node = 0;
		int count = 0;
		while(count < index){
			node = this->forward[node];
			count++;
		}
		return node;
	}
    
};

#endif //__H_ROUTE_H__
