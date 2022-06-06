#ifndef __H_LNS_H__
#define __H_LNS_H__

#include "../common/data.hpp"
#include "../common/solution.hpp"
#include "../common/timer.hpp"
#include "../common/parameters.hpp"
#include <vector>
#include <random>

class LNS{
private:
	const Data& data;
	const Parameters& par;
	int min_q, max_q;
	int min_k, max_k;
	int lsize;
	int max_iter;
	std::mt19937_64& rng;
	size_t regret_heuristic(const size_t q, Solution &s, std::vector<int>& removed);
	void shaw_removal(const size_t q, Solution& s, std::vector<int>& removed);
	void random_removal(const size_t q, Solution& s, std::vector<int>& removed);
	void worst_removal(const size_t q, Solution& s, std::vector<int>& removed);


	
public:
	//below are indices for LNS related vectors
	static const int REM_SHAW_TYPE = 0;
	static const int REM_RANDOM_TYPE = 1;
	static const int REM_WORST_TYPE = 2;

	static const int WSHAW_ALPHA = 0;
	static const int WSHAW_BETA = 1;
	static const int WSHAW_GAMMA = 2;

	//some vectors for storing information about the LNS execution
	std::vector<size_t> removal_improve;
	std::vector<size_t> removal_called;
	std::vector<size_t> removal_reinsert;
	
	size_t total_iter;
	
	double total_weight;
	std::vector<double> weight_removal;
	
	LNS(const Data& data_, const Parameters &par_, std::mt19937_64& _rng, int _min_q, int _max_q, int _min_k, int _max_k, int _lsize, int _max_iter) : data(data_), par(par_), rng(_rng){
		min_q = _min_q;
		max_q = _max_q;
		if(min_q > max_q) min_q = max_q;
		min_k = _min_k;
		max_k = _max_k;
		lsize = _lsize;
		max_iter = _max_iter;
		removal_improve = std::vector<size_t>(3, 0);
		removal_called = std::vector<size_t>(3, 0);
		removal_reinsert = std::vector<size_t>(3, 0);
		total_iter = 0;
		this->total_weight = 1;
		this->weight_removal = std::vector<double>(3, 0.0);
	};
	
	void solve(Timer& t, Solution& s);

};

#endif //__H_LNS_H__
