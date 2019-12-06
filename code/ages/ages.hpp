#ifndef __H_AGES_H__
#define __H_AGES_H__

#include <random>
#include "../common/data.hpp"
#include "../common/solution.hpp"
#include "../common/timer.hpp"
#include "../common/pairmovementtuple.hpp"
#include <vector>

class AGES{
private:
	const Data& data;
	size_t max_iter;
	//(p)erturbation parameters
	size_t psize;
	double pprob_eval, pptries;
	std::mt19937_64& rng;

	std::vector<PairMovementTuple> try_remove_pair(const Data& data, const Solution& s, const std::vector<int>& pen_count, int u, size_t ru, int kmax, bool random_insertion=true);
	
public:
	size_t pcount;
	AGES(const Data& data_, std::mt19937_64& _rng, const size_t _max_iter, const size_t _psize, const double _pprob_eval=0.6, const double _pptries=1.0) : data(data_), rng(_rng){
		max_iter = _max_iter;
		psize = _psize;
		pprob_eval = _pprob_eval;
		pptries = _pptries;
		pcount = 0;
	};

	//TODO: make the type of method a parameter instead of two explicitly separated methods!
	void solve(Solution& sl, Timer& t, double max_time);
	void original_solve(Solution& sl, Timer& t, double max_time, double prob_shift=0.5);

};

#endif //__H_AGES_H__
