#ifndef __H_PERTURB_H__
#define __H_PERTURB_H__

#include "data.hpp"
#include "solution.hpp"
#include <vector>

//another core part of the algorithm, but related solely to the perturbation
//TODO: make this a class instead of a namespace
namespace Perturb{
	void sampled_perturb(const Data& data, const size_t max_perturb, Solution& s, std::vector<size_t>& nr, size_t& ncount, double prob_eval, std::mt19937_64& rng, size_t forb_route=Def::NO_ROUTE);
	size_t sampled_reinsert(const Data& data, const double prob_eval, const double ptries, const int u, Solution &s, std::mt19937_64& rng, size_t forb_route=Def::NO_ROUTE);
	void original_perturb(const Data& data, const size_t psize, const double prob_shift, Solution& s, size_t& pcount, std::vector<size_t>& nr, std::mt19937_64& rng, size_t forb_route=Def::NO_ROUTE);
	void random_pd_shift(const Data& data, Solution& s, std::vector<size_t>& nr, std::mt19937_64& rng, size_t forb_route=Def::NO_ROUTE);
	void random_pd_swap(const Data& data, Solution& s, std::vector<size_t>& nr, std::mt19937_64& rng, size_t forb_route=Def::NO_ROUTE);
};

#endif //__H_PERTURB_H__
