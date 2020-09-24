#ifndef __H_PARAMETERS_H__
#define __H_PARAMETERS_H__

#include "defs.hpp"
#include <vector>

struct Parameters{
	std::string inst_format = Def::INST_LI_LIM_FORMAT;
	double max_time = 60.0;
	std::mt19937_64::result_type seed = 0;

	//matheuristic parameters
	std::string initial_constructor = Def::CONSTRUCTOR_GREEDY;
	bool use_spp = true;
	double ppsize = 0.83;
	double prob_eval = 0.57;
	size_t ges_max_iter = 4000;
	double ges_ppsize = 0.15;
	size_t ges_psize = 100;
	int alg_ages=Def::AGES_NEW_TYPE;
	
	int lns_min_q = 2;
	int lns_min_k = 1;
	int lns_max_k = 4;
	int lns_max_iter = 970;
	int lns_lsize = 1540;
	std::vector<int> lns_wrem = {6,3,1};
	double lns_max_mult = 0.20;
	bool lns_rollback = true;
	std::vector<double> lns_wshaw = {9,4,1};
	
	//IO parameters
	std::string print_type = Def::PRINT_HUMAN_FORMAT;
	bool write_solution = false;
	std::string solution_file;
	std::string log_filename;
	bool save_log = false;
};

#endif //__H_PARAMETERS_H__
