#ifndef __H_RESULT_INFO_H__
#define __H_RESULT_INFO_H__

#include "solution.hpp"
#include "../validator/validator.hpp"
#include "data.hpp"
#include <cstdio>
#include <string>

class ResultInfo{
public:
	Solution si; // initial solution
	double initial_time; // time to create initial solution
	size_t ges_num_reduced; // number of times the GES heuristic was able to reduce #vehicles
	double ges_total_time; // total time spent by GES heuristic
	double lns_total_time; // total time spent by LNS
	size_t spp_num_improve; // number of times SPP was able to improve solution
	double spp_total_time; // total time spent by SPP
	size_t spp_pool_size; // size of the SPP pool of routes
	Solution sb; // best solution
	size_t iter; // number of algorithm iterations
	size_t nimp; // number of times (iterations) the best solution was improved
	double total_time; // total algorithm time

	ResultInfo() : si(0), sb(0){
		initial_time = 0.0;
		ges_num_reduced = 0; 
		ges_total_time = 0.0; 
		lns_total_time = 0.0;
		spp_num_improve = 0;
		spp_total_time = 0.0;
		spp_pool_size = 0;
		iter = 0;
		nimp = 0;
		total_time = 0.0;
	}

	void print_table_format(const Data& data){
		printf("%li;%.2f;%.2f;%i;%.2f;%li;%.2f;%.2f;%i;%.2f;%li;%li;%li;%.2f;%.2f;%li;%li;%.2f\n",
			   this->si.size(),
			   this->si.cost-Def::VEHICLE_COST*this->si.size(),
			   this->si.cost,
			   Validator::validate_solution(data,this->si),
			   this->initial_time,
			   this->sb.size(),
			   this->sb.cost-Def::VEHICLE_COST*this->sb.size(),
			   this->sb.cost,
			   Validator::validate_solution(data,this->sb),
			   this->total_time,
			   this->iter,
			   this->nimp,
			   this->ges_num_reduced,
			   this->ges_total_time,
			   this->lns_total_time,
			   this->spp_num_improve,
			   this->spp_pool_size,
			   this->spp_total_time
			   );
	}

	std::string to_string(const Data& data){
		char str[500];
		sprintf(str, "%li;%.2f;%.2f;%i;%.2f;%li;%.2f;%.2f;%i;%.2f;%li;%li;%li;%.2f;%.2f;%li;%li;%.2f\n",
			   this->si.size(),
			   this->si.cost-Def::VEHICLE_COST*this->si.size(),
			   this->si.cost,
			   Validator::validate_solution(data,this->si),
			   this->initial_time,
			   this->sb.size(),
			   this->sb.cost-Def::VEHICLE_COST*this->sb.size(),
			   this->sb.cost,
			   Validator::validate_solution(data,this->sb),
			   this->total_time,
			   this->iter,
			   this->nimp,
			   this->ges_num_reduced,
			   this->ges_total_time,
			   this->lns_total_time,
			   this->spp_num_improve,
			   this->spp_pool_size,
			   this->spp_total_time
			   );

		return std::string(str);
	}

	void print_irace_format(const Data& data){
		this->print_table_format(data);
		printf("%.2f\n", this->sb.cost);
	}

	void print_human_format(const Data& data){
		printf("===== RESULTS =====\n"
			   "-Initial Solution: %s\n\tVehicles: %li\n\tCost: %.2f\n\tFull cost: %.2f\n\tTime: %.2f\n"
			   "----------\n"
			   "-Final Solution: %s\n\tVehicles: %li\n\tCost: %.2f\n\tFull cost: %.2f\n\tTime: %.2f\n\tIterations: %li\n\tPool: %li\n\tSPP: %li\n"
			   "----------\n",
			   Validator::validate_solution(data,this->si) ? "valid" : "invalid",
			   this->si.size(),
			   this->si.cost-Def::VEHICLE_COST*this->si.size(),
			   this->si.cost,
			   this->initial_time,
			   Validator::validate_solution(data,this->sb) ? "valid" : "invalid",
			   this->sb.size(),
			   this->sb.cost-Def::VEHICLE_COST*this->sb.size(),
			   this->sb.cost,
			   this->total_time,
			   this->iter,
			   this->spp_pool_size,
			   this->spp_num_improve);
	}
	
};

#endif //__H_RESULT_INFO_H__
