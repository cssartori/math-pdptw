#ifndef __H_DEFINITIONS_H__
#define __H_DEFINITIONS_H__

#include <string>
#include <limits>
#include <random>

//Comment this line to stop printing debugs
#define DEBUG
#ifdef DEBUG
#define printd(str) printf str
#else
#define printd(str)
#endif

namespace Def{

	const std::string INST_LI_LIM_FORMAT = "ll";
	const std::string INST_BCP_FORMAT = "bc";
	const std::string INST_UMOVME_FORMAT = "um";
	const std::string INST_SB_FORMAT = "sb";
	const std::string INST_JSON_FORMAT = "js";

	const std::string PRINT_TABLE_FORMAT = "table";
	const std::string PRINT_HUMAN_FORMAT = "human";
	const std::string PRINT_IRACE_FORMAT = "irace";
	
	const std::string CONSTRUCTOR_GREEDY = "greedy";
	const std::string CONSTRUCTOR_TRIVIAL = "trivial";

	const double VEHICLE_COST = 1e5;
	const int NO_NODE = -1;
	const size_t NO_ROUTE = std::numeric_limits<size_t>::max();;

	const int DELIVERY_NODE = 1;
	const int PICKUP_NODE = 2;
	const int DEPOT_NODE = 3;

	const double INF = std::numeric_limits<double>::infinity();
	const double EPSILON = 1e-5;
		
};

#endif //__H_DEFINITIONS_H__
