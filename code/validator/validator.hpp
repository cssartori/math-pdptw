#ifndef __H_VALIDATOR_H__
#define __H_VALIDATOR_H__

#include "../common/data.hpp"
#include "../common/solution.hpp"

namespace Validator{
	/*Validate the solution according to the PDPTW constraints. Return true when valid*/
	bool validate_solution(const Data& data, const Solution& s);
};

#endif//__H_VALIDATOR_H__
