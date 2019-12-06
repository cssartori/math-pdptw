#ifndef __H_ALG_H__
#define __H_ALG_H__

/*****************************************
 * Class holding the main algorithm
 * which combines all components together
 * to form athe hybrid algorithm using an
 * ILS framework
 *****************************************/

#include "../common/data.hpp"
#include "../common/parameters.hpp"
#include "../common/resultinfo.hpp"
#include "../common/solution.hpp"
#include <boost/dynamic_bitset.hpp>
#include <map>

class Algorithm{
private:
	const Data& data;
	const Parameters& par;

	bool extract_routes(const Solution& s, std::map< boost::dynamic_bitset<>, Route >& route_pool);
	
public:
	Algorithm(const Data& data_, const Parameters& par_) : data(data_), par(par_) {};
	ResultInfo solve();	
};

#endif //__H_ALG_H__
