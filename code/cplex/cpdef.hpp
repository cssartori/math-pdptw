#ifndef __H_CPLEX_DEFINITIONS_H__
#define __H_CPLEX_DEFINITIONS_H__

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <ilcplex/ilocplex.h>
#pragma GCC diagnostic pop

#include <vector>

//looks like this is just a "glorified" 'using namespace std;'
//ILOSTLBEGIN

typedef IloArray<IloNumArray> IloNumMatrix2;
typedef IloArray<IloNumVarArray> IloNumVarMatrix2;
typedef std::vector< std::vector<double> > RealMat2;

#endif //__H_CPLEX_DEFINITIONS_H__
