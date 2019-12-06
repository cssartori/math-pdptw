#ifndef __H_DATA_H__
#define __H_DATA_H__

#include "defs.hpp"
#include "solution.hpp"
#include "node.hpp"
#include <fstream>
#include <istream>
#include <vector>
#include <cassert>
#include <boost/dynamic_bitset.hpp>

class Data{
public:
    int nreq;
    int vmax;
    int vcap;
    int vcost;
	double vspeed;
    int nnod;
    int minv = 1;
	int nref = 0;
	int nconf = 0;
	int max_demand = 0;
	double max_time = 0;
    
    std::vector<Node> nodes;
	std::vector< std::vector<double> > times, costs;
	std::vector< boost::dynamic_bitset<> > conflicts;

    const static int READ_OK = 1;
    const static int READ_FAIL = 0;
	const static int WRITE_OK = 2;
	const static int WRITE_FAIL = 0;
	
    int read_from_stdin(const std::string &option);
    int read_from_file(const char* filename, const std::string &option);
	
	int initial_depot() const{ return 0; }
	int final_depot() const{ return nnod-1; }

	static void pre_process(Data& data);
	static void compute_conflicts(Data& data);
	static Solution read_solution(const Data& data, std::istream& sf);
	static int write_solution(const Data& data, const Solution& s, const std::string filename);
	
private:
    int read_stream(std::istream& inst, const std::string &option = Def::INST_LI_LIM_FORMAT);
    void read_li_lim_data(std::istream& inst);
    void read_bcp_data(std::istream& inst);
	void read_umovme_data(std::istream& inst);
	void read_sb_data(std::istream& inst);
	
};

#endif //__H_DATA_H__
