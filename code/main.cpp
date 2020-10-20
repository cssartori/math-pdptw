#include "common/defs.hpp"
#include "common/data.hpp"
#include "common/resultinfo.hpp"
#include "common/parameters.hpp"
#include "lns/lns.hpp"
#include <boost/program_options.hpp>
#include <iostream>
#include <cstdio>
#include <iomanip> // setprecision
#include <sstream> // stringstream
#include "algo/algorithm.hpp"

using namespace std;

Parameters read_parameters(int argc, char** argv);

int main(int argc, char** argv){
	Parameters par = read_parameters(argc, argv);
	
	Data data;
        if (par.inst_path == "") {
           printf("Reading instance from stdin.\n");
           if (data.read_from_stdin(par.inst_format) == Data::READ_FAIL) {
                       printf("Failed reading from stdin.\n");
                       return EXIT_FAILURE;
           }
        } else {
           printf("Reading instance from file '%s'.\n", par.inst_path.c_str());
           if (data.read_from_file(par.inst_path.c_str(), par.inst_format) == Data::READ_FAIL) {
                       printf("Failed reading from the file '%s'.\n", par.inst_path.c_str());
                       return EXIT_FAILURE;
           }
        }
   cout.precision(16);
			
	printd(("Pre-processing.\n"));
	// Data pre-processing
	Data::pre_process(data);
	Data::compute_conflicts(data);
	printd(("Pre-processing: %i tightenings | %i conflicts\n", data.nref, data.nconf));
	
	ResultInfo rinfo;
	Algorithm alg(data, par);
	rinfo = alg.solve();
	
	if(par.print_type == Def::PRINT_TABLE_FORMAT){
		rinfo.print_table_format(data);
	}else if(par.print_type == Def::PRINT_IRACE_FORMAT){
		rinfo.print_irace_format(data);
	}else{
		rinfo.print_human_format(data);
	}
		
	if(par.write_solution){
		if(Data::write_solution(data, rinfo.sb, par.solution_file) == Data::WRITE_FAIL)
			printf("Failed writing solution to file %s\n", par.solution_file.c_str());
	}
	
	return 0;
}

// convert double value to string according to precision
string dtos(double value, unsigned int precision=2){
	stringstream stream;
	stream << fixed << setprecision(precision) << value;
	return stream.str();
}

Parameters read_parameters(int argc, char** argv){
	Parameters par;
	std::string str_alg_ages = Def::AGES_NEW;
	
	namespace po = boost::program_options;
	po::options_description desc("parameters");
	//TODO: update this descriptions
	desc.add_options()
		("help,h", "print this help info")
                ("instance,i", po::value<string>(&par.inst_path), "Path to the instance file. If not set, read from stdin.")
		("format,f", po::value<string>(&par.inst_format),
		 ("the instance format: " + Def::INST_LI_LIM_FORMAT +
		  " (Li & Lim) , " + Def::INST_BCP_FORMAT + " (Rokpe BCP), " + Def::INST_UMOVME_FORMAT + " (uMov.me) and " + Def::INST_SB_FORMAT + " (Sartori & Buriol). Default: "+par.inst_format).c_str())
		("seed,s", po::value<mt19937_64::result_type>(&par.seed),
		 ("define the RNG seed. Default: " + to_string(par.seed)).c_str())
		("time,t", po::value<double>(&par.max_time),
		 ("maximum running time in seconds. Default: " + dtos(par.max_time) ).c_str())
		// ("sol-constructor", po::value<string>(&par.initial_constructor),
		//  ("constructor used to generate the initial solution: trivial, greedy. Default: " + par.initial_constructor).c_str())
		("alg-use-spp", po::value<bool>(&par.use_spp),
		 ("Wheter to use SP model phase during the search: true, false. Default: " + (par.use_spp ? string("true") : string("false"))).c_str())
		("alg-pert-prob", po::value<double>(&par.prob_eval),
		 ("Probability for the pertubation. For \"original\" is the prob. of a 'move shift'; for \"new\" is the bias mu. Default: " + dtos(par.prob_eval) ).c_str())
		("alg-ages", po::value<string>(&str_alg_ages),
		 ("Print type format: " + Def::AGES_ORIGINAL + " , " + Def::AGES_NEW + " . Default: " + par.print_type).c_str())
		("alg-perc-pert-size", po::value<double>(&par.ppsize),
		 ("Percentage P of requests to be perturbed in the main heuristic: P*R, R is the num. of requests. Default: " + dtos(par.ppsize)).c_str())
		("ages-max-iter", po::value<size_t>(&par.ges_max_iter),
		 ("Maximum number of iterations without improvement for AGES heuristic. Default: " + to_string(par.ges_max_iter)).c_str())
		("ages-pert-size", po::value<size_t>(&par.ges_psize),
		 ("Number of requests to be moved in a perturbation of the AGES. Used in combination to \"original\" AGES. Default: " + dtos(par.ges_ppsize)).c_str())
		("ages-perc-pert-size", po::value<double>(&par.ges_ppsize),
		 ("Percentage P of requests to be perturbed in AGES heuristic: P*R, R num. requests. Used in combination to the \"new\" AGES. Default: " + dtos(par.ges_ppsize)).c_str())
		("lns-min-q", po::value<int>(&par.lns_min_q),
		 ("Minimum number of requests to remove in LNS. Default: "+ to_string(par.lns_min_q)).c_str())
		("lns-max-perc-q", po::value<double>(&par.lns_max_mult),
		 ("Maximum percentage of requests to be removed in LNS. Default: "+dtos(par.lns_max_mult) ).c_str())
		("lns-min-k", po::value<int>(&par.lns_min_k),
		 ("Minimum number of routes for regret heuristic in LNS. Default: "+ to_string(par.lns_min_k)).c_str())
		("lns-max-k", po::value<int>(&par.lns_max_k),
		 ("Maximum number of routes for regret heuristic in LNS. Default: "+ to_string(par.lns_max_k)).c_str())
		("lns-max-iter", po::value<int>(&par.lns_max_iter),
		 ("Maximum number of iterations without improvement of LNS. Default: "+ to_string(par.lns_max_iter)).c_str())
		("lns-lsize", po::value<int>(&par.lns_lsize),
		 ("Size of the LAHC list in LNS. Default: "+ to_string(par.lns_lsize)).c_str())
		("lns-rollback", po::value<bool>(&par.lns_rollback),
		 ("Rollback solution to best known in LAHC acceptance. Default: "+(par.lns_rollback ? string("true") : string("false"))).c_str())
		("lns-weight-shaw", po::value<int>(&par.lns_wrem[LNS::REM_SHAW_TYPE]),
		 ("Weight of shaw removal in LNS selection. Default: "+ to_string(par.lns_wrem[LNS::REM_SHAW_TYPE])).c_str())
		("lns-weight-random", po::value<int>(&par.lns_wrem[LNS::REM_RANDOM_TYPE]),
		 ("Weight of random removal in LNS selection. Default: "+ to_string(par.lns_wrem[LNS::REM_RANDOM_TYPE])).c_str())
		("print", po::value<string>(&par.print_type),
		 ("Print type format: " + Def::PRINT_HUMAN_FORMAT + " , " + Def::PRINT_TABLE_FORMAT + " . Default: " + par.print_type).c_str())
		("log-file", po::value<string>(&par.log_filename),
		 ("Logfile to be used."))
		("solution", po::value<string>(&par.solution_file),
		 ("write final solution to the file given as parameter."))
		; 

	
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm); // seems to be what put the read values in the variables

	if (vm.count("help")) {
		cout << desc << endl;
		exit(EXIT_SUCCESS);
	}

	if(str_alg_ages == Def::AGES_ORIGINAL){
		par.alg_ages = Def::AGES_ORIGINAL_TYPE;
		if(vm.count("ages-perc-pert-size")){
			printf("**Warning: you are using parameter 'ages-perc-pert-size' with AGES 'original', but it only works for AGES 'new'.");
		}
	}else if(vm.count("ages-pert-size")){
		printf("**Warning: you are using parameter 'ages-pert-size' with AGES 'new', but it only works for AGES 'original'.");
	}
	
	// ASSERTS section
	// TODO: update this according to the new parameter values
	assert(par.inst_format == Def::INST_LI_LIM_FORMAT || par.inst_format == Def::INST_BCP_FORMAT || par.inst_format == Def::INST_UMOVME_FORMAT || par.inst_format == Def::INST_SB_FORMAT);
	assert(par.seed >= 0);
	assert(par.max_time >= 0);
	assert(par.initial_constructor == Def::CONSTRUCTOR_GREEDY || par.initial_constructor == Def::CONSTRUCTOR_TRIVIAL);
	assert(par.ppsize >= 0);
	assert(par.prob_eval >= 0);
	assert(par.ges_max_iter > 0);
	assert(par.ges_ppsize >= 0);
	assert(par.lns_min_q > 0);
	assert(par.lns_max_mult >= 0.1);
	assert(par.lns_min_k > 0);
	assert(par.lns_max_k > 0 && par.lns_max_k >= par.lns_min_k);
	assert(par.lns_max_iter > 0);
	assert(par.lns_lsize >= 0);
	assert(par.lns_wrem[LNS::REM_SHAW_TYPE] >= 0 && par.lns_wrem[LNS::REM_RANDOM_TYPE] >= 0 && (par.lns_wrem[LNS::REM_SHAW_TYPE] + par.lns_wrem[LNS::REM_RANDOM_TYPE] <= 10));
	assert(par.print_type == Def::PRINT_HUMAN_FORMAT || par.print_type == Def::PRINT_TABLE_FORMAT || par.print_type == Def::PRINT_IRACE_FORMAT);

	par.lns_wrem[LNS::REM_WORST_TYPE] = 10 - par.lns_wrem[LNS::REM_SHAW_TYPE] - par.lns_wrem[LNS::REM_RANDOM_TYPE];
	
	if(par.solution_file.size() > 0) par.write_solution = true;
	if(par.log_filename.size() > 0)	par.save_log = true;

	return par;
}
