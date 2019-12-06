#include "data.hpp"
#include <cmath>
#include <ctime>
#include <iostream>
#include <cfloat>
#include <boost/algorithm/string/classification.hpp> 
#include <boost/algorithm/string/split.hpp>

using namespace std;
using namespace boost;

int Data::read_from_stdin(const std::string &option){
    return read_stream(cin, option);
}

int Data::read_from_file(const char* filename, const std::string &option){
    ifstream file(filename);
    if(not file.is_open()) return READ_FAIL;

    return read_stream(file, option);
}

void Data::pre_process(Data& data){
	data.nref = 0;
	for(int i=1;i<data.nnod-1;i++){
        if(data.nodes[i].type != Def::PICKUP_NODE) continue; //only pickups
        int dev = data.nodes[i].pair;
		double oetw = data.nodes[i].etw;
		double oltw = data.nodes[i].ltw;
		double odetw = data.nodes[dev].etw;
		double odltw = data.nodes[dev].ltw;
		//ETW
        data.nodes[i].etw = std::max(data.nodes[i].etw, data.times[0][i]+data.nodes[0].etw);
        data.nodes[dev].etw = std::max(data.nodes[dev].etw, data.times[i][dev] + data.nodes[i].stw + data.nodes[i].etw);
		//LTW
        data.nodes[dev].ltw = std::min(data.nodes[dev].ltw, data.nodes[data.nnod-1].ltw - (data.times[dev][data.nnod-1] +data.nodes[dev].stw));
        data.nodes[i].ltw = std::min(data.nodes[i].ltw, data.nodes[dev].ltw-(data.times[i][dev]+data.nodes[i].stw));

		if(oetw != data.nodes[i].etw) data.nref++;
		if(oltw != data.nodes[i].ltw) data.nref++;
		if(odetw != data.nodes[dev].etw) data.nref++;
		if(odltw != data.nodes[dev].ltw) data.nref++;
    }	
}

void Data::compute_conflicts(Data& data){
	data.conflicts = vector< boost::dynamic_bitset<> >(data.nnod, boost::dynamic_bitset<>(data.nnod));;
	
	data.nconf = 0;
	for(int i=1;i<data.nnod-1;i++){
		if(data.nodes[i].demand <= 0) continue;
		double ei = data.nodes[i].etw;
		double li = data.nodes[i].ltw;
		double si = data.nodes[i].stw;
		int pi = data.nodes[i].pair;
		double epi = data.nodes[pi].etw;
		double lpi = data.nodes[pi].ltw;
		double spi = data.nodes[pi].stw;
		
		for(int j=i+1;j<data.nnod-1;j++){
			if(i==j || data.nodes[j].demand <= 0) continue;
			double ej = data.nodes[j].etw;
			double lj = data.nodes[j].ltw;
			double sj = data.nodes[j].stw;
			int pj = data.nodes[j].pair;
			double epj = data.nodes[pj].etw;
			double lpj = data.nodes[pj].ltw;
			double spj = data.nodes[pj].stw;
			
			double tij = data.times[i][j];
			double tji = data.times[j][i];
			double tipi = data.times[i][pi];
			double tjpj = data.times[j][pj];
			double tipj = data.times[i][pj];
			double tjpi = data.times[j][pi];
			double tpij = data.times[pi][j];
			double tpji = data.times[pj][i];
			double tpipj = data.times[pi][pj];
			double tpjpi = data.times[pj][pi];
			
            //Paths P1=(i,j,n+i,n+j) and P2=(i,j,n+j,n+i)
			bool vp1p2 = true;
            double T_i_j = ei + si + tij;
			double T_i_j_pi = std::max(T_i_j, ej) + sj + tjpi;
			double T_i_j_pj = std::max(T_i_j, ej) + sj + tjpj;
			double T_i_j_pi_pj = std::max(T_i_j_pi, epi) + spi + tpipj;
			double T_i_j_pj_pi = std::max(T_i_j_pj, epj) + spj + tpjpi;
			double T_i_j_pi_pj_dn = std::max(T_i_j_pi_pj, data.nodes[0].etw) + spj + data.times[pj][0];
			double T_i_j_pj_pi_dn = std::max(T_i_j_pj_pi, data.nodes[0].etw) + spi + data.times[pi][0];
            if(T_i_j > lj || (T_i_j_pi > lpi && T_i_j_pj > lpj) || (T_i_j_pi_pj > lpj && T_i_j_pj_pi > lpi) || (T_i_j_pi_pj_dn > data.nodes[0].ltw && T_i_j_pj_pi_dn > data.nodes[0].ltw) )
				vp1p2 = false;

			//Paths P3=(j,i,n+i,n+j) and P4=(j,i,n+j,n+i)
			bool vp3p4 = true;
            double T_j_i = ej + sj + tji;
			double T_j_i_pi = std::max(T_j_i, ei) + si + tipi;
			double T_j_i_pj = std::max(T_j_i, ei) + si + tipj;
			double T_j_i_pi_pj = std::max(T_j_i_pi, epi) + spi + tpipj;
			double T_j_i_pj_pi = std::max(T_j_i_pj, epj) + spj + tpjpi;
			double T_j_i_pi_pj_dn = std::max(T_j_i_pi_pj, data.nodes[0].etw) + spj + data.times[pj][0];
			double T_j_i_pj_pi_dn = std::max(T_j_i_pj_pi, data.nodes[0].etw) + spi + data.times[pi][0];
            if(T_j_i > li || (T_j_i_pi > lpi && T_j_i_pj > lpj) || (T_j_i_pi_pj > lpj && T_j_i_pj_pi > lpi) || (T_j_i_pi_pj_dn > data.nodes[0].ltw && T_j_i_pj_pi_dn > data.nodes[0].ltw))
				vp3p4 = false;

			//Path P5=(i,n+i,j,n+j)
			bool vp5 = true;
            double T_i_pi = ei + si + tipi;
			double T_i_pi_j = std::max(T_i_pi, epi) + spi + tpij;
			double T_i_pi_j_pj = std::max(T_i_pi_j, ej) + sj + tjpj;
			double T_i_pi_j_pj_dn = std::max(T_i_pi_j_pj, data.nodes[0].etw) + spj + data.times[pj][0];
            if(T_i_pi_j > lj || T_i_pi_j_pj > lpj || T_i_pi_j_pj_dn > data.nodes[0].ltw)
				vp5 = false;

			//Path P6=(j,n+j,i,n+i)
			bool vp6 = true;
            double T_j_pj = ej + sj + tjpj;
			double T_j_pj_i = std::max(T_j_pj, epj) + spj + tpji;
			double T_j_pj_i_pi = std::max(T_j_pj_i, ei) + si + tipi;
			double T_j_pj_i_pi_dn = std::max(T_j_pj_i_pi, data.nodes[0].etw) + spi + data.times[pi][0];
            if(T_j_pj_i > li || T_j_pj_i_pi > lpi || T_j_pj_i_pi_dn > data.nodes[0].ltw)
				vp6 = false;
			
            if(not vp1p2 && not vp3p4 && not vp5 && not vp6 && not data.conflicts[i][j]){
				//printf("Conflict %i x %i\n", i, j);
				data.conflicts[i][j] = true;
				data.conflicts[pi][j] = true;
				data.conflicts[j][pi] = true;
				data.conflicts[pi][pj] = true;
				data.conflicts[j][i] = true;
				data.conflicts[pj][i] = true;
				data.conflicts[i][pj] = true;
				data.conflicts[pj][pi] = true;
				data.nconf++;
            }            
		}
    }
}



void Data::read_li_lim_data(std::istream& inst){
    int vmax, vcap, vspeed; //num. vehicles, vehicles capacity and vehicle speed
    inst >> vmax >> vcap >> vspeed;
    
    this->vmax = vmax;
    this->vcap = vcap;
	this->vspeed = 1;
    this->nodes = vector<Node>();
	    
    int id, p, d;
    int x,y,q,etw,ltw,st; //position (x,y), demand, time window [e,l], service time, pickup and delivery points
    int nreq = 0;
    while(inst >> id >> x >> y >> q >> etw >> ltw >> st >> p >> d){
		int type = Def::DEPOT_NODE;
        if(d > 0){
            p=d;
			type = Def::PICKUP_NODE;
		}else if(p > 0)
			type = Def::DELIVERY_NODE;
		
		max_demand = max(q, max_demand);
        Node n = {id, (double)x, (double)y, q, (double)etw, (double)ltw, (double)st, p, type};
        this->nodes.push_back(n);
        nreq++; 
    }
    
    Node d0 = this->nodes[0];
    d0.id = nreq;
    this->nodes.push_back(d0);
    nreq++;
    this->nnod = nreq;
    this->nreq = (nreq-2)/2;
}

void Data::read_umovme_data(std::istream& inst){
    int vmax, vcap; //num. vehicles, vehicles capacity
	double vspeed; //vehicle speed
    inst >> vmax >> vcap >> vspeed;
    
    this->vmax = vmax;
    this->vcap = vcap;
	this->vspeed = vspeed;
    this->nodes = vector<Node>();
	    
    int id, p, d;
	double x, y, etw, ltw; //position (x,y), time window [e,l]
    int q,st; //demand, service time, pickup and delivery points
    int nreq = 0;
    while(inst >> id >> x >> y >> q >> etw >> ltw >> st >> p >> d){
		int type = Def::DEPOT_NODE;
        if(d > 0){
            p=d;
			type = Def::PICKUP_NODE;
		}else if(p > 0)
			type = Def::DELIVERY_NODE;
		
		max_demand = max(q, max_demand);
        Node n = {id, (double)x, (double)y, q, (double)etw, (double)ltw, (double)st, p, type};
        this->nodes.push_back(n);
        nreq++; 
    }
    
    Node d0 = this->nodes[0];
    d0.id = nreq;
    this->nodes.push_back(d0);
    nreq++;
    this->nnod = nreq;
    this->nreq = (nreq-2)/2;
}
//read keywords from the stream (separated by spaces)
std::string read_keyword(std::istream& f) {
	std::string keyword;
	if (f >> keyword)
		return keyword;
	else{
		throw "EOF";
	}
}
 

void Data::read_sb_data(std::istream& inst){
	char cline[1024];
	std::string dummy;
	std::string keyword;

	vcost = Def::VEHICLE_COST;
	
	do {
		keyword = read_keyword(inst);
		//printf("Read: %s\n", keyword.c_str());

		if(keyword[0] == '#'){
			//commentary just ignore it
			inst.getline(cline,1024);
			continue;
		}else if(keyword == "EOF"){
			//end of file
			break;
		}else if(keyword[keyword.length()-1] == ':') {
			keyword = keyword.substr(0,keyword.length()-1);
		}

		if(keyword == "NAME"){
			getline(inst, dummy);
		}else if(keyword == "LOCATION"){
			getline(inst, dummy);
		}else if(keyword == "COMMENT"){
			getline(inst, dummy);
		}else if(keyword == "TYPE"){
			inst >> dummy;
			if(dummy != "PDPTW"){
				printf("Wrong type %s\n", dummy.c_str());
				exit(-1);
			}
		}else if(keyword == "SIZE"){
			inst >> this->nnod;
			this->nreq = (this->nnod-1)/2;
		}else if(keyword == "DISTRIBUTION"){
			getline(inst, dummy);
		}else if(keyword == "ROUTE-TIME"){
			inst >> dummy;
		}else if(keyword == "TIME-WINDOW"){
			inst >> dummy;
		}else if(keyword == "CAPACITY"){
			inst >> this->vcap;
		}else if(keyword == "DEPOT"){
			inst >> dummy;
		}else if(keyword == "NODES"){
			for(int i=0;i<nnod;i++){
				int id, q, etw, ltw, stw, p, d;
				double lat, lon;
				inst >> id >> lat >> lon >> q >> etw >> ltw >> stw >> p >> d;
				int type = Def::DEPOT_NODE;
				if(d > 0){
					p=d;
					type = Def::PICKUP_NODE;
				}else if(p > 0)
					type = Def::DELIVERY_NODE;
		
				max_demand = max(q, max_demand);
				Node n = {id, lat, lon, q, (double)etw, (double)ltw, (double)stw, p, type};
				this->nodes.push_back(n);
			}
			Node d0 = this->nodes[0];
			d0.id = nreq;
			nodes.push_back(d0);
			nnod++;
		}else if(keyword == "EDGES"){
			times = vector< vector<double> >(nnod, vector<double>(nnod, 0.0));
			costs = vector< vector<double> >(nnod, vector<double>(nnod, 0.0));
			
			for(int i=0;i<nnod-1;i++){
				for(int j=0;j<nnod-1;j++){
					int t;
					inst >> t;
					times[i][j] = costs[i][j] = static_cast<double>(t);
					max_time = max(static_cast<double>(t), max_time);
					if(i == 0 && j!=nnod-1){
						costs[i][j] += vcost;
					}
					if(i==0){
						costs[nnod-1][j] = t;
						times[nnod-1][j] = t;
					}
					if(j==0){
						costs[i][nnod-1] = t;
						times[i][nnod-1] = t;
					}
				}
			}

			
			costs[0][nnod-1] = 0;
		}else{
			printf("Unknown keyword %s\n", keyword.c_str());
			exit(-1);
		}


	} while(true);
}

void Data::read_bcp_data(std::istream& inst){
    int n, vcap, dummy;
    inst >> dummy >> n >> dummy >> vcap >> dummy;
    
    this->vmax = n; //there is no vehicle restriction in this instances
    this->vcap = vcap;
	this->vspeed = 1;
    
    this->nodes = vector<Node>(); 
    int id;
    double x,y;
    int st, q, etw, ltw;
    for(int i=0;i<2*n+2;i++){
		inst >> id >> x >> y >> st >> q >> etw >> ltw;
		int pair = 0;
		int type = Def::DEPOT_NODE;
		if(q > 0){
			pair = n+i;
			type = Def::PICKUP_NODE;
		}else if(q < 0){
			pair = i-n;
			type = Def::DELIVERY_NODE;
		}
		max_demand = max(q, max_demand);
		Node nd = {id, x, y, q, (double)etw, (double)ltw, (double)st, pair, type};
		this->nodes.push_back(nd);
    } 
    
    this->nnod = 2*n+2;
    this->nreq = n;
}

double euclidean_distance(double x1, double y1, double x2, double y2){
	double ed = sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
	return ed;
}

int Data::read_stream(istream& inst, const std::string &option){
    if (option == Def::INST_LI_LIM_FORMAT) read_li_lim_data(inst);
	else if(option == Def::INST_UMOVME_FORMAT) read_umovme_data(inst);
    else if(option == Def::INST_BCP_FORMAT)  read_bcp_data(inst);
	else {
		read_sb_data(inst);
		return READ_OK;
	}
	
    /*Build the distance matrix*/
    int msize = this->nnod;
	
    //allocate objects
	times = vector< vector<double> >(msize, vector<double>(msize, 0.0));
	costs = vector< vector<double> >(msize, vector<double>(msize, 0.0));
	
    this->vcost = Def::VEHICLE_COST;
	
    //compute values
    for(int i=0;i<msize;i++){
		for(int j=i;j<msize;j++){
			double d = euclidean_distance(this->nodes[i].x, this->nodes[i].y, this->nodes[j].x, this->nodes[j].y);
			d = d/this->vspeed;
			this->costs[i][j] = d;
			this->costs[j][i] = d;
			this->times[i][j] = d;
			this->times[j][i] = d;

			max_time = max(d, max_time);
			
			if(i == 0 && j!=msize-1){
				this->costs[i][j] += vcost;
			}
		}
    }
	costs[0][nnod-1] = 0;

    return READ_OK;
}


Solution Data::read_solution(const Data& data, istream& sf){
	string dummy;
	getline(sf, dummy); // name
	getline(sf, dummy); // authors
	getline(sf, dummy); // date
	getline(sf, dummy); // reference
	getline(sf, dummy); // Solution
	Solution s(data.nnod);
	string line;
	while(not sf.eof()){
		getline(sf, line);
		vector<string> words;
		split(words, line, is_any_of(" "), token_compress_on);
		size_t i=3; //first node
		Route route(data.nnod);
		int node = data.initial_depot();
		int dem = 0;
		double time = 0;
		while(i < words.size()){
			int nd = atoi(words[i].c_str());
			if(nd == 0) break;
			route.insert_node(node, nd);
			route.cost += data.costs[node][nd];
			time += data.times[node][nd];
			route.reach_time[nd] = time;
			time = std::max(data.nodes[nd].etw, time) + data.nodes[nd].stw;
			dem += data.nodes[nd].demand;
			route.acc_cap[nd] = dem;
			node = nd;
			i++;
		}
		if(node != data.initial_depot()){
			route.cost += data.costs[node][data.final_depot()];
			s.insert_route(route);
			s.cost += route.cost;
			route.reach_time[data.final_depot()] = time;
		}
	}

	return s;
}

/**
 * Returns the name of the final solution file in the SINTEF format and the directory where to save it.
 **/
void get_solution_SINTEF_names(const std::string filename, std::string& iname, std::string& dirname){
	string inst_name = "";
	string dir_name = "";

	string temp = "";
	size_t idx=0;

	while(idx < filename.size() && filename[idx] != '.'){
		temp += filename[idx];
		if(filename[idx] == '/'){ // end of a folder part
			dir_name += temp;
			temp="";
		}
		idx++;
	}

	iname = temp;	
	dirname = dir_name;
}

int Data::write_solution(const Data& data, const Solution& s, const std::string filename){
	string iname="", dname="";
	get_solution_SINTEF_names(filename, iname, dname);
	char full_sol_name[1000];
	sprintf(full_sol_name, "%s.%li_%.2f.txt", iname.c_str(), s.size(), s.cost-(s.size()*Def::VEHICLE_COST));
	string fname = dname+string(full_sol_name);
	
	ofstream file(fname);
	if(not file.is_open()) return WRITE_FAIL;

	file << "Instance name\t: " << iname << endl;
	file << "Authors\t\t: " << "Carlo Sartori, Luciana Buriol" << endl;
	std::time_t t = std::time(0);   // get time now
    std::tm* now = std::localtime(&t);
	file << "Date\t\t: " << (now->tm_year + 1900) << '-' << (now->tm_mon + 1) << '-' <<  now->tm_mday << endl;
	file << "Reference\t: " << "reference" << endl;
	file << "Solution" << endl;

	for(size_t i=0;i<s.size();i++){
		file << "Route " << i+1 << " :";
		const Route& route = s.routes[i];
		int node = route.forward[0];
		while(node != data.final_depot()){
			file << " " << node;
			node = route.forward[node];
		}
		file << endl;
	}

	file.close();
	return WRITE_OK;
}
