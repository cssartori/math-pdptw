#ifndef __H_PAIR_MOVEMENT_TUPLE_H__
#define __H_PAIR_MOVEMENT_TUPLE_H__

#include "defs.hpp"
#include <cmath>

struct PairMovementTuple{
public:
	int back_u; /*Backward of node u*/
	int u;		/*Node u*/
	int for_u;	/*Forward of node u*/

	int back_v;	/*Backward of node v*/
	int v;		/*Node v*/
	int for_v;	/*Forward of node v*/

	size_t r1;		/*route 1 involved in the movement*/
	size_t r2;		/*route 2 involved in the movement*/
	
	bool lessr; /*true if the movement leads to fewer routes*/
	//int mr;		/*magnitude of the routes (square of the size of the routes)*/
	
	double cost;/*cost of the movement*/

	PairMovementTuple(){
		this->back_u = Def::NO_NODE;
		this->u = Def::NO_NODE;
		this->for_u = Def::NO_NODE;
		this->back_v = Def::NO_NODE;
		this->v = Def::NO_NODE;
		this->for_v = Def::NO_NODE;

		this->r1 = Def::NO_ROUTE;
		this->r2 = Def::NO_ROUTE;

		this->lessr = false;
		//this->mr = 0;

		this->cost = Def::INF;
	}

	PairMovementTuple(int bu, int u, int fu, int bv, int v, int fv, size_t r1, size_t r2, bool lessr, double cost){
		this->back_u = bu;
		this->u = u;
		this->for_u = fu;
		this->back_v = bv;
		this->v = v;
		this->for_v = fv;

		this->r1 = r1;
		this->r2 = r2;

		this->lessr = lessr;
		//this->mr = mr;

		this->cost = cost;
	}

	friend bool operator<(const PairMovementTuple& tp1, const PairMovementTuple& tp2){
		bool ret = false;
		if(tp1.lessr){
			if(not tp2.lessr) ret = true;
			else if(tp1.cost < tp2.cost) ret = true;
		}else if(tp2.lessr)	ret = false;
		else if(tp1.cost < tp2.cost) ret = true;	
		
		return ret;
	}

	friend bool operator>(const PairMovementTuple& tp1, const PairMovementTuple& tp2){
		return (tp2 < tp1);
	}

	friend bool operator<=(const PairMovementTuple& tp1, const PairMovementTuple& tp2){
		return not(tp1 > tp2);
	}

	friend bool operator>=(const PairMovementTuple& tp1, const PairMovementTuple& tp2){
		return not(tp1 < tp2);
	}

	friend bool operator==(const PairMovementTuple& tp1, const PairMovementTuple& tp2){
		return ((tp1 >= tp2) && (tp1 <= tp2));
	}

	friend bool operator!=(const PairMovementTuple& tp1, const PairMovementTuple& tp2){
		return not(tp1 == tp2);
	}
	
	
	
	
};




#endif //__H_PAIR_MOVEMENT_TUPLE_H__
