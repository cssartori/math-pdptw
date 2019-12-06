#ifndef __H_TIMER_H__
#define __H_TIMER_H__

#include <chrono>

struct Timer{
	std::chrono::time_point<std::chrono::system_clock> start_, end_;
	bool stopped = false;
	
	void start(){
		this->stopped = false;
		this->start_ = std::chrono::system_clock::now();
	}

	void stop(){
		this->end_ = std::chrono::system_clock::now();
		this->stopped = true;
	}

	double elapsed_seconds(){
		if (not stopped) end_ = std::chrono::system_clock::now();
	  std::chrono::duration<double> elsec = end_ - start_;
		return elsec.count();
	}
};

#endif //__H_TIMER_H__
