#compiler to be used
CXX = g++

#compilation flags to be used (apparently we need to ignore CPLEX's attributes)
CXXFLAGS = -Wall -O3 -std=c++11 -DIL_STD -Wno-ignored-attributes

#location of files and libraries to be used
COMDIR = common/
CONDIR = constructor/
ALGDIR = algo/
GESDIR = ages/
LNSDIR = lns/
CPXDIR = cplex/
VALDIR = validator/

#include dir of cplex: default for home		
ICPXDIR = -I/opt/ibm/ILOG/CPLEX_Studio1262/cplex/include/ -I/opt/ibm/ILOG/CPLEX_Studio1262/concert/include/ 
#Lib dir of cplex: default for home
LCPXDIR = -L/opt/ibm/ILOG/CPLEX_Studio1262/cplex/lib/x86-64_linux/static_pic -L/opt/ibm/ILOG/CPLEX_Studio1262/concert/lib/x86-64_linux/static_pic 

## other includes and libs
INCLUDES = -I/usr/include/ -Iinclude/ $(ICPXDIR)
LIBS = -lm -lboost_program_options -lilocplex -lconcert -lcplex -pthread
LIBDIR = -L/usr/lib $(LCPXDIR)

# define the C++ source files
SMAIN = main.cpp $(COMDIR)data.cpp $(VALDIR)validator.cpp $(CONDIR)constructor.cpp $(COMDIR)operation.cpp $(COMDIR)perturb.cpp $(GESDIR)ages.cpp $(LNSDIR)lns.cpp $(CPXDIR)sppm.cpp $(ALGDIR)algorithm.cpp
OMAIN = $(SMAIN:.cpp=.o)

# define the executable file 
MAIN = main

.PHONY: depend clean

all: $(MAIN)
	@echo  main compiled

$(MAIN): $(OMAIN)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(LIBDIR) -o $(MAIN) $(OMAIN) $(LIBS)

.cpp.o: 
	$(CXX) $(CXXFLAGS) $(DDFS) $(INCLUDES) -c $<  -o $@

clean:
	$(RM) $(COMDIR)*.o $(CONDIR)*.o $(GESDIR)*.o $(LNSDIR)*.o $(CPXDIR)*.o $(ALGDIR)*.o $(VALDIR)*.o *.o
	$(RM) *.bak 

depend: $(SMAIN)
	makedepend $(INCLUDES) $^

# DO NOT DELETE THIS LINE -- make depend needs it
