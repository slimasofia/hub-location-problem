CFLAGS= -Wall -m64 -g -w
CXX=g++

ILOG= /opt/ibm/ILOG/CPLEX_Studio2212
CPPFLAGS= -DIL_STD -I$(ILOG)/cplex/include -I$(ILOG)/concert/include
CPLEXLIB=-L$(ILOG)/cplex/lib/x86-64_linux/static_pic -lilocplex -lcplex -L$(ILOG)/concert/lib/x86-64_linux/static_pic -lconcert -lm -lpthread -ldl

all: build-mine build-taherkhani

build-mine:  
	$(CXX) $(CFLAGS) $(CPPFLAGS) -o out main.cpp model.cpp $(CPLEXLIB) 

build-taherkhani:
	$(CXX) $(CFLAGS) $(CPPFLAGS) -o taherkhani_out main-taherkhani.cpp model.cpp $(CPLEXLIB)

clean:
	rm -f out taherkhani_out my_results.txt taherkhani_results.txt *.o *.log