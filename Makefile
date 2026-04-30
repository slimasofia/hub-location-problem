CFLAGS= -Wall -m64 -g -w
CXX=g++

ILOG= /opt/ibm/ILOG/CPLEX_Studio2212
CPPFLAGS= -DIL_STD -I$(ILOG)/cplex/include -I$(ILOG)/concert/include
CPLEXLIB=-L$(ILOG)/cplex/lib/x86-64_linux/static_pic -lilocplex -lcplex -L$(ILOG)/concert/lib/x86-64_linux/static_pic -lconcert -lm -lpthread -ldl

build:  
	$(CXX) $(CFLAGS) $(CPPFLAGS) -o out main.cpp model.cpp $(CPLEXLIB) 

clean:
	rm -f out results.txt *.o *.log