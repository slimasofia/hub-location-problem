#pragma once 
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <ilcplex/ilocplex.h>

using namespace std;
ILOSTLBEGIN

struct Data {
	int n;						// Tamanho da rede (quantidades de nós)
	float alpha;				// Fator de desconto
	vector<vector<double>> w;	// Quantidade de demanda a ser enviada entre os nós i e j
	vector<vector<double>> r;	// Receita obtida por enviar uma unidade de demanda entre os nós i e j

	// custos
	vector<vector<double>> g;	// Custos por enviar uma unidade de demanda entre os nós i e j
	vector<vector<double>> c; 	// Custos fixos de instalação de um hub
	vector<double> s;			// Custos de operação nos links inter-hubs
};

void readData(Data& data, ifstream& file, int argc, char *argv[]);

void createVariables(IloEnv& env, Data& data, IloModel& mod,
					IloArray<IloNumVarArray>& x,
					IloArray<IloNumVarArray>& z,
					IloArray<IloArray<IloNumVarArray>>& f,
					IloArray<IloNumVarArray>& a,
					IloArray<IloArray<IloNumVarArray>>& b,
					vector<double>& O);

void createConstraints(IloEnv& env, IloModel& mod, Data& data,
                     IloArray<IloNumVarArray>& x,
                     IloArray<IloNumVarArray>& z,
                     IloArray<IloArray<IloNumVarArray>>& f,
                     IloArray<IloNumVarArray>& a,
                     IloArray<IloArray<IloNumVarArray>>& b,
                     vector<double>& O);


// --- Formulação Taherkhani (2019) ---
void createVariablesTaherkhani(IloEnv& env, Data& data, IloModel& mod,
                               IloArray<IloNumVarArray>& x,
                               IloArray<IloArray<IloArray<IloNumVarArray>>>& y,
                               IloArray<IloNumVarArray>& z,
                               IloArray<IloArray<IloNumVarArray>>& f,
                               vector<double>& O);

void createConstraintsTaherkhani(IloEnv& env, IloModel& mod, Data& data,
                                 IloArray<IloNumVarArray>& x,
                                 IloArray<IloArray<IloArray<IloNumVarArray>>>& y,
                                 IloArray<IloNumVarArray>& z,
                                 IloArray<IloArray<IloNumVarArray>>& f,
                                 vector<double>& O);