#include "model.hpp"

ILOMIPINFOCALLBACK3(Callback_mipinfo, double&, lb, double&, ub, double&, gap) {
    ub = getBestObjValue(); 			
    if(lb < getIncumbentObjValue()){
      lb = getIncumbentObjValue(); 	
    } 
    gap = getMIPRelativeGap();
}

int main (int argc, char *argv[]) {
    try {
        Data data;

        // LEITURA DE DADOS 
        ifstream file(argv[1]);
        if (!file.is_open()){
            cout << "Error openning file: " << argv[1] << endl;
            file.close();
            exit(EXIT_FAILURE);
        } else {
            readData(data, file, argc, argv);
            file.close();
        }

        // AMBIENTE E VARIÁVEIS DA TAHERKHANI
        IloEnv env;
        IloModel mod(env);
        
        IloArray<IloNumVarArray> x(env, data.n);
        IloArray<IloArray<IloArray<IloNumVarArray>>> y(env, data.n);
        IloArray<IloNumVarArray> z(env, data.n);
        IloArray<IloArray<IloNumVarArray>> f(env, data.n);
        vector<double> O(data.n, 0.0); 

        // CONSTRUÇÃO DO MODELO
        createVariablesTaherkhani(env, data, mod, x, y, z, f, O);
        createConstraintsTaherkhani(env, mod, data, x, y, z, f, O);

        // CONFIGURAÇÕES DO CPLEX
        IloCplex cplex(mod);
        cplex.setParam(IloCplex::EpGap, 0.0000001);             
        cplex.setParam(IloCplex::TiLim, 21600);                 
        cplex.setWarning(env.getNullStream());                  
        cplex.setOut(env.getNullStream());					    
            
        // RESOLUÇÃO
        IloTimer timer(env);    
        double lb = 0;                                      
        double ub = 10e10;                                  
        double gap = 1;                                     
        cplex.use(Callback_mipinfo(env, lb, ub, gap));      

        //Pré-fixando variáveis
		for(int i = 0; i< data.n; i++) {   
		    for(int j = 0; j < data.n; j++){
		        for(int k = 0; k < data.n; k++){
		            for(int l = 0; l < data.n; l++) {
                        if (data.r[i][j] < (data.c[i][k] + data.c[l][j])) {
                            y[i][j][k][l].setBounds(0, 0);
                        }
                    }
		        }
		    }
		}

        timer.start();
        cplex.solve();
        timer.stop();
        
        if(cplex.getStatus() == IloAlgorithm::Optimal){
            lb = cplex.getObjValue();
            ub = cplex.getObjValue();
            gap = 0.0;
        }

        // --- CÁLCULO DE DEMANDA
        double totalDemand = 0;
        double satisfiedDemand = 0;
        
        for(int i = 0; i < data.n; i++) {
            for(int j = 0; j < data.n; j++) {
                // Soma a demanda total disponível
                totalDemand += data.w[i][j];
                
                // Varre as combinações de hubs para ver onde a demanda passou
                for(int k = 0; k < data.n; k++) {
                    for(int m = 0; m < data.n; m++) {
                        double y_val = cplex.getValue(y[i][j][k][m]);
                        
                        // Usamos uma pequena tolerância para evitar erros de ponto flutuante
                        if(y_val > 0.00001) {
                            satisfiedDemand += data.w[i][j] * y_val;
                        }
                    }
                }
            }
        }
        
        // RESULTADOS 
        cout << "\n--- RESULTADOS TAHERKHANI (2019) ---" << endl;
        cout << "Função objetivo: " << cplex.getObjValue() << endl;
        cout << "Tamanho da rede (n)   : " << data.n << endl;
        cout << "Fator de desconto     : " << data.alpha << endl;
        cout << "Gap de Otimalidade    : " << (100 * gap) << " %" << endl;
        cout << "Tempo de CPU: " << (double)timer.getTime() << " segundos" << endl;
        cout << "Demanda Total da Rede : " << totalDemand << endl;
        cout << "Demanda Satisfeita    : " << satisfiedDemand << " (" 
             << (satisfiedDemand / totalDemand) * 100.0 << "%)" << endl;
             
        printf("Hubs Instalados:");
        for(int k = 0; k < data.n; k++){
          if( cplex.getValue(x[k][k]) >= 0.1){
            printf(" %d\t ", k+1);
          }
        }
        cout << "\n------------------------------------\n" << endl;

        env.end();
    }
    catch (IloException& ex) {
        cerr << "Error: " << ex << endl;
    }

    return 0;
}