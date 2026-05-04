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

        // CÁLCULO DA DEMANDA SATISFEITA POR NÚMERO DE PARES 0-D
        int satisfiedPairs = 0;
        int totalPairs = data.n * (data.n - 1); 

        for(int i = 0; i < data.n; i++){
            for(int j = 0; j < data.n; j++){
                if(i == j) continue; 
                
                bool isSatisfied = false;
                for(int k = 0; k < data.n; k++){
                    for(int m = 0; m < data.n; m++){
                        if(cplex.getValue(y[i][j][k][m]) >= 0.1){
                            isSatisfied = true;
                            break; 
                        }
                    }
                    if(isSatisfied) break; 
                }
                
                if(isSatisfied) {
                    satisfiedPairs++;
                }
            }
        }
        double percentDemand = ((double)satisfiedPairs / totalPairs) * 100.0;
        
        // RESULTADOS 
        FILE *re;
        re = fopen("taherkhani_results.txt","aw+"); 

        vector<int> hubs;
        for(int j = 0; j < data.n; j++){
            if(cplex.getValue(x[j][j]) >= 0.1){
                hubs.push_back(j + 1); 
            }
        }
        
        fprintf(re, "Informações Gerais: %s | %d | %.1f | %.0f | %.0f | %.0f \n", argv[1], data.n, data.alpha, data.r[0][0], data.s[0], data.g[0][0]);
        fprintf(re, "Valor função objetivo: %f\t \n", (double) cplex.getObjValue());
        fprintf(re, "Demanda Satisfeita: %d pares (%.2f%%)\t \n", satisfiedPairs, percentDemand);        
        fprintf(re, "Tempo de CPU: %f\t \n", (double) timer.getTime());
        fprintf(re, "Hubs: " );
        for(int hub : hubs) {
            fprintf(re, "%d\t ", hub);
        }
        fprintf(re, "\n======================================================================\n");
        fclose(re); 

        cout << "\n--- RESULTADOS TAHERKHANI (2019) ---" << endl;
        cout << "\nFunção objetivo     : " << cplex.getObjValue () << endl ;
        cout << "Tamanho da rede (n) : " << data.n << endl;
        cout << "Fator de desconto   : " << data.alpha << endl;
        cout << "Gap de Otimalidade  : " << (100 * gap) << " %" << endl;
        cout << "Tempo de CPU        : " << (double)timer.getTime() << " segundos" << endl;
        cout << "Demanda Satisfeita  : " << satisfiedPairs << " pares (" << fixed << setprecision(2) << percentDemand << "%)" << endl;        
        cout << "Hubs Instalados     : ";
        for(int hub : hubs) {
            cout << hub << " ";
        }
        cout << endl;

        env.end();
    }
    catch (IloException& ex) {
        cerr << "Error: " << ex << endl;
    }

    return 0;
}