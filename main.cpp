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

		// AMBIENTE E VARIÁVEIS
		IloEnv env;
        IloModel mod(env);
		
		IloArray<IloNumVarArray> x(env, data.n);
        IloArray<IloNumVarArray> z(env, data.n);
        IloArray<IloArray<IloNumVarArray>> f(env, data.n);
        IloArray<IloNumVarArray> a(env, data.n);
        IloArray<IloArray<IloNumVarArray>> b(env, data.n);
        vector<double> O(data.n, 0.0); 

		// CONSTRUÇÃO DO MODELO
        createVariables(env, data, mod, x, z, f, a, b, O);		// preenche as variáveis e cria a função objetivo
		createConstraints(env, mod, data, x, z, f, a, b, O);		

        // CONFIGURAÇÕES DO CPLEX
        IloCplex cplex(mod);
        cplex.setParam(IloCplex::EpGap, 0.0000001);             // Definindo uma tolerância
        cplex.setParam(IloCplex::TiLim, 21600);                 // Tempo limite de resolução
        cplex.setWarning(env.getNullStream());                  // Eliminar warnings
		cplex.setOut(env.getNullStream());					    // Elimina logs
      //cplex.setParam(IloCplex::Param::Benders::Strategy, 3);  // Ativar Benders do solver
            
        // Pré-fixando variáveis
        for(int i = 0; i < data.n; i++) {   
            for(int j = 0; j < data.n; j++) {
                for(int l = 0; l < data.n; l++) {
                    if (data.r[i][j] < data.c[l][j]) {
                        b[i][j][l].setBounds(0, 0);
                    }
                }
            }
        }

        // RESOLUÇÃO
        IloTimer timer(env);    
        double lb = 0;                                      
        double ub = 10e10;                                  
        double gap = 1;                                     
        cplex.use(Callback_mipinfo(env, lb, ub, gap));      

        timer.start();
        cplex.solve();
        timer.stop();
        
        if(cplex.getStatus() == IloAlgorithm::Optimal){
            lb = cplex.getObjValue();
            ub = cplex.getObjValue();
            gap = 0.0;
        }

        // RESULTADOS
        FILE *out = fopen("results.txt","w");
		saveResults(out, argv, data.n, data.alpha, data.w, data.g, data.r, data.s, timer, cplex, x, z, a, b, f);
        fclose(out);

        cout << "\nFunção objetivo: " << cplex.getObjValue () << endl ;
		cout << "Tamanho da rede (n)   : " << data.n << endl;
        cout << "Fator de desconto     : " << data.alpha << endl;
        cout << "Gap de Otimalidade    : " << (100 * gap) << " %" << endl;
		cout << "Tempo de CPU: " << (double)timer.getTime() << endl;

        printf("Hubs Instalados:");
        for(int k = 0; k < data.n; k++){
          if( cplex.getValue(x[k][k]) >= 0.1){
            printf(" %d\t ", k+1);
          }
        }
        cout << endl;

		env.end();
	}
    catch (IloException& ex) {
        cerr << "Error: " << ex << endl;
    }

    return 0;
}