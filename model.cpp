#include "model.hpp"

void saveResults(FILE *out, char *argv[], int n, float alpha,
                      vector<vector<double>>& w, vector<vector<double>>& g,
                      vector<vector<double>>& r, vector<double>& s,
                      IloTimer& timer, IloCplex& cplex,
                      IloArray<IloNumVarArray>& x, IloArray<IloNumVarArray>& z,
                      IloArray<IloNumVarArray>& a,IloArray<IloArray<IloNumVarArray>>& b,
                      IloArray<IloArray<IloNumVarArray>>& f) {

    // CÁLCULOS  
    float count = 0;
    float demand = 0;
    
    for(int i = 0; i < n; i++) {
        for(int j = 0; j < n; j++) {
            for(int k = 0; k < n; k++) {
                if(cplex.getValue(b[i][j][k]) > 0.001) {
                    count += 1;
                    demand += w[i][j];
                }
            }
        }
    }

    // CABEÇALHO E INFORMAÇÕES GERAIS
    fprintf(out, "[1] INFORMAÇÕES DA INSTÂNCIA E DESEMPENHO\n");
    fprintf(out, "--------------------------------------------------------\n");
    fprintf(out, "Arquivo              : %s\n", argv[1]);
    fprintf(out, "Tamanho da rede (n)  : %d\n", n);
    fprintf(out, "Fator Desconto Alpha : %.2f\n", alpha);
    fprintf(out, "Tempo de Execução    : %.4f segundos\n", (double) timer.getTime());
    fprintf(out, "Função Objetivo      : %.4f\n", (double) cplex.getObjValue());
    fprintf(out, "Demanda Total        : %.2f\n", demand);
    
    // DECISÕES ESTRATÉGICAS 
    fprintf(out, "\n[2] DECISÕES ESTRATÉGICAS\n");
    fprintf(out, "--------------------------------------------------------\n");
    
    fprintf(out, "Hubs Instalados: ");
    for(int k = 0; k < n; k++) {
        if(cplex.getValue(x[k][k]) >= 0.1) {
            fprintf(out, "[%d] ", k + 1);
        }
    }
    fprintf(out, "\n\nLinks Inter-Hubs Ativos (Variável z):\n");
    for(int k = 0; k < n; k++) {
        for(int m = 0; m < n; m++) {
            if(k != m && cplex.getValue(z[k][m]) > 0.001) {
                fprintf(out, "  -> Hub %d conectado com Hub %d\n", k + 1, m + 1);
            }
        }
    }

    // ALOCAÇÃO E ROTEAMENTO
    fprintf(out, "\n[3] DECISÕES OPERACIONAIS (Apenas variáveis > 0)\n");
    fprintf(out, "--------------------------------------------------------\n");

    fprintf(out, "\n--- Alocação aos Hubs (Variável a) ---\n");
    for(int i = 0; i < n; i++) {
        for(int k = 0; k < n; k++) {
            if(cplex.getValue(a[i][k]) > 0.001) {
                fprintf(out, "Nó %d acessa a rede pelo Hub %d (Fração: %.2f)\n", i + 1, k + 1, cplex.getValue(a[i][k]));
            }
        }
    }

    fprintf(out, "\n--- Roteamento do Destino (Variável b) ---\n");
    for(int i = 0; i < n; i++) {
        for(int j = 0; j < n; j++) {
            for(int k = 0; k < n; k++) {
                if(cplex.getValue(b[i][j][k]) > 0.001) {
                    fprintf(out, "Demanda de %d para %d usa como último Hub o %d\n", i + 1, j + 1, k + 1);
                }
            }
        }
    }

    fprintf(out, "\n--- Fluxo Inter-Hubs (Variável f) ---\n");
    for(int i = 0; i < n; i++) {
        for(int k = 0; k < n; k++) {
            for(int m = 0; m < n; m++) {
                if(k != m && cplex.getValue(f[i][k][m]) > 0.001) {
                    fprintf(out, "Origem %d: Passa fluxo de %.2f do Hub %d para o Hub %d\n", i + 1, cplex.getValue(f[i][k][m]), k + 1, m + 1);
                }
            }
        }
    }
    
    fprintf(out, "\n==============================================\n\n");
}

void readData(Data& data, ifstream& file, int argc, char *argv[]) {

	file >> data.n;

	// cria n linhas em cada matriz
	data.w.assign(data.n, vector<double>(data.n));
    data.c.assign(data.n, vector<double>(data.n));
    data.r.assign(data.n, vector<double>(data.n));
    data.g.assign(data.n, vector<double>(data.n));
    data.s.resize(data.n);


	// soma para obter a média geral dos custos fixos de instalação hub 
	float sum = 0; 

	// Coletar demanda e custos CAB
	for (int i = 0; i <  data.n; i++) { 
		for (int j = 0; j < data.n; j++) {
			file >> data.w[i][j];
			file >> data.c[i][j];
			sum = sum + data.w[i][j];
		}
	}

	// Escalonando demanda CAB
	for (int i = 0; i <  data.n; i++) { 
		for (int j = 0; j < data.n; j++) {
		    data.w[i][j] = data.w[i][j]/sum;
		}
	}

	// Coletar fator de desconto nos links inter-hubs CAB
	if (argc >= 3) 
		data.alpha = atof(argv[2]);
	else data.alpha = 0.2;

	// Coletar receita CAB
	for (int i = 0; i <  data.n; i++) { 
		for (int j = 0; j < data.n; j++) {
			data.r[i][j] = atoi(argv[3]);
		}
	}

	// Coletar custo fixo de instalação CAB
	for (int i = 0; i <  data.n; i++) { 
		data.s[i] = atoi(argv[4]);
	}

	// Coletar custo de operar links inter-hubs CAB
	for (int i = 0; i <  data.n; i++) { 
		for (int j = 0; j < data.n; j++) {
			data.g[i][j] = atoi(argv[5]);
		}
	}
}

void createVariables(IloEnv& env, Data& data, IloModel& mod,
					IloArray<IloNumVarArray>& x,
					IloArray<IloNumVarArray>& z,
					IloArray<IloArray<IloNumVarArray>>& f,
					IloArray<IloNumVarArray>& a,
					IloArray<IloArray<IloNumVarArray>>& b,
					vector<double>& O) {

        
    // ILOBOOL x[k][k] indica se um hub é localizado no nó k 
    for(int i = 0; i < data.n; i++) {
         x[i] = IloNumVarArray(env, data.n, 0, 1, ILOBOOL); 
    }
        
    // ILOBOOL z[k][m] indica se um link inter-hub é operado entre os hubs k e m
    for(int k = 0; k < data.n; k++) {   
        z[k] = IloNumVarArray(env, data.n, 0, 1, ILOBOOL); 
	}
        
    // f[i][k][m] representa a quantidade de demanda originada no nó i e roteada no link inter-hub k-m
    for(int i = 0; i < data.n; i++) {
        f[i] = IloArray<IloNumVarArray>(env, data.n);
        for(int k = 0; k < data.n; k++) {
             f[i][k] = IloNumVarArray(env, data.n, 0, IloInfinity, ILOFLOAT);
        }
    }
        
    // a[i][k] é a fração da demanda que sai do nó i e acessa a rede pelo hub k
    for(int i = 0; i < data.n; i++) {
        a[i] = IloNumVarArray(env, data.n, 0, IloInfinity, ILOFLOAT); 
	}
        
    // b[i][j][m] é a fração da demanda entre os nós i e j que é roteada através de um caminho no qual o último hub é m
    for(int i = 0; i < data.n; i++) {
        b[i] = IloArray<IloNumVarArray>(env, data.n);
        for(int j = 0; j < data.n; j++) {
            b[i][j] = IloNumVarArray(env, data.n, 0, IloInfinity, ILOFLOAT);
        }
    }

    // o fluxo originado de um determinado nó i é dado por Oi = sum w_ij
    for(int i = 0; i < data.n; i++) {
        for(int j = 0; j < data.n; j++) {
            O[i] += data.w[i][j];
        }
    }

    IloExpr expfo(env);
    for(int i = 0; i < data.n; i++){
        for(int j = 0; j < data.n; j++) {

            for (int m = 0; m < data.n; m++) {
            expfo += (data.r[i][j] - data.c[m][j]) * data.w[i][j] * b[i][j][m];
            }

            for (int k = 0; k < data.n; k++) {
                expfo -= data.c[i][k] * data.w[i][j] * a[i][k];
            } 
        }
    }
        
    for (int k = 0; k <  data.n; k++) {
        expfo -= data.s[k] * x[k][k];
        for(int m = 0; m < data.n; m++) {
            expfo -=  data.g[k][m] * z[k][m];
            for (int i = 0; i < data.n; i++){
            expfo -=  data.alpha * data.c[k][m] * f[i][k][m];
            }
        }
    }
        
        IloAdd(mod, IloMaximize(env, expfo));
        expfo.end();
}

void createConstraints(IloEnv& env, IloModel& mod, Data& data,
                     IloArray<IloNumVarArray>& x,
                     IloArray<IloNumVarArray>& z,
                     IloArray<IloArray<IloNumVarArray>>& f,
                     IloArray<IloNumVarArray>& a,
                     IloArray<IloArray<IloNumVarArray>>& b,
                     vector<double>& O) {

    // R1: forall(i in 1..n) sum(k in 1..n) x[i][k] <= 1
    for (int i = 0; i < data.n; i++) {
        IloExpr r1(env);
        for (int k = 0; k < data.n; k++) {
            r1 += x[i][k];
        }
        mod.add(r1 <= 1);
        r1.end();
    }

    // R2: forall(i in 1..n, k in 1..n) x[i][k] <= x[k][k]  
    for(int i = 0; i < data.n; i++) {
        for (int k = 0; k < data.n; k++) {
            mod.add(x[i][k] <= x[k][k]);
        }
    }

    // R3: forall(i in 1..n) sum(k in 1..n) a[i][k] <= 1
    for (int i = 0; i < data.n; i++){
        IloExpr r3(env);
        for (int k = 0; k < data.n; k++){
            r3 += a[i][k];
        }
        mod.add(r3 <= 1);
        r3.end();
    }   
    
    // R4: forall(i in 1..n, j in 1..n) sum(m in 1..n) b[i][j][m] <= 1 
    for(int i = 0; i < data.n; i++){
        for (int j = 0; j < data.n; j++){
            IloExpr r4(env);
            for (int m = 0; m < data.n; m++) {
                r4 += b[i][j][m];
            }   
            mod.add(r4 <= 1);
            r4.end();
        }
    }

    // R5: a[i][k] <= x[i][k]
    for(int i = 0; i < data.n; i++){
        for (int k = 0; k < data.n; k++) {
            mod.add(a[i][k] <= x[i][k]);
        }
    }
    
    // R6: b[i][j][m] <= x[j][m]    
    for(int i = 0; i < data.n; i++) {
        for (int j = 0; j < data.n; j++){
            for (int m = 0; m < data.n; m++) {
                mod.add(b[i][j][m] <= x[j][m]);
            }
        }
    }

    // R7: Conservação de fluxo
    for(int i = 0; i < data.n; i++) {
        for (int k = 0; k < data.n; k++) {
            IloExpr r7_left(env);
            IloExpr r7_right(env);

            for(int m = 0; m < data.n; m++) {
                if(m != k) {
                    r7_left += f[i][m][k];
                    r7_right += f[i][k][m];
                }
            } 

            r7_left += O[i] * a[i][k]; 

            for(int j = 0; j < data.n; j++) {
                r7_right += data.w[i][j] * b[i][j][k];
            }

            mod.add(r7_left == r7_right);
            r7_left.end();
            r7_right.end();
        }
    }

    // R8: f[i][k][m] <= O[i] * z[k][m]
    for(int i = 0; i < data.n; i++) {
        for(int k = 0; k < data.n; k++) {
            for (int m = 0; m < data.n; m++) {
                if (k != m) {
                    mod.add(f[i][k][m] <= O[i] * z[k][m]);
                }
            }
        }
    }
    
    // R9: z[k][m] <= x[k][k] 
    for (int k = 0; k < data.n; k++) {
        for (int m = 0; m < data.n; m++){
            mod.add(z[k][m] <= x[k][k]);
        }
    }

    // R10: z[k][m] <= x[m][m]    
    for (int k = 0; k < data.n; k++){
        for (int m = 0; m < data.n; m++) {
            mod.add(z[k][m] <= x[m][m]);
        }
    }       

	/*
	As restrições de não-negatividade já são resolvidas na hora da criação das variáveis: IloNumVarArray(env, data.n, 0, IloInfinity, ILOFLOAT)
	com o limite inferior sendo zero.
	R11: forall(i in 1..n, k in 1..n, m in 1..n) f[i][k][m] >= 0
 	R12: forall(i in 1..n, k in 1..n) a[i][k] >= 0 
    R13: forall(i in 1..n, j in 1..n, m in 1..n) b[i][j][m] > =0 
	*/
}

void createVariablesTaherkhani(IloEnv& env, Data& data, IloModel& mod,
                               IloArray<IloNumVarArray>& x,
                               IloArray<IloArray<IloArray<IloNumVarArray>>>& y,
                               IloArray<IloNumVarArray>& z,
                               IloArray<IloArray<IloNumVarArray>>& f,
                               vector<double>& O) {
    
    // x[i][k]: 1 se nó i é alocado ao hub k (se i==k, o hub k está aberto) - Equação (12)
    for(int i = 0; i < data.n; i++) {
        x[i] = IloNumVarArray(env, data.n, 0, 1, ILOBOOL);
    }

    // z[k][m]: 1 se há link inter-hub entre k e m - Equação (14)
    for(int k = 0; k < data.n; k++) {
        z[k] = IloNumVarArray(env, data.n, 0, 1, ILOBOOL);
    }

    // y[i][j][k][m]: 1 se a demanda de i para j viaja via hubs k e m - Equação (13)
    for(int i = 0; i < data.n; i++) {
        y[i] = IloArray<IloArray<IloNumVarArray>>(env, data.n);
        for(int j = 0; j < data.n; j++) {
            y[i][j] = IloArray<IloNumVarArray>(env, data.n);
            for(int k = 0; k < data.n; k++) {
                y[i][j][k] = IloNumVarArray(env, data.n, 0, 1, ILOBOOL);
            }
        }
    }

    // f[i][k][m]: fluxo contínuo originado em i passando pelo link k-m - Equação (11)
    for(int i = 0; i < data.n; i++) {
        f[i] = IloArray<IloNumVarArray>(env, data.n);
        for(int k = 0; k < data.n; k++) {
            f[i][k] = IloNumVarArray(env, data.n, 0, IloInfinity, ILOFLOAT);
        }
    }

    // O[i] (Fluxo originado do nó i)
    for(int i = 0; i < data.n; i++) {
        for(int j = 0; j < data.n; j++) {
            O[i] += data.w[i][j];
        }
    }

    // Equação (1): Função Objetivo
    IloExpr obj(env);

    for(int i = 0; i < data.n; i++) {
        for(int j = 0; j < data.n; j++) {
            for(int k = 0; k < data.n; k++) {
                for(int m = 0; m < data.n; m++) {
                    // Receita (positivo)
                    obj += data.r[i][j] * data.w[i][j] * y[i][j][k][m];
                    // Custo de coleta e distribuição (negativo)
                    obj -= (data.c[i][k] + data.c[m][j]) * data.w[i][j] * y[i][j][k][m];
                }
            }
        }
    }

    for(int i = 0; i < data.n; i++) {
        for(int k = 0; k < data.n; k++) {
            for(int m = 0; m < data.n; m++) {
                // Custo de transferência com desconto alpha (negativo)
                obj -= data.alpha * data.c[k][m] * f[i][k][m];
            }
        }
    }

    for(int k = 0; k < data.n; k++) {
        // Custo fixo de instalação do hub (negativo)
        obj -= data.s[k] * x[k][k];
        for(int m = 0; m < data.n; m++) {
            if(k != m) {
                // Custo de operação do link inter-hub (negativo)
                obj -= data.g[k][m] * z[k][m];
            }
        }
    }

    mod.add(IloMaximize(env, obj));
    obj.end();
}

void createConstraintsTaherkhani(IloEnv& env, IloModel& mod, Data& data,
                                 IloArray<IloNumVarArray>& x,
                                 IloArray<IloArray<IloArray<IloNumVarArray>>>& y,
                                 IloArray<IloNumVarArray>& z,
                                 IloArray<IloArray<IloNumVarArray>>& f,
                                 vector<double>& O) {
    
    // (2): sum(k) sum(m) y[i][j][k][m] <= 1
    for(int i = 0; i < data.n; i++) {
        for(int j = 0; j < data.n; j++) {
            IloExpr r2(env);
            for(int k = 0; k < data.n; k++) {
                for(int m = 0; m < data.n; m++) {
                    r2 += y[i][j][k][m];
                }
            }
            mod.add(r2 <= 1);
            r2.end();
        }
    }

    // (3): sum(k) x[i][k] <= 1
    for(int i = 0; i < data.n; i++) {
        IloExpr r3(env);
        for(int k = 0; k < data.n; k++) {
            r3 += x[i][k];
        }
        mod.add(r3 <= 1);
        r3.end();
    }

    // (4): x[i][k] <= x[k][k]
    for(int i = 0; i < data.n; i++) {
        for(int k = 0; k < data.n; k++) {
            mod.add(x[i][k] <= x[k][k]);
        }
    }

    // (5): y[i][j][k][m] <= x[i][k]
    for(int i = 0; i < data.n; i++) {
        for(int j = 0; j < data.n; j++) {
            for(int k = 0; k < data.n; k++) {
                for(int m = 0; m < data.n; m++) {
                    mod.add(y[i][j][k][m] <= x[i][k]);
                }
            }
        }
    }

    // (6): y[i][j][k][m] <= x[j][m]
    for(int i = 0; i < data.n; i++) {
        for(int j = 0; j < data.n; j++) {
            for(int k = 0; k < data.n; k++) {
                for(int m = 0; m < data.n; m++) {
                    mod.add(y[i][j][k][m] <= x[j][m]);
                }
            }
        }
    }

    // (7)
    for(int i = 0; i < data.n; i++) {
        for(int k = 0; k < data.n; k++) {
            IloExpr r7_left(env);
            IloExpr r7_right(env);

            for(int m = 0; m < data.n; m++) {
                if(m != k) {
                    r7_left += f[i][m][k];
                    r7_right += f[i][k][m];
                }
            }

            for(int j = 0; j < data.n; j++) {
                for(int m = 0; m < data.n; m++) {
                    r7_left += data.w[i][j] * y[i][j][k][m];
                    r7_right += data.w[i][j] * y[i][j][m][k]; 
                }
            }
            
            mod.add(r7_left == r7_right);
            r7_left.end();
            r7_right.end();
        }
    }

    // (8): f[i][k][m] <= O[i] * z[k][m]
    for(int i = 0; i < data.n; i++) {
        for(int k = 0; k < data.n; k++) {
            for(int m = 0; m < data.n; m++) {
                if(k != m) {
                    mod.add(f[i][k][m] <= O[i] * z[k][m]);
                }
            }
        }
    }

    // (9): z[k][m] <= x[k][k]
    for(int k = 0; k < data.n; k++) {
        for(int m = 0; m < data.n; m++) {
            if(k != m) {
                mod.add(z[k][m] <= x[k][k]);
            }
        }
    }

    // (10): z[k][m] <= x[m][m]
    for(int k = 0; k < data.n; k++) {
        for(int m = 0; m < data.n; m++) {
            if(k != m) {
                mod.add(z[k][m] <= x[m][m]);
            }
        }
    }
}