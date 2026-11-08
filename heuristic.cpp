#include "heuristic.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <vector>

using namespace std;

int main(int argc, char *argv[]) {

    Heuristics *ht = new Heuristics();
    double cpu_start;
    double cpu_end;

    // ========= Data input ========= //
    ht->read_main_arg(argc, argv);
    ht->read_data(argv[1]);

    // ========= Initial solution ========= //
    cpu_start = clock();

    vector<double> single_hub_vector;
    single_hub_vector = ht->single_hub_solution(); // Initial solution
    cout << "active hub: " << single_hub_vector[0] << endl;
    cout << "single-hub solution value: " << single_hub_vector[1] << endl;

    ht->h[single_hub_vector[0]] = 1;
    ht->solution_value = single_hub_vector[1];

    double initial_value = ht->solution_value;

    // ========= SmartILS-RVND heuristic ========= //
    srand(time(NULL));
    ht->smart_ils(5);

    cpu_end = clock();
    printf("Execution time = %10.2f seconds\n", (double)(cpu_end - cpu_start) / CLOCKS_PER_SEC);
    cout << "SmartILS solution: " << fixed << setprecision(2) << ht->solution_value_star << endl;

    // ========= Output files ========= //
    ofstream output_file("results-heuristic.txt", std::ofstream::app);
    if (!output_file) {
        cerr << "Error opening file\n";
        exit(0);
    }

    output_file << argv[1] << "\t" << ht->alpha << "\t" << ht->r[0][0] << "\t" << ht->s[0] << "\t" << ht->g[0][0]
                << "\t" << fixed << setprecision(2) << initial_value
                << "\t" << fixed << setprecision(2) << ht->solution_value_star
                << "\t" << fixed << setprecision(2) << (double)(cpu_end - cpu_start) / CLOCKS_PER_SEC << endl;

    ofstream config_output_file("results-configs.txt", std::ofstream::app);
    if (!config_output_file) {
        cerr << "Error opening file\n";
        exit(0);
    }

    config_output_file << "\n \n" << argv[1] << "\t" << ht->alpha << "\t" << ht->r[0][0]
                       << "\t" << fixed << setprecision(2) << initial_value
                       << "\t" << fixed << setprecision(2) << ht->solution_value_star
                       << "\t" << fixed << setprecision(2) << (double)(cpu_end - cpu_start) / CLOCKS_PER_SEC << endl;

    config_output_file << "installed hubs: ";
    for (int k = 0; k < ht->n; k++) {
        if (ht->h_star[k] > (1 - EPSILON)) {
            config_output_file << k << "\t";
        }
    }

    config_output_file << "\n" << "installed arcs:";
    for (int k = 0; k < ht->n; k++) {
        for (int m = 0; m < ht->n; m++) {
            if (m != k) {
                if (ht->z_star[km(ht->n, k, m)] > (1 - EPSILON)) {
                    config_output_file << "(" << k << "," << m << ")" << "\t";
                }
            }
        }
    }

    return 0;
}

// ===========
// Input
// ===========

// Reads the optional command line arguments (default values are used when absent)
void Heuristics::read_main_arg(int argc, char *argv[]) {
    alpha_arg = (argc > 2) ? atof(argv[2]) : 0.2;           // Alpha value
    revenue_arg = (argc > 3) ? atof(argv[3]) : 1000;        // Revenue value
    hub_cost_arg = (argc > 4) ? atof(argv[4]) : 50;         // Hub installation cost 
    hub_arc_cost_arg = (argc > 5) ? atof(argv[5]) : 5;      // Hub arc installation cost
}

// Reads the instance file and initializes all instance data and solution vectors
void Heuristics::read_data(char *file_name) {
    ifstream input_file(file_name);
    if (!input_file) {
        cerr << "Error opening file\n";
        exit(0);
    }

    input_file >> n;

    w = vector<vector<double> >(n, vector<double>(n)); // Demand to be sent between nodes i and j
    r = vector<vector<double> >(n, vector<double>(n)); // Revenue for sending one unit of demand between nodes i and j
    c = vector<vector<double> >(n, vector<double>(n)); // Cost of sending one unit of demand between nodes i and j
    s = vector<double>(n);                             // Fixed cost of installing a hub
    g = vector<vector<double> >(n, vector<double>(n)); // Operating cost of the inter-hub links
    h = vector<double>(n, 0);                          // Active hubs vector initialized with zeros
    z = vector<double>(n * (n - 1), 0);                // Arcs between active hubs initialized with zeros

    // ============== CAB data ============== //
    demand_sum = 0;

    for (int i = 0; i < n; i++) { // Read demand and costs (CAB)
        for (int j = 0; j < n; j++) {
            input_file >> w[i][j];
            input_file >> c[i][j];
            demand_sum = demand_sum + w[i][j];
        }
    }

    for (int i = 0; i < n; i++) { // Normalize demand (CAB)
        for (int j = 0; j < n; j++) {
            w[i][j] = w[i][j] / demand_sum;
        }
    }

    alpha = alpha_arg; // Discount factor between inter-hub nodes (AP)

    for (int i = 0; i < n; i++) { // Set revenue (CAB)
        for (int j = 0; j < n; j++) {
            r[i][j] = revenue_arg;
        }
    }

    for (int i = 0; i < n; i++) { // Set hub fixed installation cost (CAB)
        s[i] = hub_cost_arg; // Typical values: 50, 100, 150
    }

    for (int i = 0; i < n; i++) { // Set inter-hub link operating cost (CAB)
        for (int j = 0; j < n; j++) {
            g[i][j] = hub_arc_cost_arg; // Typical values: 5, 10, 15
        }
    }

    input_file.close();
}

// Initial solution
// best solution with a single hub in the network.
// Returns a vector with the hub (position 0) and the solution value (position 1).
vector<double> Heuristics::single_hub_solution() {

    double phi = 0.0;
    double phi_aux;
    double active_hub = -1;
    vector<vector<double> > l(n, vector<double>(n));
    vector<double> single_hub_result(2);

    for (int k = 0; k < n; k++) {
        phi_aux = -s[k];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                l[i][j] = r[i][j] * w[i][j] - ((c[i][k] + c[k][j]) * w[i][j]);
                if (l[i][j] > EPSILON) {
                    phi_aux += l[i][j];
                }
            }
        }
        if (phi_aux >= phi) {
            phi = phi_aux;
            active_hub = k;
        }
    }
    single_hub_result[0] = active_hub;
    single_hub_result[1] = phi;

    if (active_hub >= 0)
        h[active_hub] = 1;

    return single_hub_result;
}

// =========================
// Solution evaluation
// =========================

// Evaluates the current solution (h, z): single allocation with hub optimization per node
double Heuristics::shortest_path_algorithm() {

    vector<vector<double> > hub_net_sp_matrix = vector<vector<double> >(n, vector<double>(n)); // Shortest path considering only the hub network
    vector<vector<double> > sp_matrix = vector<vector<double> >(n, vector<double>(n));         // Shortest path for all nodes

    // 1. Compute the shortest paths between hub pairs in the hub sub-network
    double sol_value = 0.0; // Accumulates the installation cost of hubs and hub arcs
    list<int> hub_list;     // List of hubs

    for (int k = 0; k < n; k++) { // Build the list of hubs
        if (h[k] >= (1 - EPSILON)) {
            hub_list.insert(hub_list.end(), k);
            sol_value -= s[k];
        }
    }

    for (list<int>::iterator k = hub_list.begin(); k != hub_list.end(); k++) {
        for (list<int>::iterator m = hub_list.begin(); m != hub_list.end(); m++) {
            if (k != m) {
                if (z[km(n, *k, *m)] >= (1 - EPSILON)) {
                    hub_net_sp_matrix[*k][*m] = alpha * c[*k][*m];
                    sol_value -= g[*k][*m];
                } else {
                    hub_net_sp_matrix[*k][*m] = M;
                }
            } else {
                hub_net_sp_matrix[*k][*m] = 0.0;
            }
        }
    }

    // Floyd-Warshall on the hub network
    for (list<int>::iterator k = hub_list.begin(); k != hub_list.end(); k++) {
        for (list<int>::iterator m = hub_list.begin(); m != hub_list.end(); m++) {
            for (list<int>::iterator l = hub_list.begin(); l != hub_list.end(); l++) {
                if (m != l and m != k and l != k) {
                    double cost_aux = hub_net_sp_matrix[*m][*k] + hub_net_sp_matrix[*k][*l];
                    if (cost_aux < hub_net_sp_matrix[*m][*l]) {
                        hub_net_sp_matrix[*m][*l] = cost_aux;
                    }
                }
            }
        }
    }

    // 2. Optimize and define the SINGLE fixed hub of each node
    vector<int> fixed_hub(n, -1);

    for (int i = 0; i < n; i++) {
        int best_hub = -1;
        double min_total_cost = M;

        for (list<int>::iterator k_it = hub_list.begin(); k_it != hub_list.end(); k_it++) {
            int k = *k_it;
            double cost_sum = 0.0;

            for (int j = 0; j < n; j++) {
                double best_dest_path = M;
                for (list<int>::iterator m_it = hub_list.begin(); m_it != hub_list.end(); m_it++) {
                    int m = *m_it;
                    double path = hub_net_sp_matrix[k][m] + c[m][j];
                    if (path < best_dest_path) {
                        best_dest_path = path;
                    }
                }
                cost_sum += (c[i][k] + best_dest_path) * w[i][j];
            }

            if (cost_sum < min_total_cost) {
                min_total_cost = cost_sum;
                best_hub = k;
            }
        }
        fixed_hub[i] = best_hub; // Fills the whole vector for every node i (0 to n-1)
    }

    // 3. Compute the final shortest path matrix (single allocation)
    for (int i = 0; i < n; i++) {
        int best_hub_i = fixed_hub[i];

        for (int j = 0; j < n; j++) {
            sp_matrix[i][j] = M;
            int best_hub_j = fixed_hub[j];

            if (best_hub_i != -1 && best_hub_j != -1) {
                sp_matrix[i][j] = c[i][best_hub_i] + hub_net_sp_matrix[best_hub_i][best_hub_j] + c[best_hub_j][j];
            }

            if (r[i][j] - sp_matrix[i][j] > EPSILON) {
                sol_value += (r[i][j] - sp_matrix[i][j]) * w[i][j];
            }
        }
    }

    hub_net_sp_matrix.clear();
    hub_list.clear();
    return sol_value;
}

// Solution refinement: starts from the greedy single allocation and improves
// the allocation of each node with a local search
double Heuristics::refine_solution() {

    vector<vector<double> > hub_net_sp_matrix = vector<vector<double> >(n, vector<double>(n)); // Shortest path considering only the hub network
    vector<vector<double> > sp_matrix = vector<vector<double> >(n, vector<double>(n));         // Shortest path for all nodes

    // 1. Compute the shortest paths between hub pairs in the hub sub-network
    double sol_value = 0.0; // Accumulates the installation cost of hubs and hub arcs
    list<int> hub_list;     // List of hubs

    for (int k = 0; k < n; k++) { // Build the list of hubs
        if (h[k] >= (1 - EPSILON)) {
            hub_list.insert(hub_list.end(), k);
            sol_value -= s[k];
        }
    }

    for (list<int>::iterator k = hub_list.begin(); k != hub_list.end(); k++) {
        for (list<int>::iterator m = hub_list.begin(); m != hub_list.end(); m++) {
            if (k != m) {
                if (z[km(n, *k, *m)] >= (1 - EPSILON)) {
                    hub_net_sp_matrix[*k][*m] = alpha * c[*k][*m];
                    sol_value -= g[*k][*m];
                } else {
                    hub_net_sp_matrix[*k][*m] = M;
                }
            } else {
                hub_net_sp_matrix[*k][*m] = 0.0;
            }
        }
    }

    // Floyd-Warshall on the hub network
    for (list<int>::iterator k = hub_list.begin(); k != hub_list.end(); k++) {
        for (list<int>::iterator m = hub_list.begin(); m != hub_list.end(); m++) {
            for (list<int>::iterator l = hub_list.begin(); l != hub_list.end(); l++) {
                if (m != l and m != k and l != k) {
                    double cost_aux = hub_net_sp_matrix[*m][*k] + hub_net_sp_matrix[*k][*l];
                    if (cost_aux < hub_net_sp_matrix[*m][*l]) {
                        hub_net_sp_matrix[*m][*l] = cost_aux;
                    }
                }
            }
        }
    }

    // 2. Optimize and define the SINGLE initial fixed hub of each node (greedy)
    vector<int> fixed_hub(n, -1);

    for (int i = 0; i < n; i++) {
        int best_hub = -1;
        double min_total_cost = M;

        for (list<int>::iterator k_it = hub_list.begin(); k_it != hub_list.end(); k_it++) {
            int k = *k_it;
            double cost_sum = 0.0;

            for (int j = 0; j < n; j++) {
                double best_dest_path = M;
                for (list<int>::iterator m_it = hub_list.begin(); m_it != hub_list.end(); m_it++) {
                    int m = *m_it;
                    double path = hub_net_sp_matrix[k][m] + c[m][j];
                    if (path < best_dest_path) {
                        best_dest_path = path;
                    }
                }
                cost_sum += (c[i][k] + best_dest_path) * w[i][j];
            }

            if (cost_sum < min_total_cost) {
                min_total_cost = cost_sum;
                best_hub = k;
            }
        }
        fixed_hub[i] = best_hub;
    }

    // 2.1. Lambda that computes the global revenue
    auto compute_revenue = [&](const vector<int> &allocation) {
        double total_revenue = 0.0;
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                int k = allocation[i];
                int m = allocation[j];

                double path_cost = c[i][k] + hub_net_sp_matrix[k][m] + c[m][j];
                double pair_revenue = max(0.0, r[i][j] - path_cost);
                total_revenue += w[i][j] * pair_revenue;
            }
        }
        return total_revenue;
    };

    // 2.2. Local search / allocation refinement
    double current_revenue = compute_revenue(fixed_hub);
    bool improved = true;
    int iteration = 0;

    while (improved && iteration < 50) { // Protection against endless loops
        improved = false;
        iteration++;

        for (int i = 0; i < n; i++) {
            int best_hub = fixed_hub[i];
            double best_revenue = current_revenue;

            for (list<int>::iterator k_it = hub_list.begin(); k_it != hub_list.end(); k_it++) {
                int k = *k_it;

                vector<int> test_allocation = fixed_hub;
                test_allocation[i] = k;

                double new_revenue = compute_revenue(test_allocation);

                if (new_revenue > best_revenue + EPSILON) {
                    best_revenue = new_revenue;
                    best_hub = k;
                }
            }

            if (best_hub != fixed_hub[i]) {
                fixed_hub[i] = best_hub;
                current_revenue = best_revenue;
                improved = true;
            }
        }
    }

    // 3. Compute the final path matrix (strict single allocation)
    for (int i = 0; i < n; i++) {
        int best_hub_i = fixed_hub[i];

        for (int j = 0; j < n; j++) {
            int best_hub_j = fixed_hub[j];

            if (best_hub_i != -1 && best_hub_j != -1) {
                sp_matrix[i][j] = c[i][best_hub_i] + hub_net_sp_matrix[best_hub_i][best_hub_j] + c[best_hub_j][j];
            } else {
                sp_matrix[i][j] = M;
            }
        }
    }

    // 4. Add the final revenue to the solution value
    sol_value += current_revenue;

    hub_net_sp_matrix.clear();
    hub_list.clear();
    return sol_value;
}

// =========================
// Neighborhoods
// =========================

// Inserts a hub (N1)
void Heuristics::add_hub() {

    vector<double> inactive_hubs;
    double evaluation_value = 0.0;
    int nb_size; // Size of the neighborhood to be explored
    int position;

    h_copy = h;
    h_prime = h;
    solution_value_prime = solution_value;
    z_prime = z;

    for (int k = 0; k < n; k++) { // Collect the inactive hubs
        if (h[k] < EPSILON) {
            inactive_hubs.push_back(k);
        }
    }

    if (!inactive_hubs.empty()) {
        nb_size = 1 * (inactive_hubs.size());
        for (int j = 0; j < nb_size; j++) {
            if (j == 0) {
                position = rand() % inactive_hubs.size();
                h[inactive_hubs[position]] = 1;
                solution_value_prime = shortest_path_algorithm();
                h_prime = h;
                inactive_hubs.erase(inactive_hubs.begin() + position);
                h = h_copy;
            } else {
                position = rand() % inactive_hubs.size();
                h[inactive_hubs[position]] = 1;
                evaluation_value = shortest_path_algorithm();
                if (evaluation_value > solution_value_prime) {
                    h_prime = h;
                    solution_value_prime = evaluation_value;
                }
                inactive_hubs.erase(inactive_hubs.begin() + position);
                h = h_copy;
            }
        }
    }
    inactive_hubs.clear();
}

// Inserts a hub and connects it to the other installed hubs (N5)
void Heuristics::add_hub_with_arcs() {

    vector<double> inactive_hubs;
    vector<double> active_hubs;
    double evaluation_value = 0.0;
    int nb_size; // Size of the neighborhood to be explored
    int position;

    h_copy = h;
    h_prime = h;
    solution_value_prime = solution_value;
    z_copy = z;
    z_prime = z;

    for (int k = 0; k < n; k++) { // Collect the inactive and active hubs
        if (h[k] < EPSILON) {
            inactive_hubs.push_back(k);
        } else {
            active_hubs.push_back(k);
        }
    }

    if (!inactive_hubs.empty()) {
        nb_size = 1 * (inactive_hubs.size());
        for (int j = 0; j < nb_size; j++) {
            if (j == 0) {
                position = rand() % inactive_hubs.size();
                h[inactive_hubs[position]] = 1;
                for (int k = 0; k < active_hubs.size(); k++) {
                    z[km(n, inactive_hubs[position], active_hubs[k])] = 1;
                    z[km(n, active_hubs[k], inactive_hubs[position])] = 1;
                }
                solution_value_prime = shortest_path_algorithm();
                h_prime = h;
                z_prime = z;
                inactive_hubs.erase(inactive_hubs.begin() + position);
                h = h_copy;
                z = z_copy;
            } else {
                position = rand() % inactive_hubs.size();
                h[inactive_hubs[position]] = 1;
                for (int k = 0; k < active_hubs.size(); k++) {
                    z[km(n, inactive_hubs[position], active_hubs[k])] = 1;
                    z[km(n, active_hubs[k], inactive_hubs[position])] = 1;
                }
                evaluation_value = shortest_path_algorithm();
                if (evaluation_value > solution_value_prime) {
                    h_prime = h;
                    z_prime = z;
                    solution_value_prime = evaluation_value;
                }
                inactive_hubs.erase(inactive_hubs.begin() + position);
                h = h_copy;
                z = z_copy;
            }
        }
    }
    inactive_hubs.clear();
    active_hubs.clear();
}

// Removes an installed hub (N2)
void Heuristics::remove_hub() {

    vector<double> active_hubs;
    vector<int> arc_from;
    vector<int> arc_to;
    double evaluation_value = 0.0;
    int nb_size; // Size of the neighborhood to be explored
    int position;

    h_copy = h;
    h_prime = h;
    z_copy = z;
    z_prime = z;
    solution_value_prime = solution_value;

    for (int k = 0; k < n; k++) { // Collect the active hubs
        if (h[k] > (1 - EPSILON)) {
            active_hubs.push_back(k);
        }
    }

    for (int k = 0; k < n; k++) { // Collect the arcs between active hubs
        for (int m = 0; m < n; m++) {
            if (m != k) {
                if (z[km(n, k, m)] > (1 - EPSILON)) {
                    arc_from.push_back(k);
                    arc_to.push_back(m);
                }
            }
        }
    }

    if (active_hubs.size() > 1) {

        nb_size = 1 * active_hubs.size();
        for (int j = 0; j < nb_size; j++) {
            if (j == 0) {
                position = rand() % active_hubs.size();
                h[active_hubs[position]] = 0;
                h_prime = h;
                if (!arc_from.empty()) {
                    for (int i = 0; i < arc_from.size(); i++) {
                        if ((arc_from[i] == active_hubs[position]) || (arc_to[i] == active_hubs[position])) {
                            z[km(n, arc_from[i], arc_to[i])] = 0;
                        }
                    }
                }
                z_prime = z;
                solution_value_prime = shortest_path_algorithm();
                active_hubs.erase(active_hubs.begin() + position);
                h = h_copy;
                z = z_copy;
            } else {
                position = rand() % active_hubs.size();
                h[active_hubs[position]] = 0;
                if (!arc_from.empty()) {
                    for (int i = 0; i < arc_from.size(); i++) {
                        if ((arc_from[i] == active_hubs[position]) || (arc_to[i] == active_hubs[position])) {
                            z[km(n, arc_from[i], arc_to[i])] = 0;
                        }
                    }
                }
                evaluation_value = shortest_path_algorithm();

                if (evaluation_value > solution_value_prime) {
                    h_prime = h;
                    z_prime = z;
                    solution_value_prime = evaluation_value;
                }
                active_hubs.erase(active_hubs.begin() + position);
                h = h_copy;
                z = z_copy;
            }
        }
    }
    active_hubs.clear();
    arc_from.clear();
    arc_to.clear();
}

// Installs an arc between installed hubs (N3)
void Heuristics::add_arc() {

    vector<double> active_hubs;
    vector<int> arc_from;
    vector<int> arc_to;
    double evaluation_value = 0.0;
    int nb_size; // Size of the neighborhood to be explored
    int position;

    h_prime = h;
    z_copy = z;
    z_prime = z;
    solution_value_prime = solution_value;

    for (int k = 0; k < n; k++) { // Collect the active hubs
        if (h[k] > (1 - EPSILON)) {
            active_hubs.push_back(k);
        }
    }

    if (active_hubs.size() > 1) { // Set of possible arcs between hubs
        for (int i = 0; i < active_hubs.size(); i++) {
            for (int j = 0; j < active_hubs.size(); j++) {
                if ((i != j) && (z[km(n, active_hubs[i], active_hubs[j])] < EPSILON)) {
                    arc_from.push_back(active_hubs[i]);
                    arc_to.push_back(active_hubs[j]);
                }
            }
        }
        nb_size = 1 * arc_from.size();
        for (int j = 0; j < nb_size; j++) {
            if (j == 0) {
                position = rand() % arc_from.size();
                z[km(n, arc_from[position], arc_to[position])] = 1;
                z_prime = z;
                solution_value_prime = shortest_path_algorithm();
                arc_from.erase(arc_from.begin() + position);
                arc_to.erase(arc_to.begin() + position);
                z = z_copy;
            } else {
                position = rand() % arc_from.size();
                z[km(n, arc_from[position], arc_to[position])] = 1;
                evaluation_value = shortest_path_algorithm();

                if (evaluation_value > solution_value_prime) {
                    z_prime = z;
                    solution_value_prime = evaluation_value;
                }
                arc_from.erase(arc_from.begin() + position);
                arc_to.erase(arc_to.begin() + position);
                z = z_copy;
            }
        }
    }
    active_hubs.clear();
    arc_from.clear();
    arc_to.clear();
}

// Removes an arc between hubs (N4)
void Heuristics::remove_arc() {

    vector<int> arc_from;
    vector<int> arc_to;
    double evaluation_value = 0.0;
    int nb_size; // Size of the neighborhood to be explored
    int position;

    h_prime = h;
    z_copy = z;
    z_prime = z;
    solution_value_prime = solution_value;

    for (int k = 0; k < n; k++) { // Collect the arcs between active hubs
        for (int m = 0; m < n; m++) {
            if (m != k) {
                if (z[km(n, k, m)] > (1 - EPSILON)) {
                    arc_from.push_back(k);
                    arc_to.push_back(m);
                }
            }
        }
    }

    if (!arc_from.empty()) {
        nb_size = 1 * arc_from.size();
        for (int j = 0; j < nb_size; j++) {
            if (j == 0) {
                position = rand() % arc_from.size();
                z[km(n, arc_from[position], arc_to[position])] = 0;
                z_prime = z;
                solution_value_prime = shortest_path_algorithm();
                arc_from.erase(arc_from.begin() + position);
                arc_to.erase(arc_to.begin() + position);
                z = z_copy;
            } else {
                position = rand() % arc_from.size();
                z[km(n, arc_from[position], arc_to[position])] = 0;
                evaluation_value = shortest_path_algorithm();
                if (evaluation_value > solution_value_prime) {
                    z_prime = z;
                    solution_value_prime = evaluation_value;
                }
                arc_from.erase(arc_from.begin() + position);
                arc_to.erase(arc_to.begin() + position);
                z = z_copy;
            }
        }
    }
    arc_from.clear();
    arc_to.clear();
}

// Swaps an installed hub with a non-installed one (N6)
void Heuristics::swap_hub() {

    vector<double> active_hubs;
    vector<double> inactive_hubs;
    vector<double> inactive_hubs_copy;
    vector<int> arc_from;
    vector<int> arc_to;
    double evaluation_value = 0.0;
    int nb_size; // Size of the neighborhood to be explored
    int position;

    h_copy = h;
    h_prime = h;
    z_copy = z;
    z_prime = z;
    solution_value_prime = solution_value;

    for (int k = 0; k < n; k++) { // Collect the active and inactive hubs
        if (h[k] > (1 - EPSILON)) {
            active_hubs.push_back(k);
        } else {
            inactive_hubs.push_back(k);
        }
    }

    inactive_hubs_copy = inactive_hubs;

    for (int k = 0; k < n; k++) { // Collect the arcs between active hubs
        for (int m = 0; m < n; m++) {
            if (m != k) {
                if (z[km(n, k, m)] > (1 - EPSILON)) {
                    arc_from.push_back(k);
                    arc_to.push_back(m);
                }
            }
        }
    }

    if (active_hubs.empty()) { // No hub installed: a hub is activated randomly and neighborhood 6 is restarted
        position = rand() % inactive_hubs.size();
        h[inactive_hubs[position]] = 1;
        solution_value = shortest_path_algorithm();
        swap_hub();
        cout << "entered here" << endl;
    } else {
        for (int i = 0; i < active_hubs.size(); i++) {
            nb_size = 1 * inactive_hubs.size();
            for (int j = 0; j < nb_size; j++) {
                if (i == 0 && j == 0) {
                    h[active_hubs[i]] = 0;
                    position = rand() % inactive_hubs.size();
                    h[inactive_hubs[position]] = 1;
                    h_prime = h;
                    if (!arc_from.empty()) {
                        for (int k = 0; k < arc_from.size(); k++) {
                            if ((arc_from[k] == active_hubs[i]) || (arc_to[k] == active_hubs[i])) {
                                z[km(n, arc_from[k], arc_to[k])] = 0;
                            }
                        }
                    }
                    z_prime = z;
                    solution_value_prime = shortest_path_algorithm();
                    inactive_hubs.erase(inactive_hubs.begin() + position);
                    h = h_copy;
                    z = z_copy;
                } else {
                    h[active_hubs[i]] = 0;
                    position = rand() % inactive_hubs.size();
                    h[inactive_hubs[position]] = 1;
                    if (!arc_from.empty()) {
                        for (int k = 0; k < arc_from.size(); k++) {
                            if ((arc_from[k] == active_hubs[i]) || (arc_to[k] == active_hubs[i])) {
                                z[km(n, arc_from[k], arc_to[k])] = 0;
                            }
                        }
                    }
                    evaluation_value = shortest_path_algorithm();

                    if (evaluation_value > solution_value_prime) {
                        h_prime = h;
                        z_prime = z;
                        solution_value_prime = evaluation_value;
                    }
                    inactive_hubs.erase(inactive_hubs.begin() + position);
                    h = h_copy;
                    z = z_copy;
                }
            }
            inactive_hubs = inactive_hubs_copy;
        }
    }
    active_hubs.clear();
    inactive_hubs.clear();
    arc_from.clear();
    arc_to.clear();
}

// Random version of N6 used as perturbation: swaps a random active hub with a random inactive one
void Heuristics::swap_hub_perturb() {

    vector<double> active_hubs;
    vector<double> inactive_hubs;
    vector<double> inactive_hubs_copy;
    vector<int> arc_from;
    vector<int> arc_to;
    int position;
    int position2;

    for (int k = 0; k < n; k++) { // Collect the active and inactive hubs
        if (h[k] > (1 - EPSILON)) {
            active_hubs.push_back(k);
        } else {
            inactive_hubs.push_back(k);
        }
    }

    for (int k = 0; k < n; k++) { // Collect the arcs between active hubs
        for (int m = 0; m < n; m++) {
            if (m != k) {
                if (z[km(n, k, m)] > (1 - EPSILON)) {
                    arc_from.push_back(k);
                    arc_to.push_back(m);
                }
            }
        }
    }

    if (active_hubs.empty()) { // No hub installed: a hub is activated randomly and the perturbation is restarted
        position = rand() % inactive_hubs.size();
        h[inactive_hubs[position]] = 1;
        solution_value = shortest_path_algorithm();
        swap_hub_perturb();
    } else {
        position = rand() % active_hubs.size();
        h[active_hubs[position]] = 0;
        position2 = rand() % inactive_hubs.size();
        h[inactive_hubs[position2]] = 1;
        if (!arc_from.empty()) {
            for (int k = 0; k < arc_from.size(); k++) {
                if ((arc_from[k] == active_hubs[position]) || (arc_to[k] == active_hubs[position])) {
                    z[km(n, arc_from[k], arc_to[k])] = 0;
                }
            }
        }
        solution_value = shortest_path_algorithm();
    }
    active_hubs.clear();
    inactive_hubs.clear();
    arc_from.clear();
    arc_to.clear();
}

/// =====================
/// Metaheuristic
/// =====================

// RVND: Variable Neighborhood Descent with random neighborhood order.
// Updates h, z and solution_value.
void Heuristics::rvnd() {

    vector<int> nb_order;

    for (int i = 0; i < 6; i++) {
        nb_order.push_back(i);
    }

    int aux, j1, j2;

    for (int i = 0; i < 6; i++) { // Shuffle the neighborhood order
        j1 = rand() % (6);
        j2 = rand() % (6);
        while (j1 == j2) j2 = rand() % (6);
        aux = nb_order[j1];
        nb_order[j1] = nb_order[j2];
        nb_order[j2] = aux;
    }

    int i = 0;

    while (i < 6) {
        if (nb_order[i] == 0) {
            add_hub();
        } else if (nb_order[i] == 1) {
            add_arc();
        } else if (nb_order[i] == 2) {
            remove_arc();
        } else if (nb_order[i] == 3) {
            add_hub_with_arcs();
        } else if (nb_order[i] == 4) {
            remove_hub();
        } else {
            swap_hub();
        }

        if (solution_value_prime > solution_value) {
            h = h_prime;
            z = z_prime;
            solution_value = solution_value_prime;
            i = 0;
        } else {
            i = i + 1;
        }
    }
}

// Perturbation: applies N6 perturbation 'level' times.
// Updates h, z and solution_value.
void Heuristics::perturbation(int level) {
    int i = 1;
    while (i <= level) {
        swap_hub_perturb();
        i = i + 1;
    }
}

// Smart Iterated Local Search (SmartILS)
void Heuristics::smart_ils(int iter_max) {

    // ========= Initial RVND ========= //
    rvnd();
    h_star = h;
    z_star = z;
    solution_value_star = solution_value;
    vector<double> h_backup = h;
    vector<double> z_backup = z;
    double solution_value_backup = solution_value;
    int iter = 0;
    int level = 1;
    int no_improve_count = 1;
    int max_no_improve = 3; // Number of iterations without improvement before increasing the perturbation level

    // ========= SmartILS main loop ========= //
    while (iter < iter_max) {
        perturbation(level);
        rvnd();
        if (solution_value > solution_value_star) {
            solution_value_star = refine_solution();
            h_star = h;
            z_star = z;
            iter = 0;
            level = 1;
            h_backup = h;
            z_backup = z;
            solution_value_backup = solution_value;
        } else {
            iter = iter + 1;
            if (no_improve_count >= max_no_improve) {
                level = level + 1;
                no_improve_count = 1;
            } else {
                no_improve_count = no_improve_count + 1;
            }
            h = h_backup;
            z = z_backup;
            solution_value = solution_value_backup;
        }
    }
}