#ifndef HEURISTIC_HPP
#define HEURISTIC_HPP

#include <vector>

#define EPSILON 1e-07 // Small number used for floating point comparisons
#define M 10e10       // Sufficiently large number (acts as "infinity")

/// ==============================================
/// Variable index recovery
/// ==============================================

// Retrieves the index of variable z_km (arc from hub k to hub m), k != m
inline int km(int n, int k, int m) {
    if (k < m) {
        return (n - 1) * k + m - 1;
    } else if (k > m) {
        return (n - 1) * k + m;
    }
    return 0; // Safety value when k == m
}

/// ==============================================
/// Heuristics class
/// ==============================================

class Heuristics {
public:
    // ---------- Instance parameters ----------
    int n;                                        // Number of nodes
    double alpha;                                 // Discount factor on the transportation cost of an inter-hub link
    double alpha_arg;                             // Value of alpha read from the command line
    double revenue_arg;                           // Value of the revenue read from the command line
    double hub_cost_arg;                          // Hub installation cost read from the command line (CAB)
    double hub_arc_cost_arg;                      // Hub arc installation cost read from the command line (CAB)
    std::vector<double> coord_x;                  // x coordinate of the nodes (AP)
    std::vector<double> coord_y;                  // y coordinate of the nodes (AP)
    std::vector<std::vector<double> > w;          // Demand to be sent between nodes i and j (n x n matrix)
    std::vector<std::vector<double> > r;          // Revenue for sending one unit of demand between nodes i and j
    std::vector<std::vector<double> > c;          // Cost of sending one unit of demand between nodes i and j
    std::vector<double> s;                        // Fixed cost of installing a hub
    std::vector<std::vector<double> > g;          // Operating cost of the inter-hub links
    double demand_sum;                            // Sum of all demands (used to normalize w)

    // ---------- Solution variables ----------
    std::vector<double> h;                        // Binary vector of active hubs (size n)
    std::vector<double> h_copy;                   // Copy of h used by the neighborhoods
    std::vector<double> h_prime;                  // Vector returned by the neighborhoods
    std::vector<double> h_star;                   // Best hubs vector found
    std::vector<double> z;                        // Binary vector of arcs between active hubs (size n*(n-1))
    std::vector<double> z_copy;                   // Copy of z used by the neighborhoods
    std::vector<double> z_prime;                  // Vector returned by the neighborhoods
    std::vector<double> z_star;                   // Best arcs vector found
    double solution_value = 0.0;                  // Value of the current solution
    double solution_value_prime;                  // Evaluation value returned by the neighborhoods
    double solution_value_star;                   // Value of the best solution found

    Heuristics() {} // Constructor

    // ---------- Input ----------
    void read_main_arg(int argc, char *argv[]); // Reads the optional command line arguments
    void read_data(char *file_name);            // Reads the instance file

    // ---------- Initial solution and evaluation ----------
    std::vector<double> single_hub_solution(); // Returns {hub, value} of the best solution with a single hub (if any)
    double shortest_path_algorithm();          // Evaluates the current solution (single allocation)
    double refine_solution();                  // Refines the allocation of the current solution and evaluates it

    // ---------- Neighborhoods ----------
    void add_hub();           // Inserts a hub (N1)
    void remove_hub();        // Removes a hub and the arcs incident to it (N2)
    void add_arc();           // Adds a hub arc between active hubs (N3)
    void remove_arc();        // Removes a hub arc (N4)
    void add_hub_with_arcs(); // Inserts a hub and connects it to the other installed hubs (N5)
    void swap_hub();          // Swaps an active hub with an inactive one and removes the arcs incident to the deactivated hub (N6)

    // ---------- Metaheuristic ----------
    void rvnd();               // Variable Neighborhood Descent (random neighborhood order)
    void perturbation(int level); // Applies the perturbation 'level' times
    void swap_hub_perturb();  // Random version of N6 used as perturbation
    void smart_ils(int iter_max); // Smart Iterated Local Search
};

#endif // HEURISTIC_HPP
