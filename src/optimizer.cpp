

// STDLib dependencies
#include <csignal>    // SIGINT and other signals
#include <sstream>    // ostringstream
#include <queue>      // queue
#include <iostream>   // cin, cout, endl
#include <chrono>     // time functions
#include <iomanip>    // setfill, setw
#include <cstring>    // strcpy

// Dependencies from this package
#include "kcmc_instance.h"  // KCMC Instance class headers
#include "genetic_algorithm_operators.h"  // exit_signal_handler


/** FLOOD-DINIC ALGORITM
 * For each POI, finds M node-disjoint paths connecting the POI to the SINK. Then "floods" the set of POIs found paths.
 * Flooding: Let path A connect POI P to sink S. Let A also be be a sequence of active sensors so that the first sensor
 * i0 in the sequence covers POI P, and the last sensor ik connects to sink S. So, A = (P)i0,i1,...ik(S).
 * The "flooded" version of path A is a set of sensors that contains, for each connected triple ix-1, ix, ix+1 in A,
 * all sensors that connect both to ix-1 and ix+1. At the starting edge of A, ix-1 is P. At the end edge of A, ix+1 is S
 */
int KCMC_Instance::flood(int k, int m, bool full,
                         std::unordered_set<int> &inactive_sensors, std::unordered_set<int> *visited_sensors) {

    // Base case
    if (m < 1){return -1;}

    // Create the level graph, loop controls and buffers
    bool break_loop;
    int level_graph[this->num_sensors], predecessors[this->num_sensors],
        paths_found, path_end, a_poi, path_length, longest_required_path_length, previous, next_i,
        total_paths_found = 0;

    // Update the level graph
    this->level_graph(level_graph, inactive_sensors);

    // Prepare the set of "used" sensors for each POI
    std::unordered_set<int> used_sensors;

    // Validate K-Coverage
    if (this->fast_k_coverage(k, inactive_sensors, &used_sensors) != -1) {
        throw std::runtime_error("INVALID INSTANCE! (INSUFFICIENT COVERAGE)");
    }

    // Reset the results buffer
    visited_sensors->clear();

    // Add all poi-covering sensors to the result buffer
    for (a_poi=0; a_poi < this->num_pois; a_poi++) {
        for (const int &a_sensor : this->poi_sensor[a_poi]) {
            visited_sensors->insert(a_sensor);
        }
    }

    // Run for each POI, returning at the first failure
    for (a_poi=0; a_poi < this->num_pois; a_poi++) {
        break_loop = false;  // Mark the loop for processing
        paths_found = 0;  // Clear the number of paths found for the POI
        longest_required_path_length = 0; // reset the stored length of the last found path
        used_sensors = inactive_sensors;  // Reset the set of used sensors for each POI

        // While the stopping criteria was not found
        while (not break_loop) {
            std::fill(predecessors, predecessors+this->num_sensors, -2);  // Reset the predecessors buffer

            // Find a path
            path_end = this->find_path(a_poi, used_sensors, level_graph, predecessors);

            // Reset the control buffers
            next_i = -1;
            path_length = 0;

            // If the path ends in an invalid sensor, mark the loop to end. If we do not have enough paths, throu error
            if (path_end == -1) {
                break_loop = true;
                if (paths_found < m) { throw std::runtime_error("INVALID INSTANCE! (INSUFFICIENT CONNECTIVITY)"); }
            }

            // If it is a sucessful path
            else {
                paths_found += 1;  // Count the newfound path
                total_paths_found += 1;

                // Unravel the path, marking each sensor in it as used and flooding it
                while (path_end != -1) {
                    used_sensors.insert(path_end);
                    path_length += 1;

                    // Get the previous sensor in the path
                    previous = predecessors[path_end];
                    if (previous == -2) { throw std::runtime_error("FORBIDDEN ADDRESS!"); }

                    /* If the previous sensor is a POI and the next is a SINK
                     * Add all active sensors that connect both to the POI and the SINK to the result buffer
                     */
                    if ((previous == -1) and (next_i == -1)) {
                        for (const int &bridge: this->poi_sensor[a_poi]) {
                            if (isin(this->sensor_sink, bridge) and (not isin(inactive_sensors, bridge))) {
                                visited_sensors->insert(bridge);
                            }
                        }
                    } else {
                        /* If the previous sensor is a POI (and the next cannot be a SINK)
                         * Add all active sensors that cover the POI and connect to the path_end sensor to the result
                         */
                        if (previous == -1) {
                            for (const int &cover: this->poi_sensor[a_poi]) {
                                if (isin(this->sensor_sensor[cover], path_end) and (not isin(inactive_sensors, cover))) {
                                    visited_sensors->insert(cover);
                                }
                            }
                        } else {
                            /* If the previous sensor is NOT a POI and the next IS a SINK
                             * Add all active sensors that connect to both the previous sensor and the sink
                             */
                            if (next_i == -1) {
                                for (const int &conn: this->sensor_sensor[previous]) {
                                    if (isin(this->sensor_sink, conn) and (not isin(inactive_sensors, conn))) {
                                        visited_sensors->insert(conn);
                                    }
                                }
                            } else {
                                /* If the previous sensor is NOT a POI ant the next is NOT a sink
                                 * Add all active sensors that connect to both the previous and the next to the result
                                 */
                                for (const int &conn: this->sensor_sensor[previous]) {
                                    if (isin(this->sensor_sensor[conn], next_i) and (not isin(inactive_sensors, conn))) {
                                        visited_sensors->insert(conn);
                                    }
                                }
                            }
                        }
                    }

                    // Mark that the next in the path is not a sink and advance the path
                    next_i = path_end;
                    path_end = previous;
                }

                /* FULL version:
                 * If we have enough paths, but the current is no larger than the last, continue the loop.
                 * MIN version:
                 * Stop as soon as we get M paths for this POI
                 */
                if (full) {
                    if (paths_found <= m) {
                        longest_required_path_length = (path_length > longest_required_path_length) ? path_length : longest_required_path_length;
                    }
                    if (path_length > longest_required_path_length) { break_loop = true; }
                } else {
                    longest_required_path_length = path_length;
                    if (paths_found == m) { break_loop = true; }
                }
            }
        }
    }

    // Success in each and every POI! Return the total of found paths
    return total_paths_found;
}


/* #####################################################################################################################
 * RUNTIME
 * */


void printout_short(KCMC_Instance *instance, int k, int m,
                    const int num_sensors, const std::string operation,
                    const long duration, std::unordered_set<int> &used_installation_spots) {

    // Validate the instance
    std::unordered_set<int> inactive_sensors;
    instance->invert_set(used_installation_spots, &inactive_sensors);
    bool valid = instance->validate(false, k, m, inactive_sensors);

    // Reformat the used installation spots as an array of 0/1
    int individual[num_sensors];
    std::fill(individual, individual+num_sensors, 0);
    for (const int &used_spot : used_installation_spots) { individual[used_spot] = 1; }

    // Prepare the output buffer
    std::ostringstream out;

    // Print a line with:
    // - The key of the instance
    // - The name of the current operation
    // - The amount of microsseconds the method needed to run
    // - The number of used installation spots
    // - The resulting map of the instance, as a binary of num_sensors bits
    out << instance->key() << "\t" << k << "\t" << m
        << "\t" << operation
        << "\t" << duration
        << "\t" << (valid ? "OK" : "INVALID")
        << "\t" << used_installation_spots.size()
        << "\t" << std::fixed << std::setprecision(5) << (double)(inactive_sensors.size()) / (double)num_sensors
        << "\t";
    for (int i=0; i<num_sensors; i++) {out << individual[i];}
    // Flush
    std::cout << out.str() << std::endl;
}


void help() {
    std::cout << "Please, use the correct input for the KCMC instance heuristic optimizer:" << std::endl << std::endl;
    std::cout << "./optimizer_dinic <instance> <k> <m>" << std::endl;
    std::cout << "  where:" << std::endl << std::endl;
    std::cout << "<instance> is the serialized KCMC instance" << std::endl;
    std::cout << "Integer 0 < K < 10 is the desired K coverage" << std::endl;
    std::cout << "Integer 0 < M < 10 is the desired M connectivity" << std::endl;
    std::cout << "K migth be the pair K,M in the format (K{k}M{m}). In this case M is ignored" << std::endl;
    exit(0);
}


int main(int argc, char* const argv[]) {
    if (argc < 3) { help(); }

    // Registers the signal handlers
    signal(SIGINT, exit_signal_handler);
    signal(SIGALRM, exit_signal_handler);
    signal(SIGABRT, exit_signal_handler);
    signal(SIGSTOP, exit_signal_handler);
    signal(SIGTERM, exit_signal_handler);
    signal(SIGKILL, exit_signal_handler);

    // Buffers
    int k, m, num_paths;
    std::string serialized_instance, alt_k;
    std::unordered_set<int> emptyset, used_installation_spots, seed_sensors;

    /* Parse base Arguments
     * Serialized KCMC Instance (will be immediately de-serialized)
     * KCMC K and M parameters
     * */
    auto *instance = new KCMC_Instance(argv[1]);
    alt_k = argv[2];
    std::transform(alt_k.begin(), alt_k.end(),alt_k.begin(), ::toupper);
    char p[alt_k.size()];
    strcpy(p, alt_k.c_str());
    if (alt_k.find('K') != std::string::npos) {
        k = ((int)p[2]) - ((int)'0');  // ONLY FOR K,M < 10!!!
        m = ((int)p[4]) - ((int)'0');  // ONLY FOR K,M < 10!!!
    } else {
        k = std::stoi(argv[2]);
        m = std::stoi(argv[3]);
    }

    // Prepare the clock buffers
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    long duration;

    // Print the header
    // printf("Key\tK\tM\tOperation\tRuntime\tValid\tObjective\tCompression\tSolution\n");

    // Validate the whole instance, getting the first local optima using DINIC Algorithm
    used_installation_spots.clear();
    start = std::chrono::high_resolution_clock::now();
    instance->local_optima(k, m, emptyset, &used_installation_spots);
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    printout_short(instance, k, m, instance->num_sensors, "dinic", duration, used_installation_spots);

    // Process the Minimal-Flood mapping of the instance
    used_installation_spots.clear();
    start = std::chrono::high_resolution_clock::now();
    num_paths = instance->flood(k, m, false, emptyset, &used_installation_spots);
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    printout_short(instance, k, m, instance->num_sensors,
                   "min_flood_" + std::to_string(num_paths),  // Add the number of paths found
                   duration, used_installation_spots);

    // Process the Max-Flood mapping of the instance
    used_installation_spots.clear();
    start = std::chrono::high_resolution_clock::now();
    num_paths = instance->flood(k, m, true, emptyset, &used_installation_spots);
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    printout_short(instance, k, m, instance->num_sensors,
                   "max_flood_" + std::to_string(num_paths),  // Add the number of paths found
                   duration, used_installation_spots);

    return 0;
}