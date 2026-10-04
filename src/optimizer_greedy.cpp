

// STDLib dependencies
#include <sstream>    // ostringstream
#include <queue>      // queue
#include <iostream>   // cin, cout, endl
#include <iomanip>    // setfill, setw

// Dependencies from this package
#include "kcmc_instance.h"  // KCMC Instance class headers

// Constants to improve readability
constexpr int UNVISITED_SENSOR = -2;
constexpr int INVALID_PREVIOUS_POI = -1;


/** GREEDY DKOV
 * Use the greedy heuristic to get sensors enough for m-connectivity, and then add sensors to also achieve k-coverage.
 */
int KCMC_Instance::greedy_dkov(const int k, const int m, std::unordered_set<int> &inactive_sensors, std::unordered_map<int, int> *visited_sensors) {

    // Local buffers
    int num_paths, p, missing_coverage, pre_k_cov_sensors;
    std::unordered_set<int> set_used_sensors, poi_covering_sensors, pois_missing_coverage;

    // We run the greedy M-connectivity normally
    visited_sensors->clear();
    num_paths = this->m_connectivity_greedy(m, inactive_sensors, visited_sensors);
    if (num_paths >= LARGE_NUMBER) {
        throw std::runtime_error("Failed to find " + std::to_string(m) + " paths for POI " + std::to_string((num_paths/LARGE_NUMBER)-1) + " (only found " + std::to_string(num_paths%LARGE_NUMBER) + ")");
    }

    // Run the ADD-KCOV procedure to ensure k-coverage
    pre_k_cov_sensors = ((int)(visited_sensors->size()));
    this->add_kcov(k, visited_sensors);
    return pre_k_cov_sensors;
}


/** REUSE
 * To minimize the number of sensors, we try to maximize reuse of sensors (i.e. the same sensor is used in multiple
 * connection paths, each from a different POI).
 * To do that, we compute the greedy heuristic to find as many paths as possible from each POI to each SINIK.
 * Each POI gives one vote for each sensor in its paths to the SINKs.
 * This is the PRIVATE version, that is used in heuristic GREEDY REUSE when path_increase_tolerance < 0, and GREEDY BREADTH when path_increase_tolerance = 0.
 */
int KCMC_Instance::greedy_reuse(int k, int m, std::unordered_set<int> &inactive_sensors, std::unordered_map<int, int> *visited_sensors, int path_increase_tolerance) {

    // Local buffers
    int num_paths, paths_found, path_end, a_poi, active_covering_sensors, add_sensor, pre_k_cov_sensors;
    std::vector<int> inv_frequency_array(this->num_sensors), predecessors(this->num_sensors);

    // First we clear out the output buffer
    // The visited_sensors map will be populated by the m_connectivity_greedy method with the number of paths each sensor is part of.
    visited_sensors->clear();

    // We run M-connectivity with an M that is infinity (or a very large number)
    num_paths = this->m_connectivity_greedy(LARGE_NUMBER, inactive_sensors, visited_sensors, path_increase_tolerance);

    /* Then format the frequency graph as a vector for minimization, similar to the level-graph
     * This is called the *inverse frequency array* (IFA). It holds no values smaller than 1.
     * In the IFA, sensors that were not found by the m_connectivity_greedy method have value num_paths
     * In the IFA, sensors that were found by the m_connectivity_greedy method have value num_paths-(frequency)
     * This inversion is done so the minimization loop can still be used
     */
    std::fill(inv_frequency_array.begin(), inv_frequency_array.end(), num_paths);
    for (const auto &i : *visited_sensors) {inv_frequency_array[i.first] = num_paths - i.second;}

    // Prepare the set of "used" sensors and clear the map of visited sensors
    std::unordered_set<int> used_sensors, set_visited_sensors, final_inactive_sensors;
    visited_sensors->clear();

    // Run for each POI, returning at the first failure
    for (a_poi=0; a_poi < this->num_pois; a_poi++) {
        paths_found = 0;  // Clear the number of paths found for the POI
        used_sensors = inactive_sensors;  // Reset the set of used sensors for each POI

        // While there are still paths to be found
        while (paths_found < m) {
            std::fill(predecessors.begin(), predecessors.end(), UNVISITED_SENSOR);  // Reset the predecessors buffer

            // Find a path
            path_end = this->greedy_find_path(a_poi, used_sensors, inv_frequency_array, predecessors);

            // If the path ends in an invalid sensor, break the loop. Other POIs will fix it
            if (path_end == INVALID_PREVIOUS_POI) {break;}
            else {
                paths_found += 1;  // Count the newfound path
                // Unravel the path, marking each sensor in it as used
                while (path_end != INVALID_PREVIOUS_POI) {
                    used_sensors.insert(path_end);
                    vote(*visited_sensors, path_end);  // Get the complete frequency map of all used sensors
                    path_end = predecessors[path_end];
                    if (path_end == UNVISITED_SENSOR) {throw std::runtime_error("FORBIDDEN ADDRESS!");}
                }
            }
        }
    }

    // Run the ADD-KCOV procedure to ensure k-coverage
    pre_k_cov_sensors = (int)(visited_sensors->size());
    this->add_kcov(k, visited_sensors);
    return pre_k_cov_sensors;
}
int KCMC_Instance::greedy_reuse(int k, int m, std::unordered_set<int> &inactive_sensors, std::unordered_map<int, int> *visited_sensors) {
    return this->greedy_reuse(k, m, inactive_sensors, visited_sensors, -1);  // No tolerance for path increases
}
int KCMC_Instance::greedy_breadth(int k, int m, std::unordered_set<int> &inactive_sensors, std::unordered_map<int, int> *visited_sensors) {
    return this->greedy_reuse(k, m, inactive_sensors, visited_sensors, 0);  // Tolerance for non-increasing paths
}
