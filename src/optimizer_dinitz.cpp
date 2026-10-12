

// STDLib dependencies
#include <sstream>    // ostringstream
#include <queue>      // queue
#include <iostream>   // cin, cout, endl
#include <iomanip>    // setfill, setw

// Dependencies from this package
#include "kcmc_instance.h"  // KCMC Instance class headers


/** DINITZ DKOV
 * Use the dinitz algorithm to get sensors enough for m-connectivity, and then add sensors to also achieve k-coverage.
 */
int KCMC_Instance::dinitz_dkov(const int k, const int m, std::unordered_set<int> &inactive_sensors, std::unordered_set<int> &solution) {

    // Local buffers
    int sentinel, pre_k_cov_sensors;

    // We run the greedy M-connectivity normally
    solution.clear();
    sentinel = this->m_connectivity(m, inactive_sensors, solution);
    if (sentinel >= LARGE_NUMBER) {
        throw std::runtime_error("Failed to find " + std::to_string(m) + " paths for POI " + std::to_string((sentinel/LARGE_NUMBER)-1) + " (only found " + std::to_string(sentinel%LARGE_NUMBER) + ")");
    }
    if (sentinel != -1) {throw std::runtime_error("Failed to find " + std::to_string(m) + " paths! And we obtained no more information!");}

    // Run the ADD-KCOV procedure to ensure k-coverage
    pre_k_cov_sensors = ((int)(solution.size()));
    this->add_kcov(k, solution);
    return pre_k_cov_sensors;
}


/** DINITZ REUSE
 * Use the dinitz algorithm to get the sensor-disjoint paths that yield max-flow.
 * Then, while there are still POIs that do not meet the m-connectivity requirement:
 *     - For each path that was not included in the result
 *         - Count how many POIs (that still don't meet the m-connectivity requirement) that are covered by the path
 *         - Select the path that covers the most POIs (that still don't meet the m-connectivity requirement) and add it to the solution
 *             - If there are two candidates that cover the same highest number of POIs, select the shortest path
 *                 - If there is still a tie, choose arbitrarily
 * The sensors in the paths chosen are the heuristically-chosen minimal set.
 */
int KCMC_Instance::dinitz_reuse(const int k, const int m, std::unordered_set<int> &inactive_sensors, std::unordered_set<int> &solution) {

    // Local buffers
    int poi, sentinel, pre_k_cov_sensors;
    std::vector<std::vector<int>> poi_paths, all_paths;

    // For each POI
    for (poi = 0; poi < this->num_pois; ++poi) {

        // Get the connectivity map for the POI - each required sensor, and to wich path each sensor belongs to (in the POI)
        sentinel = m_connectivity(m, poi, inactive_sensors, poi_paths);

        // Do the safety checks
        if (sentinel >= LARGE_NUMBER) {
            throw std::runtime_error("Failed to find " + std::to_string(m) + " paths for POI " + std::to_string((sentinel/LARGE_NUMBER)-1) + " (only found " + std::to_string(sentinel%LARGE_NUMBER) + ")");
        }
        if (sentinel != -1) {throw std::runtime_error("Failed to find " + std::to_string(m) + " paths! And we obtained no more information!");}

    }

    // For every path returned, see how many POIs it covers


    // Run the ADD-KCOV procedure to ensure k-coverage
    pre_k_cov_sensors = ((int)(solution.size()));
    this->add_kcov(k, solution);
    return pre_k_cov_sensors;
}
