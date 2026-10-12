

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
    int poi, sentinel, num_path, path_covered_pois, best_path_num, best_path_coverage, best_path_length, pre_k_cov_sensors;
    std::vector<int> path;
    std::unordered_set<int> ignore_path, used_paths;
    std::vector<std::vector<int>> poi_paths, all_paths;
    std::unordered_map<int, int> required_connectivity, added_connectivity;

    // For each POI
    all_paths.clear();
    for (poi = 0; poi < this->num_pois; ++poi) {

        // Get the connectivity map for the POI - each required sensor, and to wich path each sensor belongs to (in the POI)
        sentinel = m_connectivity(m, poi, inactive_sensors, poi_paths);

        // Do the safety checks
        if (sentinel >= LARGE_NUMBER) {
            throw std::runtime_error("Failed to find " + std::to_string(m) + " paths for POI " + std::to_string((sentinel/LARGE_NUMBER)-1) + " (only found " + std::to_string(sentinel%LARGE_NUMBER) + ")");
        }
        if (sentinel != -1) {throw std::runtime_error("Failed to find " + std::to_string(m) + " paths! And we obtained no more information!");}

        // Add each path to the collection of all paths
        for (const auto &path : poi_paths) {
            all_paths.push_back(path);
        }
    }

    // Start the buffer of POIs that need connectivity (initially all POIs require full connectivity)
    for (poi = 0; poi < this->num_pois; ++poi) {required_connectivity[poi] = m;}

    // While there are still POIs that do not meet the m-connectivity requirement
    while (!required_connectivity.empty()) {
        added_connectivity.clear();

        // For every path returned
        for (num_path = 0; num_path < ((int)all_paths.size()); ++num_path) {
            if (isin(ignore_path, num_path)) {continue;}
            if (isin(used_paths, num_path)) {continue;}
            path = all_paths[num_path];

            // For every sensor in the path, add the number of POIs it covers (only POIs that still require connectivity)
            path_covered_pois = 0;
            for (const auto &sensor : path) {
                if (isin(inactive_sensors, sensor)) {throw std::runtime_error("Inactive sensor found in path!");}
                path_covered_pois += ((int)set_intersection(this->sensor_poi[sensor], required_connectivity).size());
            }

            // If the path does not adds any connectivity, add it to the list of ignored paths.
            // If it does add connectivity, add it to the list of candidate paths.
            if (path_covered_pois == 0) {ignore_path.insert(num_path);}
            else {added_connectivity[num_path] = path_covered_pois;}
        }

        // Get the best path based on the number of POIs it covers, and its length (prefer shorter paths in case of ties)
        best_path_num = -1;
        best_path_coverage = -1;
        best_path_length = LARGE_NUMBER;
        for (const auto &entry : added_connectivity) {
            num_path = entry.first;
            path_covered_pois = entry.second;
            path = all_paths[num_path];

            // If the current path has better coverage, or, having the same coverage, is shorter, pick it
            // The first path found will always be picked
            if (
                (path_covered_pois > best_path_coverage)
             || (
                    (path_covered_pois == best_path_coverage)
                 && (((int)path.size()) < best_path_length)
                )
            ) {
                best_path_num = num_path;
                best_path_coverage = path_covered_pois;
                best_path_length = ((int)path.size());
            }
        }
        if (best_path_num == -1) {throw std::runtime_error("No valid path found!");}

        // Mark the best path as already included in the result
        ignore_path.insert(best_path_num);
        used_paths.insert(best_path_num);

        // Update the required connectivity based on the best path
        path = all_paths[best_path_num];
        for (const auto &sensor : path) {
            for (const auto &poi : this->sensor_poi[sensor]) {

                // If the POI requires connectivity, update it
                if (isin(required_connectivity, poi)) {
                    required_connectivity[poi] -= 1;
                }

                // Remove POIs that no longer require connectivity
                if (required_connectivity[poi] <= 0) {required_connectivity.erase(poi);}
            }
        }
    }
    // Here, every POI has the required connectivity!
    // Add all the sensors of every used path to the solution
    solution.clear();
    for (const auto &path_num : used_paths) {
        for (const auto &sensor : all_paths[path_num]) {
            solution.insert(sensor);
        }
    }

    // Run the ADD-KCOV procedure to ensure k-coverage
    pre_k_cov_sensors = ((int)(solution.size()));
    this->add_kcov(k, solution);
    return pre_k_cov_sensors;
}
