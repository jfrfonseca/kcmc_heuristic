

// STDLib dependencies
#include <sstream>    // ostringstream
#include <queue>      // queue
#include <iostream>   // cin, cout, endl
#include <iomanip>    // setfill, setw
#include <algorithm>  // sort

// Dependencies from this package
#include "kcmc_instance.h"  // KCMC Instance class headers


/** DINITZ DKOV
 * For each POI:
 *     - Use the dinitz algorithm to get the sensor-disjoint paths that yield max-flow between the POI and the SINK.
 *     - Sort the paths by length (shortest first)
 *     - Add the M shortest paths to the solution
 * Use AdKCov to add sensors to the solution until every POI has enough coverage.
 * The sensors in the paths chosen, plus the added sensors for k-coverage, are the heuristically-chosen minimal set.
 */
int KCMC_Instance::dinitz_dkov(const int k, const int m, std::unordered_set<int> &inactive_sensors, std::unordered_set<int> &solution) {

    // Local buffers
    int poi, sentinel, pre_k_cov_sensors, num_path;
    std::vector<int> path;
    std::vector<std::vector<int>> poi_paths;
    std::vector<LevelNode> poi_path_lengths;

    // For each POI
    solution.clear();
    for (poi = 0; poi < this->num_pois; ++poi) {
        poi_path_lengths.clear();

        // Get the connectivity map for the POI - each required sensor, and to wich path each sensor belongs to (in the POI)
        sentinel = m_connectivity(m, poi, inactive_sensors, poi_paths);

        // Do the safety checks
        if (sentinel >= LARGE_NUMBER) {
            throw std::runtime_error(this->key() + " DINITZ DKOV Failed to find " + std::to_string(m) + " paths for POI " + std::to_string((sentinel/LARGE_NUMBER)-1) + " (only found " + std::to_string(sentinel%LARGE_NUMBER) + ")");
        }
        if (sentinel != -1) {throw std::runtime_error(this->key() + " DINITZ DKOV Failed to find " + std::to_string(m) + " paths! And we obtained no more information!");}
        if (poi_paths.size() < m) {throw std::runtime_error(this->key() + " DINITZ DKOV Failed to find " + std::to_string(m) + " paths! And we obtained no more information!");}

        // Make a LevelNode mapping the length of the path to its index and sort the paths by increasing order of length
        for (num_path = 0; num_path < poi_paths.size(); ++num_path) {
            poi_path_lengths.push_back(LevelNode{num_path, static_cast<int>(poi_paths[num_path].size())});
        }
        std::sort(poi_path_lengths.begin(), poi_path_lengths.end(), SortLevelNode());

        // Add the sensors from the first m shortest paths to the solution
        for (num_path=0; num_path<m; ++num_path) {
            for (const auto &sensor : poi_paths[poi_path_lengths[num_path].index]) {
                solution.insert(sensor);
            }
        }
    }

    // Run the ADD-KCOV procedure to ensure k-coverage
    pre_k_cov_sensors = ((int)(solution.size()));
    this->add_kcov(k, inactive_sensors, solution);
    return pre_k_cov_sensors;
}


/** DINITZ REUSE
 * For each POI:
 *     - Use the dinitz algorithm to get the sensor-disjoint paths that yield max-flow between the POI and the SINK.
 *     - For every path found between the POI and the SINK:
 *         - Store the path for later usage
 *             - For every sensor in the path
 *                 - For every POI covered by that sensor
 *                     - Add the path to the set of paths that cover this POI
 *                     - Add the POI to the set of POIs covered by this path
 * Start an array listing the chosen paths that provide connectivity for each POI.
 * For each POI with fewer than m chosen paths to provide connectivity for it:
 *     - For each path that is not in the current solution:
 *         - If it convers the POI and is node-disjoint with all the chosen paths that provide connectivity for this POI:
 *             - Add one vote to the path
 *     - Select the path with the most votes and add it to the solution
 *         - In case of tie, choose the one that adds the fewest new sensors
 *             - In case of another tie, choose the shortest path
 *                 - In case of yet another tie, choose arbitrarily
 *     - Add the most voted path to the list of chosen paths of each POI that voted for it
 * Use AdKCov to add sensors to the solution until every POI has enough coverage.
 * The sensors in the paths chosen, plus the added sensors for k-coverage, are the heuristically-chosen minimal set.
 */
int KCMC_Instance::dinitz_reuse(const int k, const int m, std::unordered_set<int> &inactive_sensors, std::unordered_set<int> &solution) {

    // Local buffers
    bool connectivity_is_sufficient, is_node_disjoint;
    int poi, sentinel, pre_k_cov_sensors, num_path, a_sensor,
        best_path_idx, best_path_votes, best_path_added_sensors, best_path_length;
    std::vector<std::vector<int>> poi_paths, all_paths;
    std::unordered_map<int, std::unordered_set<int>> current_connectivity, poi_is_covered_by_paths, path_cover_pois, votes;
    std::unordered_set<int> used_paths;

    // For each POI
    all_paths.clear();
    for (poi = 0; poi < this->num_pois; ++poi) {

        // Get the connectivity map for the POI - each required sensor, and to wich path each sensor belongs to (in the POI)
        sentinel = m_connectivity(m, poi, inactive_sensors, poi_paths);

        // Do the safety checks
        if (sentinel >= LARGE_NUMBER) {
            throw std::runtime_error(this->key() + " DINITZ REUSE Failed to find " + std::to_string(m) + " paths for POI " + std::to_string((sentinel/LARGE_NUMBER)-1) + " (only found " + std::to_string(sentinel%LARGE_NUMBER) + ")");
        }
        if (sentinel != -1) {throw std::runtime_error(this->key() + " DINITZ REUSE Failed to find " + std::to_string(m) + " paths! And we obtained no more information!");}
        if (poi_paths.size() < m) {throw std::runtime_error(this->key() + " DINITZ REUSE Failed to find " + std::to_string(m) + " paths! And we obtained no more information!");}

        // Add each path to the list of all paths
        for (num_path = 0; num_path < poi_paths.size(); ++num_path) {
            all_paths.push_back(poi_paths[num_path]);
            sentinel = all_paths.size() - 1;
            path_cover_pois[sentinel].insert(poi);
            poi_is_covered_by_paths[poi].insert(sentinel);

            // For every sensor in the path, note which POIs it covers
            for (a_sensor = 0; a_sensor < poi_paths[num_path].size(); ++a_sensor) {
                for (const int a_poi : this->sensor_poi[poi_paths[num_path][a_sensor]]) {

                    // Add the a_poi to the list of POIs covered by this path
                    path_cover_pois[sentinel].insert(a_poi);

                    // Add the path to the list of paths that cover this a_poi
                    poi_is_covered_by_paths[a_poi].insert(sentinel);
                }
            }

            // Sanity checks. This path must cover the poi that found it and vice versa
            if (!isin(path_cover_pois[sentinel], poi)) {throw std::runtime_error(this->key() + " DINITZ REUSE Path " + std::to_string(sentinel) + " does not cover POI " + std::to_string(poi));}
            if (!isin(poi_is_covered_by_paths[poi], sentinel)) {throw std::runtime_error(this->key() + " DINITZ REUSE POI " + std::to_string(poi) + " is not covered by path " + std::to_string(sentinel));}
        }
    }

    // Prepare the buffers
    solution.clear();
    used_paths.clear();
    connectivity_is_sufficient = false;

    // While we still don't have enough connectivity for all POIs
    while (!connectivity_is_sufficient) {

        // Reset the voting tally and pointers
        best_path_idx = -1;
        best_path_votes = 1;
        best_path_length = LARGE_NUMBER;
        best_path_added_sensors = LARGE_NUMBER;
        connectivity_is_sufficient = true;  // Will be set to false if this is not true
        votes.clear();  // Reset the votes for each iteration of the while loop

        // For each POI that doesn't have enough connectivity
        for (poi = 0; poi < this->num_pois; ++poi) {
            if (current_connectivity[poi].size() < m) {
                connectivity_is_sufficient = false;  // A single POI with insufficient connectivity makes this variable false

                // For each path that covers this POI and is not yet used in the solution
                for (const int num_path : poi_is_covered_by_paths[poi]) {
                    if (!isin(used_paths, num_path)) {

                        // If the path is node-disjoint with all the paths that currently provide connectivity for this POI
                        is_node_disjoint = true;
                        for (const int num_poi_path : current_connectivity[poi]) {
                            if (set_intersection(all_paths[num_poi_path], all_paths[num_path]).size() > 0) {
                                is_node_disjoint = false;
                                break;
                            }
                        }
                        if (!is_node_disjoint) {continue;}

                        // Add one vote for this path, noting the POI it helps cover
                        push(votes, num_path, poi);
                    }
                }
            }
        }
        if (connectivity_is_sufficient) {break;}  // Exit the while loop if all POIs have sufficient connectivity

        // Get the best path
        for (num_path = 0; num_path < (int)all_paths.size(); ++num_path) {
            if (isin(used_paths, num_path)) {continue;}  // Skip paths that are already used

            // If either the current path had more votes than the current best,
            // Or the same number of votes but adds fewer sensors,
            // Or the same number of votes and adds the same number of sensors but is shorter,
            // Choose it as the best path
            if (
                   ((int)votes[num_path].size()) > best_path_votes
                || ((((int)votes[num_path].size()) == best_path_votes) && (((int)set_diff(all_paths[num_path], solution).size()) < best_path_added_sensors))
                || ((((int)votes[num_path].size()) == best_path_votes) && (((int)set_diff(all_paths[num_path], solution).size()) == best_path_added_sensors) && (((int)all_paths[num_path].size()) < best_path_length))
            ) {
                best_path_idx = num_path;
                best_path_votes = (int)votes[num_path].size();
                best_path_length = (int)all_paths[num_path].size();
                best_path_added_sensors = ((int)set_diff(all_paths[num_path], solution).size());
            }
        }
        if (best_path_idx == -1) {throw std::runtime_error(this->key() + " DINITZ REUSE No valid path found");}

        // Add the best path to the solution
        used_paths.insert(best_path_idx);
        for (const int a_sensor : all_paths[best_path_idx]) {solution.insert(a_sensor);}

        // Increase the connectivity of all POIs that voted for the path
        for (const int a_poi : votes[best_path_idx]) {push(current_connectivity, a_poi, best_path_idx);}
    }

    // Run the ADD-KCOV procedure to ensure k-coverage
    pre_k_cov_sensors = ((int)(solution.size()));
    this->add_kcov(k, inactive_sensors, solution);
    return pre_k_cov_sensors;
}


/** DINITZ BREADTH
 * For each POI:
 *     - Use the dinitz algorithm to get the sensor-disjoint paths that yield max-flow between the POI and the SINK.
 *     - For every path found between the POI and the SINK:
 *         - Store the path for later usage
 *             - For every sensor in the path
 *                 - For every POI covered by that sensor
 *                     - Add the path to the set of paths that cover this POI
 *                     - Add the POI to the set of POIs covered by this path
 * Start an array listing the chosen paths that provide connectivity for each POI.
 * For each POI with fewer than m chosen paths to provide connectivity for it:
 *     - For each path that is not in the current solution:
 *         - If it convers the POI and is node-disjoint with all the chosen paths that provide connectivity for this POI:
 *             - Add one vote to the path
 *     - Select the path with the most votes and add it to the solution
 *         - In case of tie, choose the one that adds the fewest new sensors
 *             - In case of another tie, choose the shortest path
 *                 - In case of yet another tie, choose arbitrarily
 *     - Add the most voted path to the list of chosen paths of each POI that voted for it
 * Count the number of sensors in the solution - this is size S
 * For every path not yet in the solution
 *     - If the number of sensors in the solution got to 2*S, stop the iteration
 *     - If the path is no longer than the shortest path already in the solution
 *         - Add the path to the solution
 * Use AdKCov to add sensors to the solution until every POI has enough coverage.
 * The sensors in the paths chosen, plus the added sensors for k-coverage, are the heuristically-chosen minimal set.
 */
int KCMC_Instance::dinitz_breadth(const int k, const int m, std::unordered_set<int> &inactive_sensors, std::unordered_set<int> &solution) {

    // Local buffers
    bool connectivity_is_sufficient, is_node_disjoint;
    int poi, sentinel, pre_k_cov_sensors, num_path, a_sensor,
        best_path_idx, best_path_votes, best_path_added_sensors, best_path_length,
        shortest_chosen_path_length, starting_size;
    std::vector<std::vector<int>> poi_paths, all_paths;
    std::unordered_map<int, std::unordered_set<int>> current_connectivity, poi_is_covered_by_paths, path_cover_pois, votes;
    std::unordered_set<int> used_paths;

    // For each POI
    all_paths.clear();
    for (poi = 0; poi < this->num_pois; ++poi) {

        // Get the connectivity map for the POI - each required sensor, and to wich path each sensor belongs to (in the POI)
        sentinel = m_connectivity(m, poi, inactive_sensors, poi_paths);

        // Do the safety checks
        if (sentinel >= LARGE_NUMBER) {
            throw std::runtime_error(this->key() + " DINITZ BREADTH Failed to find " + std::to_string(m) + " paths for POI " + std::to_string((sentinel/LARGE_NUMBER)-1) + " (only found " + std::to_string(sentinel%LARGE_NUMBER) + ")");
        }
        if (sentinel != -1) {throw std::runtime_error(this->key() + " DINITZ BREADTH Failed to find " + std::to_string(m) + " paths! And we obtained no more information!");}
        if (poi_paths.size() < m) {throw std::runtime_error(this->key() + " DINITZ BREADTH Failed to find " + std::to_string(m) + " paths! And we obtained no more information!");}

        // Add each path to the list of all paths
        for (num_path = 0; num_path < poi_paths.size(); ++num_path) {
            all_paths.push_back(poi_paths[num_path]);
            sentinel = all_paths.size() - 1;
            path_cover_pois[sentinel].insert(poi);
            poi_is_covered_by_paths[poi].insert(sentinel);

            // For every sensor in the path, note which POIs it covers
            for (a_sensor = 0; a_sensor < poi_paths[num_path].size(); ++a_sensor) {
                for (const int a_poi : this->sensor_poi[poi_paths[num_path][a_sensor]]) {

                    // Add the a_poi to the list of POIs covered by this path
                    path_cover_pois[sentinel].insert(a_poi);

                    // Add the path to the list of paths that cover this a_poi
                    poi_is_covered_by_paths[a_poi].insert(sentinel);
                }
            }

            // Sanity checks. This path must cover the poi that found it and vice versa
            if (!isin(path_cover_pois[sentinel], poi)) {throw std::runtime_error(this->key() + " DINITZ BREADTH Path " + std::to_string(sentinel) + " does not cover POI " + std::to_string(poi));}
            if (!isin(poi_is_covered_by_paths[poi], sentinel)) {throw std::runtime_error(this->key() + " DINITZ BREADTH POI " + std::to_string(poi) + " is not covered by path " + std::to_string(sentinel));}
        }
    }

    // Prepare the buffers
    solution.clear();
    used_paths.clear();
    connectivity_is_sufficient = false;
    shortest_chosen_path_length = LARGE_NUMBER;  // Amongst all POIs

    // While we still don't have enough connectivity for all POIs
    while (!connectivity_is_sufficient) {

        // Reset the voting tally and pointers
        best_path_idx = -1;
        best_path_votes = 1;
        best_path_length = LARGE_NUMBER;
        best_path_added_sensors = LARGE_NUMBER;
        connectivity_is_sufficient = true;  // Will be set to false if this is not true
        votes.clear();  // Reset the votes for each iteration of the while loop

        // For each POI that doesn't have enough connectivity
        for (poi = 0; poi < this->num_pois; ++poi) {
            if (current_connectivity[poi].size() < m) {
                connectivity_is_sufficient = false;  // A single POI with insufficient connectivity makes this variable false

                // For each path that covers this POI and is not yet used in the solution
                for (const int num_path : poi_is_covered_by_paths[poi]) {
                    if (!isin(used_paths, num_path)) {

                        // If the path is node-disjoint with all the paths that currently provide connectivity for this POI
                        is_node_disjoint = true;
                        for (const int num_poi_path : current_connectivity[poi]) {
                            if ((num_poi_path != num_path) && (set_intersection(all_paths[num_poi_path], all_paths[num_path]).size() > 0)) {
                                is_node_disjoint = false;
                                break;
                            }
                        }
                        if (!is_node_disjoint) {continue;}

                        // Add one vote for this path, noting the POI it helps cover
                        push(votes, num_path, poi);
                    }
                }
            }
        }
        if (connectivity_is_sufficient) {break;}  // Exit the while loop if all POIs have sufficient connectivity

        // Get the best path
        for (num_path = 0; num_path < (int)all_paths.size(); ++num_path) {
            if (isin(used_paths, num_path)) {continue;}  // Skip paths that are already used

            // If either the current path had more votes than the current best,
            // Or the same number of votes but adds fewer sensors,
            // Or the same number of votes and adds the same number of sensors but is shorter,
            // Choose it as the best path
            if (
                   ((int)votes[num_path].size()) > best_path_votes
                || ((((int)votes[num_path].size()) == best_path_votes) && (((int)set_diff(all_paths[num_path], solution).size()) < best_path_added_sensors))
                || ((((int)votes[num_path].size()) == best_path_votes) && (((int)set_diff(all_paths[num_path], solution).size()) == best_path_added_sensors) && (((int)all_paths[num_path].size()) < best_path_length))
            ) {
                best_path_idx = num_path;
                best_path_votes = (int)votes[num_path].size();
                best_path_length = (int)all_paths[num_path].size();
                best_path_added_sensors = ((int)set_diff(all_paths[num_path], solution).size());

                // Update the shortest chosen path length if the current best path is shorter
                if (best_path_length < shortest_chosen_path_length) {shortest_chosen_path_length = best_path_length;}
            }
        }
        if (best_path_idx == -1) {throw std::runtime_error(this->key() + " DINITZ BREADTH No valid path found");}

        // Add the best path to the solution
        used_paths.insert(best_path_idx);
        for (const int a_sensor : all_paths[best_path_idx]) {solution.insert(a_sensor);}

        // Increase the connectivity of all POIs that voted for the path
        for (const int a_poi : votes[best_path_idx]) {push(current_connectivity, a_poi, best_path_idx);}
    }

    // Add all paths not yet in the solution that are no longer than the shortest chosen path
    starting_size = (int)solution.size();
    for (num_path = 0; num_path < (int)all_paths.size(); ++num_path) {
        if ((!isin(used_paths, num_path)) && (((int)all_paths[num_path].size()) <= shortest_chosen_path_length)) {
            used_paths.insert(num_path);
            for (const int a_sensor : all_paths[num_path]) {solution.insert(a_sensor);}
            if (((int)solution.size()) >= (2*starting_size)){break;}
        }
    }

    // Run the ADD-KCOV procedure to ensure k-coverage
    pre_k_cov_sensors = ((int)(solution.size()));
    this->add_kcov(k, inactive_sensors, solution);
    return pre_k_cov_sensors;
}
