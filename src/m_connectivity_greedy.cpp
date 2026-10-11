

// STDLib dependencies
#include <sstream>    // ostringstream
#include <queue>      // priority_queue

// Dependencies from this package
#include "kcmc_instance.h"  // KCMC Instance class headers


/** A* (A-STAR) PATHFINDING ALGORITHM
 */
int KCMC_Instance::greedy_find_path(const int poi_number, std::unordered_set<int> &used_sensors, std::vector<int> &level_graph, std::vector<int> &predecessors) {

    // Local buffers
    int i_sensor;
    std::priority_queue<LevelNode, std::vector<LevelNode>, CompareLevelNode> queue;

    // Prepare a queue with each active unused sensor that covers the POI
    // Add each of those sensors to the predecessors map having "INVALID_PREVIOUS_POI" as the predecessor, meaning "the POI is the predecessor"
    for (const int &a_sensor : this->poi_sensor[poi_number]) {
        if (!isin(used_sensors, a_sensor)) {
            queue.push({a_sensor, level_graph[a_sensor]});
            predecessors[a_sensor] = INVALID_PREVIOUS_POI;
        }
    }

    // Iterate until the queue is empty
    while (!queue.empty()) {
        // Get the top sensor in the queue (lowest level) and visit it
        i_sensor = queue.top().index;
        queue.pop();

        // If the sensor is neighbor of a sink, return the sensor as the beginning of the path
        if (isin(this->sensor_sink, i_sensor)) {return i_sensor;}

        // For each neighbor of the top sensor, if the neighbor has not been used or visited yet,
        // Add the unvisited active neighbor to the queue and the top sensor as its predecessor
        for (const int &neighbor : this->sensor_sensor[i_sensor]) {
            if ((!isin(used_sensors, neighbor)) && (predecessors[neighbor] == UNVISITED_SENSOR)){
                queue.push({neighbor, level_graph[neighbor]});
                predecessors[neighbor] = i_sensor;
                // If the neighbor is sink-adjacent, we can return it directly
                if (isin(this->sensor_sink, neighbor)) {return neighbor;}
            }
        }
    }

    // If we got here, there is no possible path :(
    return INVALID_PREVIOUS_POI;
}


/** M-Connectivity by the Greedy Heuristic
 * Gets the list of used sensors needed to get m-connectivity by the GREEDY heuristic.
 */
int KCMC_Instance::m_connectivity_greedy(const int m, std::unordered_set<int> &inactive_sensors, std::unordered_map<int, int> *all_used_sensors, const int path_increase_tolerance) {

    // Clear the set of active sensors
    all_used_sensors->clear();

    // Base case
    if (m < 1){return INVALID_PREVIOUS_POI;}

    // Create the level graph
    std::vector<int> level_graph(this->num_sensors, INVALID_PREVIOUS_POI);
    this->compute_level_graph(level_graph, inactive_sensors);

    // Prepare the set of "used" sensors
    std::unordered_set<int> poi_used_sensors;

    // Create a loop control flag and pointer buffers
    int paths_found, path_end, current_path_step, a_poi, total_paths_found=0, path_length=0, longest_path_length=0;  // the lengths must be initialized as 0
    std::vector<int> predecessors(this->num_sensors, UNVISITED_SENSOR);

    // Run for each POI, returning at the first failure
    for (a_poi=0; a_poi < this->num_pois; a_poi++) {
        paths_found = 0;  // Clear the number of paths found for the POI
        poi_used_sensors = std::unordered_set<int>(inactive_sensors.begin(), inactive_sensors.end());  // Reset the set of used sensors for each POI

        // While there are still paths to be found
        while ((paths_found < m) || (path_increase_tolerance >= 0)) {
            std::fill(predecessors.begin(), predecessors.end(), UNVISITED_SENSOR);  // Reset the predecessors buffer

            // Find a path
            path_end = this->greedy_find_path(a_poi, poi_used_sensors, level_graph, predecessors);

            // If the path ends in an invalid sensor, return the failure.
            if (path_end == INVALID_PREVIOUS_POI) {
                return ((1+a_poi)*LARGE_NUMBER) + paths_found;  // Encoded the two ints. We must not have more than a LARGE_NUMBER of POIs!

            // If success, count the path and mark all the sensors with predecessors as "used"
            } else {

                // If we already found the required number of paths for this POI, we are in a BREADTH iteration
                if ((paths_found >= m) && (path_increase_tolerance >= 0)) {

                    // Unravel the path to get its size
                    path_length = 0;  // Initialize the path length counter
                    current_path_step = path_end;  // Initialize the current path step to the end of the path
                    while (current_path_step != INVALID_PREVIOUS_POI) {
                        path_length += 1;  // Count the current step in the path
                        current_path_step = predecessors[current_path_step];
                    }

                    // If the size is larger than the longest path length + path_increase_tolerance,
                    // break here and now
                    if (path_length > (longest_path_length + path_increase_tolerance)) {
                        break;
                    }
                }

                // If we got here, we must include the path in our solution
                paths_found += 1;  // Count the newfound path
                total_paths_found += 1;  // Count in the absolute total
                path_length = 0;  // Initialize the path length counter
                current_path_step = path_end;  // Initialize the current path step to the end of the path
                while (current_path_step != INVALID_PREVIOUS_POI) {
                    if (current_path_step == UNVISITED_SENSOR) {throw std::runtime_error("FORBIDDEN ADDRESS!");}

                    poi_used_sensors.insert(current_path_step);
                    vote(*all_used_sensors, current_path_step);  // Add a vote to the current sensor

                    // update the path length
                    path_length += 1;
                    current_path_step = predecessors[current_path_step];
                }

                // Update the longest path length if the current path is longer, but onlyu if we are in the first m paths
                if (paths_found <= m) {
                    longest_path_length = std::max(longest_path_length, path_length);
                }
            }
        }

        // ERROR CASE - return the POI that has insuficient paths, encoded with the number of paths
        if (paths_found < m) {return ((1+a_poi)*LARGE_NUMBER) + paths_found;}
    }

    // Success in each and every POI!
    return INVALID_PREVIOUS_POI;
}
int KCMC_Instance::m_connectivity_greedy(const int m, std::unordered_set<int> &inactive_sensors, std::unordered_map<int, int> *all_used_sensors) {
    return this->m_connectivity_greedy(m, inactive_sensors, all_used_sensors, -1);  // No tolerance for path increases
}
int KCMC_Instance::m_connectivity_greedy(const int m, std::unordered_set<int> &inactive_sensors, std::unordered_set<int> *all_used_sensors) {
    // Run with a map
    std::unordered_map<int, int> buffer;
    int result = this->m_connectivity_greedy(m, inactive_sensors, &buffer);
    // adjust the result
    if (result < LARGE_NUMBER) {result = INVALID_PREVIOUS_POI;}  // WE MUST HAVE FEWER THAN A LARGE_NUMBER OF PATHS!
    // Revert back to set
    all_used_sensors->clear();
    for (const auto i : buffer) {all_used_sensors->insert(i.first);}
    return result;
}
