/** KCMC_INSTANCE_HANDLING.cpp
 * Implementation of non-payload services of the KCMC instance object headers
 * Jose F. R. Fonseca
 */


// STDLib dependencies
#include <limits>     // getline
#include <string>     // string
#include <iostream>   // cout, endl, cerr
#include <sstream>    // ostringstream
#include <random>     // mt19937, uniform_real_distribution
#include <algorithm>  // std::find
#include <queue>      // priority_queue

// Dependencies from this package
#include "kcmc_instance.h"  // KCMC Instance class headers


/* #####################################################################################################################
 * UTILITY FUNCTIONS
 */


// Waits for the user to give an input before continuing
void wait_user() {
  std::string s;
  std::getline(std::cin, s);
}


// Handles output signals (like Ctrl+C) to exit gracefully
void exit_signal_handler(int signal) {
   std::cerr << "Interrupt signal (" << signal << ") received. Exiting gracefully..." << std::endl;
   exit(0);
}

// Stringfy a value with leading spaces up to a given length
std::string stringfy(int length, int value) {
    std::string message = std::to_string(value);
    std::string indent(length - message.length(), ' ');

    return indent + message;
}


/* ISIN (IS IN)
 * A method to determine if a given value is in a given set of values, overloaded for maximum re-usability
 */
bool isin(std::unordered_map<int, std::unordered_set<int>> &ref, const int item){return ref.find(item) != ref.end();}
bool isin(std::unordered_map<int, int> &ref, const int item){return ref.find(item) != ref.end();}
bool isin(std::unordered_set<int> &ref, const int item){return ref.find(item) != ref.end();}
bool isin(std::unordered_set<std::string> &ref, const std::string &item){return ref.find(item) != ref.end();}  // CANNOT BE A POINTER TO THE SET!
bool isin(std::vector<int> &ref, const int item){return std::find(ref.begin(), ref.end(), item) != ref.end();}
bool isin(std::vector<int> *ref, const int item){return std::find(ref->begin(), ref->end(), item) != ref->end();}


/* SET DIFF
 * Returns the set that is the difference between the given sets
 */
std::unordered_set<int> set_diff(const std::unordered_set<int> &left, const std::unordered_set<int> &right) {
    auto rightend = right.end();
    std::unordered_set<int> remainder;
    for (const int &item : left) {
        if (right.find(item) == rightend) { // If item NOT IN right side
            remainder.insert(item);
        }
    }
    return remainder;
}


/* SET INTERSECTION
 * Returns the set that is the intersection of the given sets
 */
std::unordered_set<int> set_intersection(const std::unordered_set<int> &left, const std::unordered_set<int> &right) {
    std::unordered_set<int> intersection;
    for (const int &item : left) {
        if (right.find(item) != right.end()) { // If item IN right side
            intersection.insert(item);
        }
    }
    return intersection;
}


/* DISTANCE
 * Distance between two components, overloaded for maximum re-usability
 */
double distance(Placement source, Placement target){
    /** Euclidean distance */
    return sqrt(pow((source.x - target.x), 2.0)
                 + pow((source.y - target.y), 2.0));
}


/* PUSH
 * Adds and element to an HashMap of Unordered Sets
 */
void push(std::unordered_map<int, std::unordered_set<int>> &buffer, const int source, const int target){
    if (isin(buffer, source)){buffer[source].insert(target);}
    else {
        std::unordered_set<int> new_set;
        new_set.insert(target);
        buffer[source] = new_set;
    }
}


/* VOTE
 * Adds a vote to an element in an unordered map. Adds the element to the map if not there
 */
void vote(std::unordered_map<int, int> &buffer, const int target, const int value){
    if (isin(buffer, target)){buffer[target] = buffer[target] + value;}
    else {buffer[target] = value;}
}
void vote(std::unordered_map<int, int> &buffer, const int target) {vote(buffer, target, 1);}


void setify(std::unordered_set<int> &target, int size, int source[], int reference) {
    target.clear();
    for (int i=0; i<size; i++) {if (source[i] == reference) {target.insert(i);}}
}


void setify(std::unordered_set<int> &target, std::unordered_map<int, int> reference) {
    target.clear();
    for (const auto &i : reference) {target.insert(i.first);}
}


/* #####################################################################################################################
 * KCMC-PROBLEM K-COVERAGE METHODS
 */


/** K-Coverage Validator
 * Very trivial k-coverage validator
 */
int KCMC_Instance::k_coverage(const int k, std::unordered_set<int> &inactive_sensors) {
    // Base case
    if (k < 1){return -1;}

    // Start buffers
    unsigned long active_coverage;

    // For each POI, count its coverage, returning and error if insufficient
    for (int n_poi=0; n_poi < this->num_pois; n_poi++) {
        active_coverage = set_diff(this->poi_sensor[n_poi], inactive_sensors).size();
        if (active_coverage < k) {
            return (n_poi*1000000)+(int)(active_coverage);
        }
    }

    // Success in each and every POI!
    return -1;
}


/** K-Coverage Validator that also returns the used sensors in k coverage
 * Very trivial k-coverage validator
 */
int KCMC_Instance::k_coverage(const int k, std::unordered_set<int> &inactive_sensors, std::unordered_set<int> &solution) {

    // Base case
    if (k < 1){return -1;}

    // Start buffers
    std::unordered_set<int> poi_solution;
    unsigned long active_coverage;

    // For each POI, count its coverage, returning and error if insufficient. Also note all used sensors
    solution.clear();
    for (int n_poi=0; n_poi < this->num_pois; n_poi++) {
        poi_solution = set_diff(this->poi_sensor[n_poi], inactive_sensors);
        active_coverage = poi_solution.size();
        solution = set_merge(solution, poi_solution);
        if (active_coverage < k) {
            return (n_poi*1000000)+(int)(active_coverage);
        }
    }

    // Success in each and every POI!
    return -1;
}


/** LEVEL-GRAPH ALGORITM
 * Sets in each active sensor its level, that is the lowest distance to a sink using only active sensors
 */
int KCMC_Instance::compute_level_graph(std::vector<int> &level_graph, std::unordered_set<int> &inactive_sensors) {
    /* Sets the lowest distance in hops from each active sensor to the nearest sink using only active sensors
     */

    // Reused buffers
    int level = 0;
    std::unordered_set<int> visited, work_set, next_set;

    // Get the set of active neighbors of sinks. Set each neighbor's level to 0
    for (const auto &a_sink : this->sink_sensor) {
       for (const int &neighbor : a_sink.second) {
           if (!isin(inactive_sensors, neighbor)) {
               level_graph[neighbor] = 0;
               work_set.insert(neighbor);
           }
       }
    }

    // Mark all sensors in the set as visited, as well as the inactive sensors
    visited = set_merge(inactive_sensors, work_set);

    // While there are still sensors to visit, find and visit them and set their level
    while (!work_set.empty()) {
        // advance the level
        level++;

        // update the next set and the levels of the sensors in the work set
        next_set.clear();
        for (const int &source : work_set) {
            for (const int &neighbor : this->sensor_sensor[source]) {
                if (!isin(visited, neighbor)) {
                    next_set.insert(neighbor);
                    level_graph[neighbor] = level;
                }
            }
        }

        // Mark the work set as visited and swap it to the next set
        visited = set_merge(visited, work_set);
        work_set = next_set;
    }

    // Return the max level found
    return level;
}


int KCMC_Instance::add_kcov(const int k, std::unordered_set<int> &visited_sensors) {
    int p, missing_coverage, i, coverage_contribution, qt_added=0;
    std::unordered_set<int> pois_missing_coverage, poi_covering_sensors;
    std::priority_queue<std::pair<int, int>> queue;

    // Do a first loop
    do {
        // For each POI, check if it has enough coverage
        for (p = 0; p < this->num_pois; p++) {
            poi_covering_sensors = this->poi_sensor[p];

            // Get how much coverage the POI is still missing, using olny the set of used sensors
            missing_coverage = k - ((int)(set_intersection(poi_covering_sensors, visited_sensors).size()));

            // If we have missing coverage, add the POI to the list of POIs missing coverage
            if (missing_coverage > 0) {
                pois_missing_coverage.insert(p);
            }
        }
        if (pois_missing_coverage.empty()) {break;}  // stop here if no POI needs coverage

        // For each sensor NOT in the set of used sensors
        while (!queue.empty()) {queue.pop();}  // Clear the priority queue before filling it again
        for (i = 0; i < this->num_sensors; i++) {
            if (!isin(visited_sensors, i)) {

                // Check how many POIs this sensor covers that are missing coverage,
                // and add this sensor to a priority queue ordered by the size of its coverage contribution
                coverage_contribution = (int)(set_intersection(pois_missing_coverage, this->sensor_poi[i]).size());
                if (coverage_contribution > 0) {
                    queue.push(std::make_pair(coverage_contribution, i));
                }
            }
        }
        if (queue.empty()) {throw std::runtime_error("No available sensors to cover the remaining POIs.");}

        // Add the sensor with the highest coverage contribution to the set of used sensors
        if (!queue.empty()) {
            visited_sensors.insert(queue.top().second);
            qt_added++;
        }

        // And from here we will keep looping
    }
    while (!pois_missing_coverage.empty());

    // If we got here, every POI has enough coverage.
    return qt_added;  // Return the number of sensors added
}
