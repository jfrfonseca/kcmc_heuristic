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
std::unordered_set<int> set_intersection(const std::unordered_set<int> &left, const std::unordered_map<int,int> &right) {
    std::unordered_set<int> intersection;
    for (const int &item : left) {
        if (right.find(item) != right.end()) { // If item IN right side
            intersection.insert(item);
        }
    }
    return intersection;
}
std::unordered_set<int> set_intersection(const std::unordered_map<int,int> &left, const std::unordered_set<int> &right) {return set_intersection(right, left);}


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


int KCMC_Instance::rank(std::vector<int> &poi_rank, std::vector<int> &sensor_rank, std::vector<int> &sink_rank) {

    // Prepare the buffers
    int total = 0;
    poi_rank.clear();
    poi_rank.resize(this->num_pois);
    sensor_rank.clear();
    sensor_rank.resize(this->num_sensors);
    sink_rank.clear();
    sink_rank.resize(this->num_sinks);

    // Get the rank of each POI based on the number of sensors covering it
    for (int n_poi=0; n_poi < this->num_pois; n_poi++) {
        poi_rank[n_poi] = ((int)this->poi_sensor[n_poi].size());
        total += poi_rank[n_poi];
    }

    // Get the rank of each sensor based on the number of neighbors it can communicate to
    for (int n_sensor=0; n_sensor < this->num_sensors; n_sensor++) {
        sensor_rank[n_sensor] = ((int)this->sensor_sensor[n_sensor].size());
        total += sensor_rank[n_sensor];
    }

    // Get the rank of each sink based on the number of sensors it can communicate to
    for (int n_sink=0; n_sink < this->num_sinks; n_sink++) {
        sink_rank[n_sink] = ((int)this->sink_sensor[n_sink].size());
        total += sink_rank[n_sink];
    }

    // Return the total rank
    return total;
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
            return (n_poi*LARGE_NUMBER)+(int)(active_coverage);
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
            return (n_poi*LARGE_NUMBER)+(int)(active_coverage);
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


int KCMC_Instance::add_kcov(const int k, std::unordered_set<int> &inactive_sensors, std::unordered_set<int> &solution) {

    // buffers
    int poi, qt_added=0, most_voted_index, most_voted_votes;
    std::vector<int> current_coverage, pois_missing_coverage;
    std::unordered_map<int, int> sensor_votes;  // Map from sensor to its quantity of votes

    // Start a buffer counting, for each poi, how many of the sensors already in the solution cover it
    // Start a queue of POIs that still need coverage
    // Return early if no POIs are still in need of coverage
    current_coverage.clear();
    current_coverage.resize(this->num_pois);
    pois_missing_coverage.clear();
    for (poi = 0; poi < this->num_pois; ++poi) {
        current_coverage[poi] = (int)(set_intersection(this->poi_sensor[poi], solution).size());
        // Add to the queue if the POI doesnt have enough coverage
        if (current_coverage[poi] < k) {pois_missing_coverage.push_back(poi);}
    }
    if (pois_missing_coverage.empty()) {return 0;}

    // While there are still POIs in the queue that need coverage
    while (!pois_missing_coverage.empty()) {

        // Reset the sensor votes for this iteration
        sensor_votes.clear();
        most_voted_votes=0;
        most_voted_index=-1;

        // For each POI in the queue
        for (const int &poi : pois_missing_coverage) {

            // For each sensor that covers this POI, is not inactive nor is already in the solution
            for (const int &sensor : this->poi_sensor[poi]) {
                if (!isin(inactive_sensors, sensor) && !isin(solution, sensor)) {

                    // Add a vote to the sensor
                    if (sensor_votes.find(sensor) == sensor_votes.end()) {sensor_votes[sensor] = 0;}
                    sensor_votes[sensor] += 1;

                    // Update the champion
                    if (sensor_votes[sensor] > most_voted_votes) {
                        most_voted_votes = sensor_votes[sensor];
                        most_voted_index = sensor;
                    }
                }
            }
        }
        if (most_voted_index == -1) {throw std::runtime_error("No valid sensor found to cover remaining POIs.");}

        // Add the most voted sensor to the solution
        solution.insert(most_voted_index);
        qt_added += 1;

        // Update the coverage - the whole coverage, since the sensor might cover many pois
        current_coverage.clear();
        current_coverage.resize(this->num_pois);
        pois_missing_coverage.clear();
        for (poi = 0; poi < this->num_pois; ++poi) {
            current_coverage[poi] = (int)(set_intersection(this->poi_sensor[poi], solution).size());
            // Add to the queue if the POI doesnt have enough coverage
            if (current_coverage[poi] < k) {pois_missing_coverage.push_back(poi);}
        }
        if (pois_missing_coverage.empty()) {return 0;}
    }

    return qt_added;  // Return the number of sensors added
}
