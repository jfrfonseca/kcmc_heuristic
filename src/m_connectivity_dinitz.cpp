

// STDLib dependencies
#include <sstream>    // ostringstream
#include <queue>      // priority_queue

// Dependencies from this package
#include "kcmc_instance.h"  // KCMC Instance class headers


/** M-CONNECTIVITY ACCORDING TO THE DINITZ ALGORITHM
 */
int KCMC_Instance::m_connectivity(int m, std::unordered_set<int> &inactive_sensors, std::unordered_set<int> *all_used_sensors) {

    // Buffers
    std::vector<int> level_graph(this->num_sensors, INVALID_PREVIOUS_POI);
    std::unordered_map<int, std::vector<Edge>> e_poi_sensor, e_poi_sensor_r,
                                               e_sensor_sensor_in, e_sensor_sensor_in_r,
                                               e_sensor_sensor_out, e_sensor_sensor_out_r,
                                               e_sensor_sink, e_sensor_sink_r;
    all_used_sensors->clear();

    return -1;  // Success in each and every POI, not the total value of connectivity!
}
int KCMC_Instance::m_connectivity(int m, std::unordered_set<int> &inactive_sensors) {
    std::unordered_set<int> emptyset;
    return this->m_connectivity(m, inactive_sensors, &emptyset);
}
