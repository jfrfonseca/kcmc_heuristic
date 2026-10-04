

// STDLib dependencies
#include <sstream>    // ostringstream
#include <queue>      // priority_queue

// Dependencies from this package
#include "kcmc_instance.h"  // KCMC Instance class headers

// Constants to improve readability
constexpr int UNVISITED_SENSOR = -2;
constexpr int INVALID_PREVIOUS_POI = -1;


/** COMPUTE THE NETWORK FLOW GRAPH
 * The network flow graph is computed splitting each sensor in the network into an "in" and "out" versions of the sensors.
 * The "in" version of a sensor receives incoming edges of the graph, while the "out" version sends outgoing edges.
 * There is a single directed edge from the "in" version of a sensor to the "out" version of the sensor.
 * This allows modeling edge-disjoint (and then node-disjoint) paths using edge capacities.
 */
void KCMC_Instance::compute_network_flow_graph() {

    // For every sensor in the instance
    for (int i=0; i<this->num_sensors; i++){

        // Get the sensor and check for errors
        Node &a_sensor = this->sensor[i];
        if (i != a_sensor.index) { throw std::runtime_error("MISALIGNED INDEXES! SENSOR " + std::to_string(a_sensor.index) + " SHOULD BE IN POSITION " + std::to_string(i)); }

        // Create the split versions of the sensor: "in" and "out" and add to the network nodes
        NetworkFlowNode sensor_in  = {&a_sensor, tNETWORK_IN_NODE};
        NetworkFlowNode sensor_out = {&a_sensor, tNETWORK_OUT_NODE};
        this->network_nodes.push_back(sensor_in);   // position = 2*a_sensor.index
        this->network_nodes.push_back(sensor_out);  // position = 2*a_sensor.index + 1

        // Add a single edge from the "in" version to the "out" version of the sensor, with capacity 1, without a reverse edge
        NetworkFlowEdge sensor_in_to_out_edge = {sensor_in, sensor_out, 1, INVALID_REVERSE_EDGE};
        this->network_edges.push_back(sensor_in_to_out_edge);
    }

    // For every POI in the instance
    for (int i=0; i<this->num_pois; i++){

        // Get the POI and check for errors
        Node &a_poi = this->poi[i];
        if (i != a_poi.index) { throw std::runtime_error("MISALIGNED INDEXES! POI " + std::to_string(a_poi.index) + " SHOULD BE IN POSITION " + std::to_string(i)); }

        // Just add the POI as an OUT node
        NetworkFlowNode poi_out = {&a_poi, tNETWORK_OUT_NODE};
        this->network_nodes.push_back(poi_out);  // position = (2*num_sensors) + a_poi.index
    }

    // For every SINK in the instance
    for (int i=0; i<this->num_sinks; i++){

        // Get the SINK and check for errors
        Node &a_sink = this->sink[i];
        if (i != a_sink.index) { throw std::runtime_error("MISALIGNED INDEXES! SINK " + std::to_string(a_sink.index) + " SHOULD BE IN POSITION " + std::to_string(i)); }

        // Just add the SINK as an IN node
        NetworkFlowNode sink_out = {&a_sink, tNETWORK_IN_NODE};
        this->network_nodes.push_back(sink_out);  // position = (2*num_sensors) + (num_pois) + a_sink.index
    }

    // For every connection between a POI and a sensor, add an edge from the correpsonding NF_POI node to the corresponding NF_SENSOR_IN node
    // AND a reverse edge from the SENSOR IN node to the POI out node, with 0 capacity
    for (const auto &poi_sensor_pair : this->poi_sensor) {
        int p = poi_sensor_pair.first;
        for (const int &sensor_index : poi_sensor_pair.second) {

            // Get the NetworkFlowNodes corresponding to the POI out node and the SENSOR IN node
            // Make an edge from the POI out node to the SENSOR IN node with capacity 1
            // Also add its reverse edge, with 0 capacity
            int nf_p = (2 * this->num_sensors) + p;
            int nf_i = 2 * sensor_index;
            NetworkFlowEdge poi_to_sensor_edge = {this->network_nodes[nf_p], this->network_nodes[nf_i], 1, ((int)this->network_edges.size()) + 1};
            NetworkFlowEdge reverse_poi_to_sensor_edge = {this->network_nodes[nf_i], this->network_nodes[nf_p], 0, ((int)this->network_edges.size())};
            this->network_edges.push_back(poi_to_sensor_edge);
            this->network_edges.push_back(reverse_poi_to_sensor_edge);
        }
    }

    // For every connection between a sensor, and the SINK, add an edge from the correpsonding NF_SENSOR_OUT node to the corresponding NF_SINK node
    // AND a reverse edge from the SINK node to the SENSOR OUT node, with 0 capacity
    for (const auto &sink_sensor_pair : this->sink_sensor) {
        int s = sink_sensor_pair.first;
        for (const int &sensor_index : sink_sensor_pair.second) {

            // Get the NetworkFlowNodes corresponding to the SINK IN node and the SENSOR OUT node
            // Make an edge from the SENSOR OUT node to the SINK IN node with capacity 1
            // Also add its reverse edge, with 0 capacity
            int nf_o = (2 * sensor_index) + 1;
            int nf_s = (2 * this->num_sensors) + (this->num_pois) + s;
            NetworkFlowEdge sensor_to_sink_edge = {this->network_nodes[nf_o], this->network_nodes[nf_s], 1, ((int)this->network_edges.size()) + 1};
            NetworkFlowEdge reverse_sensor_to_sink_edge = {this->network_nodes[nf_s], this->network_nodes[nf_o], 0, ((int)this->network_edges.size())};
            this->network_edges.push_back(sensor_to_sink_edge);
            this->network_edges.push_back(reverse_sensor_to_sink_edge);
        }
    }

    // For every connection between a sensor and another sensor, add an edge from the corresponding NF_SENSOR_OUT node to the corresponding NF_SENSOR_IN node
    // AND a reverse edge with 0 capacity
    for (const auto &sensor_sensor_pair : this->sensor_sensor) {
        int i = sensor_sensor_pair.first;
        for (const int &neighbor_index : sensor_sensor_pair.second) {

            // Get the NetworkFlowNodes corresponding to the SENSOR OUT node and the SENSOR IN node, and add it to the network edges
            int nf_i_out = (2 * i) + 1;
            int nf_j_in = (2 * neighbor_index);
            NetworkFlowEdge sensor_to_sensor_edge = {this->network_nodes[nf_i_out], this->network_nodes[nf_j_in], 1, ((int)this->network_edges.size()) + 1};
            NetworkFlowEdge reverse_sensor_to_sensor_edge = {this->network_nodes[nf_j_in], this->network_nodes[nf_i_out], 0, ((int)this->network_edges.size())};
            this->network_edges.push_back(sensor_to_sensor_edge);
            this->network_edges.push_back(reverse_sensor_to_sensor_edge);
        }
    }
}


/** M-CONNECTIVITY ACCORDING TO THE DINITZ ALGORITHM
 */
int KCMC_Instance::m_connectivity(int m, std::unordered_set<int> &inactive_sensors, std::unordered_set<int> *all_used_sensors) {
    throw std::runtime_error("m_connectivity method not yet implemented");
    return 0;  // Placeholder return value
}
int KCMC_Instance::m_connectivity(int m, std::unordered_set<int> &inactive_sensors) {
    std::unordered_set<int> emptyset;
    return this->m_connectivity(m, inactive_sensors, &emptyset);
}
