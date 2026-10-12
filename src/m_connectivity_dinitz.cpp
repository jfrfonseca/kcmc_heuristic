

// STDLib dependencies
#include <sstream>    // ostringstream
#include <queue>      // priority_queue

// Dependencies from this package
#include "kcmc_instance.h"  // KCMC Instance class headers

// Debug mode
#define DEBUG_DINITZ false


/** M-CONNECTIVITY ACCORDING TO THE DINITZ ALGORITHM
 */
bool compute_network_level_graph(
    std::unordered_map<int, std::vector<Edge>> &network,  // Invariant
    std::vector<int> &level_graph                         // Result buffer
) {

    // Buffers
    // ALWAYS start from the penultimate position of the level graph and get to the last one
    int a_sensor, from_sensor, idx_source=(((int)level_graph.size())-2), idx_sink=(((int)level_graph.size())-1);
    std::queue<int> queue;
    queue.push(idx_sink);  // Start at the SINK

    // Reset the result buffer
    fill(level_graph.begin(), level_graph.end(), INVALID_PREVIOUS_POI);
    level_graph[idx_sink] = 0;  // Set the sink level to 0

    // While there is still sensors to explore
    // Get the sensor at the front of the queue and pop (remove) it from the queue
    while (!queue.empty()) {
        a_sensor = queue.front();
        queue.pop();

        // For each edge leaving the current sensor
        for (const auto& edge : network[a_sensor]) {
            from_sensor = edge.towards_sensor;
            const auto& inverted_edge_to_follow = network[from_sensor][edge.reverse_edge_index];

            // If the edge still have capacity and the target sensor has not been visited yet
            // GO TO THE EDGE IN THE OPPOSITE DIRECTION, GOING TO THE from_sensor INSTEAD OF THE towards_sensor
            if (((inverted_edge_to_follow.capacity - inverted_edge_to_follow.flow) > 0) && (level_graph[from_sensor] == INVALID_PREVIOUS_POI)) {

                // Set the sensor to one level above the current sensor and add it to the end of the queue
                level_graph[from_sensor] = level_graph[a_sensor] + 1;
                queue.push(from_sensor);
            }
        }
    }

    // Return TRUE if the source is reachable from the sink in the level graph
    return (level_graph[idx_source] != INVALID_PREVIOUS_POI);  // The source is reachable from the sink in the level graph if its level is not INVALID_PREVIOUS_POI
}


int find_blocking_flow(
    std::vector<int> &level_graph,
    std::vector<int> &edge_checkpoint,
    std::unordered_map<int, std::vector<Edge>> &network,
    int current_node,
    int destination_node,
    int pushed_flow
) {
    if (DEBUG_DINITZ) {printf("Node %d[%d]\n", current_node, level_graph[current_node]);}

    // Buffers
    int current_flow;

    // Base case - if no flow can be pushed or we've reached the destination node, return the pushed flow
    if ((pushed_flow == 0) || (current_node == destination_node)) return pushed_flow;

    // The edge checkpoint ensures we don't re-examine edges (and paths) that cannot bear any more flow
    // This is the main reason why the edges in the network must be an ordered vector leaving each node
    // We must iterate until we have no more edges to examine from the current node, that is to say, when
    //   the checkpoint reaches the end of the edge list for the current node.
    while (edge_checkpoint[current_node] < ((int)network[current_node].size())) {
        Edge& edge = network[current_node][edge_checkpoint[current_node]];

        if (DEBUG_DINITZ) {printf("  EDGE (%d/%d) {%d/%d} -> %d[%d] = %d\n    (%b || %b) && %b\n",
            edge.flow, edge.capacity,
            edge_checkpoint[current_node], (int)network[current_node].size(),
            edge.towards_sensor, level_graph[edge.towards_sensor],
            edge.flow,
            ((level_graph[edge.towards_sensor] + 1) == level_graph[current_node]),
            (edge.towards_sensor == destination_node),
            ((edge.capacity - edge.flow) > 0)
        );}

        // If the edge leads to the next level, or if it can still bear more flow
        if (
            (
                ((level_graph[edge.towards_sensor] + 1) == level_graph[current_node])  // ONLY moves in the right direction, one step at a time
             || (edge.towards_sensor == destination_node)  // Allow moving directly to the destination node, bypassing the level graph constraint
            )
         && ((edge.capacity - edge.flow) > 0)  // ONLY moves forward if the current edge was not used in no path!
        ){
            if (DEBUG_DINITZ) {printf("    FOLLOW\n");}

            // Compute the flow that can be pushed through this edge by recursively
            current_flow = std::min(pushed_flow, (edge.capacity - edge.flow));
            pushed_flow = find_blocking_flow(
                level_graph,          // Loop invariant
                edge_checkpoint,      // Will be updated
                network,              // Will be updated
                edge.towards_sensor,  // Changes every loop
                destination_node,     // Loop invariant
                current_flow          // Not changed in the function
            );

            if (DEBUG_DINITZ) {
                printf("Node %d[%d]\n", current_node, level_graph[current_node]);
                printf("  EDGE (%d/%d) {%d/%d} -> %d[%d] GOT +%d = %d\n",
                    edge.flow, edge.capacity,
                    edge_checkpoint[current_node], (int)network[current_node].size(),
                    edge.towards_sensor, level_graph[edge.towards_sensor],
                    pushed_flow, edge.flow+pushed_flow
                );
            }

            if (DEBUG_DINITZ) {wait_user();}

            // If we can still push more flow through the edge
            if (pushed_flow > 0) {

                // Add flow to the present edge and remove it from the reverse edge
                edge.flow += pushed_flow;
                network[edge.towards_sensor][edge.reverse_edge_index].flow -= pushed_flow;

                // Return immediately. If we end up at a dead-end we will backtrack using the edge checkpoint vector
                return pushed_flow;
            }
        }

        // If we got here, the edge cannot bear any more flow or it leads to a dead-end.
        // Thus, we update the edge checkpoint to skip this edge in future iterations.
        edge_checkpoint[current_node]++;
    }
    return 0;
}


void add_directed_edge_and_reversed_edge(std::unordered_map<int, std::vector<Edge>> &edges, int source_sensor, int destination_sensor, int cap) {
    // Create an edge and the corresponding reverse edge
    // The directed edge goes from the source to the destination, and the reversed edge goes in the opposite direction.
    // The reverse edge starts with 0 capacity
    edges[source_sensor].push_back(     {destination_sensor, 0, cap, (int)edges[destination_sensor].size()});
    // REVERSED edge goes from the destination to the source with 0 initial capacity
    edges[destination_sensor].push_back({source_sensor, 0,   0, ((int)edges[source_sensor].size())-1});  // The -1 here is VERY IMPORTANT to avoid segfault
}


int KCMC_Instance::m_connectivity(
    const int m,    // Constant - minimal number of disjoint paths to find
    const int poi,  // Loop pointer - the current POI
    std::unordered_set<int> &inactive_sensors,  // Loop invariant - the set of sensors that are currently inactive
    std::unordered_map<int,int> &poi_used_sensors   // Output buffer - The sensors used in the paths for this POI, mapping to the number of its path
) {

    // Buffers
    bool required_added;
    int num_path, pushed_flow, a_sensor_in, a_sensor_out, next_sensor_in, a_sink, loop_counter=0,
        total_flow=0, idx_poi=(2*this->num_sensors), idx_sink=((2*this->num_sensors) + 1);
    std::unordered_map<int, std::vector<Edge>> network;
    std::vector<int> level_graph, edge_checkpoint, path;

    // Set the size of the level graph buffer, counting each sensor twice (in and out versions)
    // Also put two positions, one for the POI and one for the SINK nodes.
    // Start a buffer for the list of added edges to avoid duplicates
    level_graph.resize((2*this->num_sensors) + 2);
    edge_checkpoint.resize((2*this->num_sensors) + 2);

    // Reset the network with the two versions of sensors (IN and OUT)
    //   The IN version of the sensor is always in its original index
    //   The OUT version of a sensor is always its index plus the total number of sensors.
    // Add the IN -> OUT edges for each sensor to its other version, always with capacity 1
    // Add the OUT -> IN edges for the sensor to each of its active neighbors, always with capacity 1
    // Always ignore inactive sensors.
    // There is still the need to add the POI node for the current POI, and the SINK node
    network.clear();
    for (a_sensor_in = 0; a_sensor_in < this->num_sensors; ++a_sensor_in) {
        if (isin(inactive_sensors, a_sensor_in)) {continue;}
        a_sensor_out = a_sensor_in + this->num_sensors;

        // Add an edge from the IN version to the OUT version of the current sensor.
        add_directed_edge_and_reversed_edge(network, a_sensor_in, a_sensor_out, 1);

        // For each active neighbor of the current sensor,
        // add an edge from the OUT version of the current sensor to the IN version of the neighbor.
        for (const auto& a_neighbor_in : this->sensor_sensor[a_sensor_in]) {
            if (isin(inactive_sensors, a_neighbor_in)) {continue;}  // Ignore inactive neighbor sensors

            // Mark this edge as added to avoid adding it again in the opposite direction.
            add_directed_edge_and_reversed_edge(network, a_sensor_out, a_neighbor_in, 1);  // Always the IN version of the neighbor
        }
    }

    // Add the POI node for the current POI, and an edge from the POI to the IN version of each active sensor that covers it.
    // The POI node will be in position "2*num_sensors", and the capacity is always 1
    // Ignore inactive sensors
    required_added = false;
    for (const auto& a_sensor_in : this->poi_sensor[poi]) {
        if (isin(inactive_sensors, a_sensor_in)) {continue;}
        add_directed_edge_and_reversed_edge(network, idx_poi, a_sensor_in, 1);  // Always the IN version of the sensor
        required_added = true;
    }
    if (!required_added) {throw std::runtime_error("No active sensor covers the current POI.");}

    // Add the SINK node. Since we only have to reach any sink, then we add a single node for all sinks.
    // Add an edge from the OUT version of each active sensor that covers the sink to the SINK node.
    // The SINK node will be in position "2*num_sensors + 1", and the capacity is always 1
    // Ignore inactive sensors
    required_added = false;
    for (a_sink=0; a_sink<this->num_sinks; ++a_sink) {
        for (const auto& a_sensor : this->sink_sensor[a_sink]) {
            if (isin(inactive_sensors, a_sensor)) {continue;}
            a_sensor_out = a_sensor + this->num_sensors;
            add_directed_edge_and_reversed_edge(network, a_sensor_out, idx_sink, 1);  // Always the OUT version of the sensor
            required_added = true;
        }
    }
    if (!required_added) {throw std::runtime_error("No active sensor can transmit to any sink.");}

    // Print the current state of the level graph and the network, if in debug_DINITZ mode
    if (DEBUG_DINITZ) {
        printf("====================================================================================================");
        printf("\nFlow Network:\n");
        // header
        printf("    ");
        for (int i = 0; i < (int)network.size(); ++i) {
            if (i >= 10) printf("%d |", i);
            else printf(" %d |", i);
        }
        printf("\n    ");
        for (int i = 0; i < (int)network.size(); ++i) {printf("----");}
        // rows of the network, each representing a node and its outgoing edges
        for (int i = 0; i < (int)network.size(); ++i) {
            if (i >= 10) printf("\n%d |", i);
            else printf("\n %d |", i);
            for (int j=0; j < (int)network.size(); j++) {
                bool has_edge = false;
                for (const auto edge : network[i]) {
                    if (edge.towards_sensor == j) {
                        printf("  %d|", edge.flow);
                        has_edge = true;
                        break;
                    }
                }
                if (!has_edge) {printf("   |");}
            }
        }
        printf("\n");
        wait_user();
    }

    // While there is still flow to be had in the network
    while (compute_network_level_graph(
        network,     // Will not be updated here (no change in flow to compute the level graph)
        level_graph  // Will be updated with the level graph of the current network state
    )) {

        // Reset the edge checkpoints, since the level graph was just recomputed
        std::fill(edge_checkpoint.begin(), edge_checkpoint.end(), 0);

        // Compute the flow that can still be pushed
        while (true) {
            pushed_flow = find_blocking_flow(
                level_graph,      // Will not be updated here (no change to the level graph until the next iteration)
                edge_checkpoint,  // Will be updated with the current position in the edge list for each node
                network,          // Will be updated with the current flow in each edge
                idx_poi,          // Loop invariant
                idx_sink,         // Loop invariant
                LARGE_NUMBER      // Loop starter
            );
            if (pushed_flow < 0) {throw std::runtime_error("NEGATIVE FLOW!");}
            if (pushed_flow == 0) {break;}
            else {total_flow += pushed_flow;}
        }

        // Print the current state of the level graph and the network, if in debug_DINITZ mode
        if (DEBUG_DINITZ) {
            printf("====================================================================================================");
            printf("\nLOOP %d | TOTAL FLOW %d | PUSHED %d | \nLevel Graph:\n", loop_counter, total_flow);
            for (int i = 0; i < (int)level_graph.size(); ++i) {printf("%s[%d] ", stringfy(2, i).c_str(), level_graph[i]);}
            printf("\nFlow Network:\n");
            // header
            printf("     ");
            for (int i = 0; i < (int)network.size(); ++i) {printf(" %s|", stringfy(3, i).c_str());}
            printf("\n     ");
            for (int i = 0; i < (int)network.size(); ++i) {printf("-----");}
            // rows of the network, each representing a node and its outgoing edges
            for (int i = 0; i < (int)network.size(); ++i) {
                printf("\n %s|", stringfy(3, i).c_str());
                for (int j=0; j < (int)network.size(); j++) {
                    bool has_edge = false;
                    for (const auto edge : network[i]) {
                        if (edge.towards_sensor == j) {
                            printf(" %s|", stringfy(3, edge.flow).c_str());
                            has_edge = true;
                            break;
                        }
                    }
                    if (!has_edge) {printf("    |");}
                }
            }
            printf("\n");
            wait_user();
        }

        // Count the loop
        loop_counter++;
    }

    // Here, we know the total flow!
    // If m < 0, we only want to know the value of the flow
    if (m < 0) {return total_flow;}

    // If (m >= 0), we want to check if we have at leas m vertex-disjoint paths. If we dont, return while signaling an error
    if (total_flow < m){return ((1+poi)*LARGE_NUMBER) + total_flow;}  // Encoded the two ints. We must not have more than a LARGE_NUMBER of POIs!}

    // If we got here, we have at least m flow and we want to know all the sensors that are a part of it
    // For each path found
    poi_used_sensors.clear();
    for (num_path = 0; num_path < total_flow; ++num_path) {

        // Reset the path buffer and the pointer of the start ofthe path
        path.clear();
        a_sensor_in = idx_poi;

        // While we are not at the sink
        while (a_sensor_in != idx_sink) {

            // Compute the next sensor in the path
            next_sensor_in = INVALID_PREVIOUS_POI;
            for (auto& edge : network[a_sensor_in]) {

                // If the edge has flow and it is at capacity
                if (edge.flow == 1 && edge.capacity == 1) {

                    // Clear the flow (so we can~t use the edge again) and use it to get to the next sensor
                    edge.flow = 0;
                    next_sensor_in = edge.towards_sensor;
                    break;
                }
            }

            // If we got here and the next sensor is still invalid, we arrived at the end of the path
            if ((next_sensor_in == INVALID_PREVIOUS_POI) || (next_sensor_in == idx_sink)) {break;}
            // If we got here, the next sensor is a part of the path

            // Add the next sensor to the registry of outputs
            path.push_back(next_sensor_in);  // IN version of the next_sensor
            poi_used_sensors[next_sensor_in] = num_path;  // Add the IN version, mapping to this path

            // Redirect the a_sensor to the OUT version of the next_sensor
            a_sensor_in = next_sensor_in + this->num_sensors;
        }
    }
    return INVALID_PREVIOUS_POI;  // Signal SUCCESS at the POI!
}


int KCMC_Instance::m_connectivity(int m, std::unordered_set<int> &inactive_sensors, std::unordered_set<int> &all_used_sensors) {
    // Prepare the buffers
    int poi, poi_connectivity;
    std::unordered_map<int,int> poi_used_sensors;
    all_used_sensors.clear();

    // For each POI
    for (poi = 0; poi < this->num_pois; ++poi) {

        // Get the connectivity in the POI
        poi_connectivity = m_connectivity(m, poi, inactive_sensors, poi_used_sensors);

        // Check for the error encoded in a large number
        if (poi_connectivity > LARGE_NUMBER){return poi_connectivity;}  // Return the error signaled
        if (poi_connectivity != INVALID_PREVIOUS_POI){throw std::runtime_error("IMPOSSIBLE CONDITION WITH CONNECTIVITY " + std::to_string(m));}

        // If we got here, success for the POI! Lets add all the used sensors to the set
        for (const auto &entry : poi_used_sensors) {all_used_sensors.insert(entry.first);}
    }

    // Here, we had success in each and every POI, and the all_used_sensors set contains all the sensors we used
    return -1;  // Success in each and every POI, not the total value of connectivity!
}


int KCMC_Instance::m_connectivity(int m, std::unordered_set<int> &inactive_sensors) {
    std::unordered_set<int> emptyset;
    return this->m_connectivity(m, inactive_sensors, emptyset);
}
