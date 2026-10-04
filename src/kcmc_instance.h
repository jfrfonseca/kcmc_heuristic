/** KCMC_INSTANCE.h
 * Header of the KCMC instance object
 * Jose F. R. Fonseca
 */


// STDLib dependencies
#include <vector>         // vector object
#include <unordered_set>  // unordered_set object
#include <unordered_map>  // unordered_map HashMap object
#include <cmath>          // sqrt, pow


#ifndef KCMC_INSTANCE_H
#define KCMC_INSTANCE_H

// Types
#define tPOI 0
#define tSENSOR 1
#define tSINK 2
#define tNETWORK_IN_NODE 3
#define tNETWORK_OUT_NODE 4

// Sentinels & Hard Limits
#define LARGE_NUMBER 1000000
#define MAX_M_CONNECTIVITY 100
#define INVALID_REVERSE_EDGE -10
#define MAX_GENERATION_ATTEMPTS 1000
#define USE_GREEDY true

// GenAlg
//#define INSPECTION_FREQUENCY 100
//#define WORST_FITNESS 9999999999

// Signal handler for exit events
void exit_signal_handler(int signal);


/* NODE
 * Basic building block of the KCMC Instance. Contains its type (poi, sensor, sink), index (in array) and dinic level
 * LevelNodes can be compared in function of their level.
 * Placement is a buffer for positioning nodes when generating a random instance.
 * The euclidean distance between two placements can also be computed in function of its X and Y coordinates.
 */
struct Node {
    int node_type;
    int index;
};

struct LevelNode {
    int index;
    int level;
};

struct CompareLevelNode {
    bool operator()(LevelNode const& a, LevelNode const& b) {
        // return "true" if "a" is ordered before "b"
        if (a.level == b.level){return (a.index <= b.index);}  // INCREASING order, if at the same level
        else {return (a.level > b.level);}  // DECREASING order
    }
};

struct Placement {
    Node *node;
    int x, y;
};

double distance(Placement source, Placement target);


/* ISIN
 * Many-types-of-input verification if a given item is in the reference set.
 * If the reference set is a mapping, the search is in its keys.
 */
bool isin(std::unordered_map<int, std::unordered_set<int>> &ref, int item);
bool isin(std::unordered_map<int, int> &ref, int item);
bool isin(std::unordered_set<int> &ref, int item);
bool isin(std::unordered_set<std::string> &ref, const std::string &item);
bool isin(std::vector<int> &ref, int item);
bool isin(std::vector<int> *ref, int item);


/* PUSH AND VOTE
 * PUSH: Adds a new pair in a mapping, from the source to a set containing only the target.
 *       If the source is already in the set, the target is added to its mapped data.
 * VOTE: Adds an element to a map, pointing to the value 1.
 *       It the source element is alreary in the map, increase the value of its mapped data by 1.
 */
void push(std::unordered_map<int, std::unordered_set<int>> &buffer, int source, int target);
void vote(std::unordered_map<int, int> &buffer, int target, int value);
void vote(std::unordered_map<int, int> &buffer, int target);


/* SET MERGE & DIFF
 * Returns the set that is the sum (or difference) of the given sets
 */
template<class T>
T set_merge (T a, T b) {T t(a); t.insert(b.begin(),b.end()); return t;}
std::unordered_set<int> set_diff(const std::unordered_set<int> &left, const std::unordered_set<int> &right);
std::unordered_set<int> set_intersection(const std::unordered_set<int> &left, const std::unordered_set<int> &right);


/* SETIFY
 * Returns a set from other data structure
 */
void setify(std::unordered_set<int> &target, int size, int source[], int reference);
void setify(std::unordered_set<int> &target, std::unordered_map<int, int> *reference);


/* FLOW NETWORK
 * Internal representation of the Network Flow used in the KCMC instance.
 * NetworkFlowNode is a wrapper around the original Node, used in the network representation of the instance, since every node gets two network nodes (in and out).
 * NetworkFlowEdge is an edge in the network flow representation, containing the source and target nodes, current flow, maximum flow, and the index of the reverse edge in the edges array.
 */

struct NetworkFlowNode {
    Node *original_node;
    int network_node_type;
};

struct NetworkFlowEdge {
    NetworkFlowNode source;
    NetworkFlowNode target;
    int current_flow;
    int max_flow;
    int idx_reverse_edge;
};


// #####################################################################################################################


/** KCMC Instance Object
 * Contains node vectors for pois, sensors, sinks.
 * Vector of unordered sets listing the neighbors of each poi, sensor and sink
 */


class KCMC_Instance {

    public:
        /* Descriptor INTEGER constants
         * Quantity of POIs, sensors, sinks.
         * Dispersion area of the instance
         * Sensor coverage and communication radius
         * Random seed for component placement
         */
        int num_pois, num_sensors, num_sinks, area_side, sensor_coverage_radius, sensor_communication_radius;
        long long random_seed;

        /* Component buffers
         * Separate Node Vectors for pois, sensors and sinks
         * Each vector contains a node of index to its position in the vector
         * Also a vector for the Network nodes and another for the Network edges.
         */
        std::vector<Node> poi, sensor, sink;
        std::vector<NetworkFlowNode> network_nodes;
        std::vector<NetworkFlowEdge> network_edges;

        /* Graph edges sparse matrix
         * Separate HashMaps of UnorderedSet of indexes of Nodes for poi-sensor, sensor-sensor and sensor-sink edges.
         * Each HashMap contains an UnorderedSet of indexes of Nodes. The index in the HashMap is relative to the
         *     index of the base Node it represents. The content of the HashMap is an unordered set of the indexes of
         *     the Nodes that the index node is neighbor of.
         * There are no Poi-Poi, Poi-Sink nor Sink-Sink edges
         */
        std::unordered_map<int, std::unordered_set<int>> poi_sensor, sensor_poi, sensor_sensor, sensor_sink, sink_sensor;

        /* Random-instance generator constructor
         * Receives the instance descriptive constants and makes an instance of randomly-placed Nodes.
         */
        KCMC_Instance(int num_pois, int num_sensors, int num_sinks,
                      int area_side, int coverage_radius, int communication_radius,
                      long long random_seed);

        /* Instance de-serializes constructor
         * Receives a serialized instance and constructs an KCMC_Instance object from it.
         */
        explicit KCMC_Instance(const std::string& serialized_kcmc_instance);

        /* Instance utilities
         * Get the KEY of the current instance
         * Serialize the current instance as a string
         * Gets the degree of each sensor in the instance considering the given set of inactive sensors.
         * Shows the placements of POIs, sensors, and sinks in the instance.
         * Invert a set of sensors (get every sensor in the instance not in the set)
         */
        std::string key() const;
        std::string serialize();
        int get_degree(int buffer[], std::unordered_set<int> &inactive_sensors);
        void get_placements(Placement *pl_pois, Placement *pl_sensors, Placement *pl_sinks);
        int invert_set(std::unordered_set<int> &source_set, std::unordered_set<int> *target_set);

        /* Instance payload services - K-COVERAGE
         * Work both in GREEDY and DINITZ versions of the heuristics
         * Get coverage is retrieves the number of sensors covering each POI in the instance.
         * Validates k-coverage in the instance considering the given set of inactive sensors
         */
        int get_coverage(int buffer[], std::unordered_set<int> &inactive_sensors);
        int k_coverage(int k, std::unordered_set<int> &inactive_sensors);
        int k_coverage(int k, std::unordered_set<int> &inactive_sensors, std::unordered_set<int> *all_used_sensors);

        // METHODS FOR THE GREEDY HEURISTIC ---------------------------------------------------------------------------

        /* Instance payload services - GREEDY heuristic
         * See below why the GREEDY version does not always works
         * Validates m-connectivity in the instance considering the given set of inactive sensors
         */
        int greedy_level_graph(std::vector<int> &greedy_level_graph, std::unordered_set<int> &inactive_sensors);
        int m_connectivity_greedy(int m, std::unordered_set<int> &inactive_sensors, std::unordered_map<int, int> *all_used_sensors, int path_increase_tolerance);
        int m_connectivity_greedy(int m, std::unordered_set<int> &inactive_sensors, std::unordered_map<int, int> *all_used_sensors);
        int m_connectivity_greedy(int m, std::unordered_set<int> &inactive_sensors, std::unordered_set<int> *all_used_sensors);
        int get_connectivity_greedy(int buffer[], std::unordered_set<int> &inactive_sensors, int target);
        int get_connectivity_greedy(int buffer[], std::unordered_set<int> &inactive_sensors);

        /* Instance Preprocessors - GREEDY heuristic
         * We compute a level graph using an heuristic based on a misunderstanding of the implications of Menger's Theorem on Dinitz's algorithm.
         * We compute a level-graph using BFS starting at the sink, and then find the shortest paths from POIs to the sink using a greedy DFS.
         * This heuristic supposes that a greedy DFS can yield the max flow in a network if all edges have max capacity of 1, and that is incorrect.
         * It will fail in some cases like the one below, in which p are POIs, i are sensors, s is the sink and edges are - or / or \
         *     i2 ---- i3
         *    /          \
         *   p1 -- i0 -- i1 -- s
         *          \         /
         *           i4 --- i5
         * Here, the greedy DFS will find a single node-disjoint path (p1 -> i0 -> i1 -> s), and there are actually two node-disjoint paths available.
         * Those would be (p1 -> i2 -> i3 -> i1 -> s) and (p1 -> i0 -> i4 -> i5 -> s)
         * The other heuristics below are based on the greedy approach and do not overcome its limitations.
         *
         * DKov runs the greedy heuristic from every POI to the sink, getting M paths between the POI and the sink.
         *   The sensors in each path are gathered in a set, and more POI-covering sensors are added to the set until each POI is covered
         *     by at least K sensors, giving a feasible solution for the KCMC Problem on the instance.
         *
         * Reuse is similar to DKOV, however
         *   The Minimal flood does it only for the required dinic paths. Full flood keeps on adding paths to the
         *     minimal requirements until paths start to increase, so it has way more sensors.
         * Reuse uses the full-flood to get paths. Each path votes on all its composing sensors. Then, new paths are
         *   created preferring the most voted sensors in each dinic level.
         */
        int greedy_dkov(int k, int m, std::unordered_set<int> &inactive_sensors, std::unordered_map<int, int> *visited_sensors);
        int greedy_reuse(int k, int m, std::unordered_set<int> &inactive_sensors, std::unordered_map<int, int> *visited_sensors);
        int greedy_breadth(int k, int m, std::unordered_set<int> &inactive_sensors, std::unordered_map<int, int> *visited_sensors);

        // METHODS FOR THE DINITZ HEURISTIC ---------------------------------------------------------------------------
        int m_connectivity(int m, std::unordered_set<int> &inactive_sensors);
        int m_connectivity(int m, std::unordered_set<int> &inactive_sensors, std::unordered_set<int> *all_used_sensors);
        //int get_connectivity(int buffer[], std::unordered_set<int> &inactive_sensors, int target);
        //int get_connectivity(int buffer[], std::unordered_set<int> &inactive_sensors);

        // METHODS THAT MAY USE THE GREEDY OR THE DINITZ HEURISTICS ---------------------------------------------------

        /* VALIDATION
         * Validate the instance, raising errors if invalid. Some arguments are optional
         */
        bool validate(bool raise, int k, int m);  // Do NOT use the greedy heuristic, by default
        bool validate(bool raise, int k, int m, bool use_greedy);  // Option to use the greedy heuristic
        bool validate(bool raise, int k, int m, std::unordered_set<int> &inactive_sensors);
        bool validate(bool raise, int k, int m, std::unordered_set<int> &inactive_sensors, bool use_greedy);  // Option to use the greedy heuristic
        bool validate(bool raise, int k, int m, std::unordered_set<int> &inactive_sensors,
                      std::unordered_set<int> *k_used_sensors,
                      std::unordered_set<int> *m_used_sensors);
        bool validate(bool raise, int k, int m, std::unordered_set<int> &inactive_sensors,
                      std::unordered_set<int> *k_used_sensors,
                      std::unordered_set<int> *m_used_sensors, bool use_greedy);  // Option to use the greedy heuristic

    private:
        // GENERAL ================================================================================
        // Private version of the get_placements method
        void get_placements(Placement *pl_pois, Placement *pl_sensors, Placement *pl_sinks, bool push);
        // Instance de-serialization from the short instance definition
        void regenerate();
        // Method to parse the edges of a serialized instance
        int parse_edge(int stage, const std::string& token);
        // Method to add non-visited sensors to achieve k-coverage
        int add_kcov(const int k, std::unordered_map<int, int> *visited_sensors);

        // GREEDY =================================================================================
        // Method that uses A* to find a path for the greedy heuristic
        int greedy_find_path(int poi_number, std::unordered_set<int> &used_sensors,
                             std::vector<int> &level_graph, std::vector<int> &predecessors);
        // Private version of Reuse that also implements the heuristic Breadth
        int greedy_reuse(int k, int m, std::unordered_set<int> &inactive_sensors, std::unordered_map<int, int> *visited_sensors, int path_increase_tolerance);

        // DINITZ =================================================================================
        // Method to compute the network flow graph from the instance
        void compute_network_flow_graph();

};

#endif
