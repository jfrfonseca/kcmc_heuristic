/** KCMC_INSTANCE_HANDLING.cpp
 * Implementation of non-payload services of the KCMC instance object headers
 * Jose F. R. Fonseca
 */


// STDLib dependencies
#include <vector>     // vector
#include <sstream>    // ostringstream
#include <random>     // mt19937, uniform_real_distribution
#include <algorithm>  // std::find

// Dependencies from this package
#include "kcmc_instance.h"  // KCMC Instance class headers


/* #####################################################################################################################
 * INSTANCE OPERATION & CONSTRUCTORS
 */

/** RANDOM-INSTANCE COMPONET PLACEMENTS (RE)GENERATOR
 * Generates the instance's components placements, assuming the instance already have all main attributes.
 * THE PLACEMENTS WILL BE USED TO COMPUTE THE EDGES
 */
void KCMC_Instance::get_placements(std::vector<Placement> &pl_pois, std::vector<Placement> &pl_sensors, std::vector<Placement> &pl_sinks, bool push) {

    // Iteration buffers
    int i;

    // Prepare the random number generators
    std::mt19937 gen(this->random_seed);
    std::uniform_real_distribution<> point(0, this->area_side);

    // Set the POIs buffers
    for (i=0; i<this->num_pois; i++) {
        Node a_poi = {tPOI, i};
        if (push) {this->poi.push_back(a_poi);}
        pl_pois[i] = {&a_poi, (int)(point(gen)), (int)(point(gen))};
    }

    // Set the SENSORs buffers
    for (i=0; i<this->num_sensors; i++) {
        Node a_sensor = {tSENSOR, i};
        if (push) {this->sensor.push_back(a_sensor);}
        pl_sensors[i] = {&a_sensor, (int)(point(gen)), (int)(point(gen))};
    }

    // Set the SINKs buffers (if there is a single sink, it will be at the center of the area)
    if (this->num_sinks == 1) {
        Node a_sink = {tSINK, 0};
        if (push) {this->sink.push_back(a_sink);}
        pl_sinks[0] = {&a_sink, (int)(this->area_side / 2.0), (int)(this->area_side / 2.0)};
    } else {
        for (i=0; i<this->num_sinks; i++) {
            Node a_sink = {tSINK, i};
            if (push) {this->sink.push_back(a_sink);}
            pl_sinks[i] = {&a_sink, (int)(point(gen)), (int)(point(gen))};
        }
    }
}
void KCMC_Instance::get_placements(std::vector<Placement> &pl_pois, std::vector<Placement> &pl_sensors, std::vector<Placement> &pl_sinks) {
    this->get_placements(pl_pois, pl_sensors, pl_sinks, false);
}


/** RANDOM-INSTANCE (RE)GENERATOR
 * Generates the instance's placements and edges, assuming the instance already have all main attributes
 */
void KCMC_Instance::regenerate() {
    /** Random-instance (re)generator
     * This constructor is used only to generate a new random instance that already has the seed attributes
     */

    // Prepare iteration buffers
    int i, j;

    // Prepare the placement buffers. The scope of these buffers is only the constructor itself
    std::vector<Placement> pl_pois(this->num_pois), pl_sensors(this->num_sensors), pl_sinks(this->num_sinks);

    // Get the placemens of the instance objects
    this->get_placements(pl_pois, pl_sensors, pl_sinks, true);  // Use the private version, that pushes components

    // Iterate each sensor and find its connections
    for (i=0; i<this->num_sensors; i++) {

        // Iterate each POI, identifying sensor-poi coverage
        for (j=0; j < this->num_pois; j++) {
            if (distance(pl_sensors[i], pl_pois[j]) <= this->sensor_coverage_radius) {
                push(this->poi_sensor, j, i);
                push(this->sensor_poi, i, j);
            }
        }

        // Verify if the sensor can connect to a SINK
        for (j=0; j<this->num_sinks; j++) {
            if (distance(pl_sensors[i], pl_sinks[j]) <= this->sensor_communication_radius) {
                push(this->sensor_sink, i, j);  // Symetric communication between sink and sensors
                push(this->sink_sensor, j, i);  // Symetric communication between sink and sensors
            }
        }

        // Iterate each further sensor, identifying connections between sensors
        for (j=i+1; j < this->num_sensors; j++) {
            if (distance(pl_sensors[i], pl_sensors[j]) <= this->sensor_communication_radius) {
                push(this->sensor_sensor, i, j);  // Symetric communication between sensors
                push(this->sensor_sensor, j, i);  // Symetric communication between sensors
            }
        }
    }
    // From here on, the placement buffers are no longer needed
}


/** RANDOM-INSTANCE GENERATOR CONSTRUCTOR
 * Constructor of a random KCMC instance object
 */
KCMC_Instance::KCMC_Instance(int num_pois, int num_sensors, int num_sinks,
                             int area_side, int coverage_radius, int communication_radius,
                             long long random_seed) {
    /** Random-instance generator constructor
     * This constructor is used only to generate a new random instance
     */

    // Copy the variables
    this->num_pois = num_pois;
    this->num_sensors = num_sensors;
    this->num_sinks = num_sinks;
    this->area_side = area_side;
    this->sensor_coverage_radius = coverage_radius;
    this->sensor_communication_radius = communication_radius;
    this->random_seed = random_seed;

    // Now that we have the main attributes, we can (re)generate the instance
    this->regenerate();
}


/** INSTANCE DE-SERIALIZER CONSTRUCTOR
 * Constructor of a KCMC instance object from a serialized string
 */
KCMC_Instance::KCMC_Instance(const std::string& serialized_kcmc_instance) {
    /** Instance de-serializer constructor
     * This constructor is used to load a previously-generated instance. Node placements are irrelevant
     */

    // Iterate the string, looking for tokens
    size_t previous = 0, pos = 0;
    std::string token;
    int stage = 0, has_edges = 0;
    while ((pos = serialized_kcmc_instance.find(';', previous)) != std::string::npos) {
        token = serialized_kcmc_instance.substr(previous, pos-previous);
        std::stringstream s_token(token);

        switch(stage) {
            case 0:
                // VALIDATE PREFIX
                if ("KCMC" != token) {throw std::runtime_error("INSTANCE DOES NOT STARTS WITH PREFIX 'KCMC'");}
                stage = 1;
                break;
            case 1:
                s_token >> this->num_pois;
                s_token >> this->num_sensors;
                s_token >> this->num_sinks;
                stage = 2;
                break;
            case 2:
                s_token >> this->area_side;
                s_token >> this->sensor_coverage_radius;
                s_token >> this->sensor_communication_radius;
                stage = 3;
                break;
            case 3:
                s_token >> this->random_seed;
                stage = 4;
                break;
            case 4:
                // FIRST-STAGE PARSER
                has_edges = 1;
                stage = this->parse_edge(stage, token);
                break;
            case 5:
                // POI-SENSOR (PS) STAGE
                has_edges = 1;
                stage = this->parse_edge(stage, token);
                break;
            case 6:
                // SENSOR-SENSOR (SS) STAGE
                has_edges = 1;
                stage = this->parse_edge(stage, token);
                break;
            case 7:
                // SENSOR-SINK (SK) STAGE
                has_edges = 1;
                stage = this->parse_edge(stage, token);
                break;
            case 8:
                // END-STAGE
                break;
            default: throw std::runtime_error("FORBIDDEN STAGE!");
        }
        previous = pos+1;
    }
    if (this->num_pois == 0) {throw std::runtime_error("INSTANCE HAS NO POIS!");}
    if (this->num_sensors == 0) {throw std::runtime_error("INSTANCE HAS NO SENSORS!");}
    if (this->num_sinks == 0) {throw std::runtime_error("INSTANCE HAS NO SINKS!");}

    // If we got here and have no edges, we must re-generate this instance.
    if (has_edges == 0) { this->regenerate(); }
}


/** Utility method to the De-Serializer Constructor
 */
int KCMC_Instance::parse_edge(const int stage, const std::string& token){
    /* Instance de-serializer helper method. Parses a single edge */

    // Parse the stage itself
    std::unordered_set<std::string> tags = {"PS", "SS", "SK", "END"};
    if (isin(tags, token)){
        if      (token == "PS"){return 5;}
        else if (token == "SS"){return 6;}
        else if (token == "SK"){return 7;}
        else if (token == "END"){return 8;}
    } else if (stage == 4) {throw std::runtime_error("UNKNOWN TOKEN!");}

    // Parsing at the current stage
    std::stringstream s_token(token);
    int source, target;
    s_token >> source;
    s_token >> target;
    switch (stage) {
        case 5:
            push(this->poi_sensor, source, target);
            push(this->sensor_poi, target, source);
            return 5;
        case 6:
            push(this->sensor_sensor, source, target);
            push(this->sensor_sensor, target, source);
            return 6;
        case 7:
            push(this->sensor_sink, source, target);
            push(this->sink_sensor, target, source);
            return 7;
        case 8: return 8;
        default: throw std::runtime_error("FORBIDDEN STAGE!");
    }
}


/* #####################################################################################################################
 * FUNCTIONAL CLASS SERVICES & METHODS
 */


/** Instance identification utility
 */
std::string KCMC_Instance::key() const {
    /* Returns the settings KEY of the instance */
    std::ostringstream out;
    out << num_pois  <<' '<< num_sensors            <<' '<< num_sinks                   << ';';
    out << area_side <<' '<< sensor_coverage_radius <<' '<< sensor_communication_radius << ';';
    out << random_seed;
    return out.str();
}


/** Instance serializer
 */
std::string KCMC_Instance::serialize() {
    /* Serializes an instance as an string */
    int source, target;

    std::ostringstream out;
    out << "KCMC;" << this->key() << ';';

    // Set the poi-sensor connections
    out << "PS;";
    for (source=0; source<num_pois; source++) {
        for (target=0; target<num_sensors; target++) {
            if (isin(poi_sensor[source], target)) {
                out << source << ' ' << target << ';';  // MUCH SLOWER than iterating the hashmap, but deterministic
            }
        }
    }

    // Set the sensor-sensor connections
    out << "SS;";
    for (source=0; source<num_sensors; source++) {
        for (target=source; target<num_sensors; target++) {
            if (isin(this->sensor_sensor[source], target)) {
                out << source << ' ' << target << ';';  // MUCH SLOWER than iterating the hashmap, but deterministic
            }
        }
    }

    // Set the sensor-sink connections
    out << "SK;";
    for (source=0; source<num_sensors; source++) {
        for (target=0; target<num_sinks; target++) {
            if (isin(this->sensor_sink[source], target)) {
                out << source << ' ' << target << ';';  // MUCH SLOWER than iterating the hashmap, but deterministic
            }
        }
    }

    // Return the out string
    out << "END";
    return out.str();
}


int KCMC_Instance::invert_set(std::unordered_set<int> &source_set, std::unordered_set<int> &target_set) {
    target_set.clear();
    for (int i=0; i<this->num_sensors; i++) {
        if (!isin(source_set, i)) {
            target_set.insert(i);
        }
    }
    return target_set.size();
}


std::string KCMC_Instance::to_sif(std::unordered_set<int> &inactive_sensors) {
    std::string output = "";

    // For each edge Poi-Sensor
    for (const auto &pair : this->poi_sensor) {
        int poi = pair.first;
        for (const auto &sensor : pair.second) {
            if (isin(inactive_sensors, sensor)){continue;}
            output += "p" + std::to_string(poi) + " covg s" + std::to_string(sensor) + "\n";
        }
    }

    // For each edge Sensor-Sensor
    for (const auto &pair : this->sensor_sensor) {
        int sensor1 = pair.first;
        if (isin(inactive_sensors, sensor1)){continue;}
        for (const auto &sensor2 : pair.second) {
            if (isin(inactive_sensors, sensor2)){continue;}
            output += "s" + std::to_string(sensor1) + " conn s" + std::to_string(sensor2) + "\n";
        }
    }

    // For each edge Sensor-Sink
    for (const auto &pair : this->sensor_sink) {
        int sensor = pair.first;
        if (isin(inactive_sensors, sensor)){continue;}
        for (const auto &sink : pair.second) {
            output += "s" + std::to_string(sensor) + " conn k" + std::to_string(sink) + "\n";
        }
    }

    return output;
}
std::string KCMC_Instance::to_sif() {
    std::unordered_set<int> inactive_sensors;
    inactive_sensors.clear();
    return this->to_sif(inactive_sensors);
}


std::string KCMC_Instance::to_cx2(std::unordered_set<int> &inactive_sensors) {

    // Buffers
    std::string status, output = "";
    int id_counter = 0, qt_edges = 0;
    std::vector<Placement> poi_placements(this->num_pois), sensor_placements(this->num_sensors), sink_placements(this->num_sinks);

    // Get the placements
    this->get_placements(poi_placements, sensor_placements, sink_placements);

    // Start the CX2 output with the version and fragment information
    for (const auto &pair : this->poi_sensor) {qt_edges += pair.second.size();}
    for (const auto &pair : this->sensor_sensor) {qt_edges += pair.second.size();}
    for (const auto &pair : this->sensor_sink) {qt_edges += pair.second.size();}
    output += "[\n  { \"CXVersion\": \"2.0\", \"hasFragments\": false },\n  { \"metaData\": [\n    { \"elementCount\": " + std::to_string(this->num_pois + this->num_sensors + this->num_sinks) + ", \"name\": \"nodes\"},\n    { \"elementCount\": " + std::to_string(qt_edges) + ", \"name\": \"edges\"}\n  ] },\n";

    // Add the Attribute Declarations (only for nodes)
    output += "  {\n    \"attributeDeclarations\": [\n      {\n        \"nodes\": {\n          \"node_type\": { \"d\": \"string\" },\n          \"status\": { \"d\": \"string\" }\n        }\n      }\n    ]\n  },\n";

    // Declare all POIs, sensors, and sinks
    output += "  {\n    \"nodes\": [\n";
    for (int poi = 0; poi < this->num_pois; poi++) {
        output += "      { \"id\": " + std::to_string(id_counter+poi) + ", \"v\": { \"name\": \"p" + std::to_string(poi) + "\", \"node_type\": \"POI\", \"status\": \"active\" }, \"x\": " + std::to_string(poi_placements[poi].x) + ", \"y\": " + std::to_string(poi_placements[poi].y) + " },\n";
    }
    id_counter += this->num_pois;
    for (int sensor = 0; sensor < this->num_sensors; sensor++) {
        if (isin(inactive_sensors, sensor)){status = "inactive";} else {status = "active";}
        output += "      { \"id\": " + std::to_string(id_counter+sensor) + ", \"v\": { \"name\": \"s" + std::to_string(sensor) + "\", \"node_type\": \"SENSOR\", \"status\": \"" + status + "\" }, \"x\": " + std::to_string(sensor_placements[sensor].x) + ", \"y\": " + std::to_string(sensor_placements[sensor].y) + " },\n";
    }
    id_counter += this->num_sensors;
    for (int sink = 0; sink < this->num_sinks; sink++) {
        output += "      { \"id\": " + std::to_string(id_counter+sink) + ", \"v\": { \"name\": \"k" + std::to_string(sink) + "\", \"node_type\": \"SINK\", \"status\": \"active\" }, \"x\": " + std::to_string(sink_placements[sink].x) + ", \"y\": " + std::to_string(sink_placements[sink].y) + " },\n";
    }
    id_counter += this->num_sinks;
    output = output.substr(0, output.size() - 2) + "\n    ]\n  },\n  {\n    \"edges\": [\n";  // Remove the trailing coma, close the nodes list and start the edges list

    // For each edge Poi-Sensor
    for (const auto &pair : this->poi_sensor) {
        int poi = pair.first;
        for (const auto &sensor : pair.second) {
            output += "      { \"id\": " + std::to_string(id_counter) + ", \"s\": " + std::to_string(poi) + ", \"t\": " + std::to_string(sensor+this->num_pois) + " },\n";
            id_counter += 1;
        }
    }

    // For each edge Sensor-Sensor
    for (const auto &pair : this->sensor_sensor) {
        int sensor1 = pair.first;
        if (isin(inactive_sensors, sensor1)){continue;}
        for (const auto &sensor2 : pair.second) {
            if (isin(inactive_sensors, sensor2)){continue;}
            output += "      { \"id\": " + std::to_string(id_counter) + ", \"s\": " + std::to_string(sensor1+this->num_pois) + ", \"t\": " + std::to_string(sensor2+this->num_pois) + " },\n";
            id_counter += 1;
        }
    }

    // For each edge Sensor-Sink
    for (const auto &pair : this->sensor_sink) {
        int sensor = pair.first;
        if (isin(inactive_sensors, sensor)){continue;}
        for (const auto &sink : pair.second) {
            output += "      { \"id\": " + std::to_string(id_counter) + ", \"s\": " + std::to_string(sensor+this->num_pois) + ", \"t\": " + std::to_string(sink+this->num_pois+this->num_sensors) + " },\n";
            id_counter += 1;
        }
    }
    output = output.substr(0, output.size() - 2) + "\n    ]\n  },\n  {\n    \"status\": [\n      { \"error\": \"\", \"success\": true }\n    ]\n  }";  // Remove the trailing coma, close the edges list and add the ending


    return output+"\n]";
}
std::string KCMC_Instance::to_cx2() {
    std::unordered_set<int> inactive_sensors;
    inactive_sensors.clear();
    return this->to_cx2(inactive_sensors);
}


bool KCMC_Instance::validate(const bool raise, const int k, const int m,
                             std::unordered_set<int> &inactive_sensors,
                             std::unordered_set<int> &k_used_sensors,
                             std::unordered_set<int> &m_used_sensors,
                             bool use_greedy) {
    int valid;
    std::string key = this->key(), notice;
    if (use_greedy) { notice = "GREEDY"; } else { notice = "DINITZ"; }

    // Check validity, recovering the used sensors for K coverage and M connectivity
    try {
        valid = this->k_coverage(k, inactive_sensors, k_used_sensors);
        if (valid != INVALID_PREVIOUS_POI) { throw std::runtime_error("INVALID INSTANCE " + key + "! (INSUFFICIENT COVERAGE '" + notice + "')"); }
    }
    catch (const std::exception &exc) {
        if (raise) {throw std::runtime_error("INVALID INSTANCE " + key + "! (INSUFFICIENT COVERAGE '" + notice + "')");}
        else {return false;}
    }

    try {
        if (use_greedy) {
            valid = this->m_connectivity_greedy(m, inactive_sensors, m_used_sensors);
        } else {
            valid = this->m_connectivity(m, inactive_sensors, m_used_sensors);
        }
        if (valid != INVALID_PREVIOUS_POI) { throw std::runtime_error("INVALID INSTANCE " + key + "! (INSUFFICIENT CONNECTIVITY '" + notice + "')"); }
    }
    catch (const std::exception &exc) {
        if (raise) {throw std::runtime_error("INVALID INSTANCE " + key + "! (INSUFFICIENT CONNECTIVITY '" + notice + "')");}
        else {return false;}
    }
    return true;
}


bool KCMC_Instance::validate(const bool raise, const int k, const int m,
                             std::unordered_set<int> &inactive_sensors,
                             std::unordered_set<int> &k_used_sensors,
                             std::unordered_set<int> &m_used_sensors) {
    return this->validate(raise, k, m, inactive_sensors, k_used_sensors, m_used_sensors, false);
}


bool KCMC_Instance::validate(const bool raise, const int k, const int m,
                             std::unordered_set<int> &inactive_sensors,
                             bool use_greedy) {
    // Prepare the ignored results buffer and the empty set of inactive sensors
    std::unordered_set<int> ignored;
    return this->validate(raise, k, m, inactive_sensors, ignored, ignored, use_greedy);
}


bool KCMC_Instance::validate(const bool raise, const int k, const int m, bool use_greedy) {
    // Prepare the ignored results buffer and the empty set of inactive sensors
    std::unordered_set<int> ignored;
    return this->validate(raise, k, m, ignored, ignored, ignored, use_greedy);
}


bool KCMC_Instance::validate(const bool raise, const int k, const int m) {
    // Prepare the ignored results buffer and the empty set of inactive sensors
    std::unordered_set<int> ignored;
    return this->validate(raise, k, m, ignored, ignored, ignored, false);
}
