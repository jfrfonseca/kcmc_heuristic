/** KCMC_INSTANCE_HANDLING.cpp
 * Implementation of non-payload services of the KCMC instance object headers
 * Jose F. R. Fonseca
 */


// STDLib dependencies
#include <sstream>    // ostringstream
#include <random>     // mt19937, uniform_real_distribution
#include <algorithm>  // std::find

// Dependencies from this package
#include "kcmc_instance.h"  // KCMC Instance class headers


// CONSTRUCTORS ########################################################################################################


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

    // If we got here and have no edges, we must re-generate this instance
    if (has_edges == 0) { this->regenerate(); }
}


// CONSTRUCTOR SUPPORT METHODS #########################################################################################


/** RANDOM-INSTANCE COMPONET PLACEMENTS (RE)GENERATOR
 * Generates the instance's components placements, assuming the instance already have all main attributes.
 * THE PLACEMENTS WILL BE USED TO COMPUTE THE EDGES
 */
void KCMC_Instance::get_placements(Placement *pl_pois, Placement *pl_sensors, Placement *pl_sinks, bool push) {

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
void KCMC_Instance::get_placements(Placement *pl_pois, Placement *pl_sensors, Placement *pl_sinks) {
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
    Placement pl_pois[this->num_pois], pl_sensors[this->num_sensors], pl_sinks[this->num_sinks];

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


/** Utility method to the De-Serializer Constructor
 * Parses an edge from a serialized instance
 */
int KCMC_Instance::parse_edge(const int stage, const std::string& token){
    /* Instance de-serializer helper method. Parses a single edge */

    // Parse the stage itself
    std::unordered_set<std::string> tags = {"PI", "II", "IS", "END"};
    if (isin(tags, token)){
        if      (token == "PI"){return 5;}
        else if (token == "II"){return 6;}
        else if (token == "IS"){return 7;}
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


// INSTANCE SERIALIZERS ################################################################################################


/** Instance SEED - Arguments for contructor
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
    out << "PI;";
    for (source=0; source<num_pois; source++) {
        for (target=0; target<num_sensors; target++) {
            if (isin(poi_sensor[source], target)) {
                out << source << ' ' << target << ';';  // MUCH SLOWER than iterating the hashmap, but deterministic
            }
        }
    }

    // Set the sensor-sensor connections
    out << "II;";
    for (source=0; source<num_sensors; source++) {
        for (target=source; target<num_sensors; target++) {
            if (isin(this->sensor_sensor[source], target)) {
                out << source << ' ' << target << ';';  // MUCH SLOWER than iterating the hashmap, but deterministic
            }
        }
    }

    // Set the sensor-sink connections
    out << "IS;";
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