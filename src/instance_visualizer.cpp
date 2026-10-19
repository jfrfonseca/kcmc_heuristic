/*
 * KCMC Instance validator
 * Validates the instances for the KCMC problem. Operates like an UNIX filter
 */


// STDLib Dependencies
#include <iostream>  // cin, cout, endl, printf
#include <string>    // to_string
#include <fstream>   // ofstream

// Dependencies from this package
#include "kcmc_instance.h"


/* #####################################################################################################################
 * RUNTIME
 * */


 void help(int argc, char* const argv[]) {
    std::cout << "RECEIVED LINE (" << argc << "): ";
    for (int i=0; i<argc; i++) {std::cout << argv[i] << " ";}
    std::cout << std::endl;
    std::cout << "Please, use the correct input for the KCMC instance evaluator:" << std::endl << std::endl;
    std::cout << "./instance_validator <p> <s> <k> <area_s> <cov_v> <com_r> <seed> <kcmc_k> <kcmc_m> <s>*" << std::endl;
    std::cout << "  where:" << std::endl << std::endl;
    std::cout << "p > 0 is the number of POIs to be randomly generated" << std::endl;
    std::cout << "  if p is in one of the special cases, the corresponding instance will be used and all further parameters will be ignored" << std::endl;
    std::cout << "  if p=0, the 'Sao Paulo' instance will be used: 'KCMC;1 6 1;10 4 6;0;PS;0 0;0 2;SS;0 1;0 4;2 3;4 5;3 1;SK;1 0;5 0;' for K=2 M=1" << std::endl;
    std::cout << "  if p<0 the value of p will be treated as its absolute value and the inactive sensors will be inverted, treated as the only active sensors" << std::endl;
    std::cout << "s > 0 is the number of Sensors to be generated" << std::endl;
    std::cout << "  if s<0 the value of s will be treated as its absolute value and the data directory will be one directory above the current one" << std::endl;
    std::cout << "k > 0 is the number of Sinks to be generated. If n=1, the sink will be placed at the center of the area" << std::endl;
    std::cout << "area > 0.0 is the int length of the square area where features will be placed" << std::endl;
    std::cout << "cov_r > 0.0 is the int radius around a Sensor where it can cover POIs" << std::endl;
    std::cout << "com_r > 0.0 is the int radius around a Sensor where it can communicate with other Sensors or Sinks" << std::endl << std::endl;
    std::cout << "seed is an integer number that is used as seed of the PRNG." << std::endl;
    std::cout << "s* is an optional list of inactive sensors (by their indices)" << std::endl;
    exit(0);
}


int main(int argc, char* const argv[]) {
    if ((argc != 2) && (argc < 8)) { help(argc, argv); }

    // Buffers
    bool should_invert = false;
    std::string prefix = "", data_root = "";
    int p, s, k, area_s, cov_v, com_r, arg_index, sensor;
    long long seed;
    std::unordered_set<int> inactive_sensors, inverted_inactive_sensors;
    KCMC_Instance* instance = nullptr;

    p = atoi(argv[1]);
    if (p == 0) {
        printf("Using the 'Sao Paulo' instance\n");
        instance = new KCMC_Instance("KCMC;1 6 1;10 4 6;0;PS;0 0;0 2;SS;0 1;0 4;2 3;4 5;1 3;SK;1 0;5 0;");

    } else {
        if (p < 0) {
            p = -1*p;
            should_invert = true;
            prefix = "SOL_";
        }
        s = atoi(argv[2]);
        if (s < 0) {
            s = -1*s;
            data_root = "..";
        }
        k = atoi(argv[3]);
        if (p <= 0 || s <= 0 || k <= 0) {throw std::invalid_argument("Invalid Pois, Sensors or Sinks parameters.");}
        area_s = atoi(argv[4]);
        cov_v = atoi(argv[5]);
        com_r = atoi(argv[6]);
        if (area_s <= 0 || cov_v <= 0 || com_r <= 0) {throw std::invalid_argument("Invalid area, coverage or communication parameters.");}
        seed = atoll(argv[7]);

        // Parse the inactive sensors
        inactive_sensors.clear();
        if (argc > 8) {for (arg_index=8; arg_index<argc; arg_index++){inactive_sensors.insert(atoi(argv[arg_index]));}}
        if (should_invert && (!inactive_sensors.empty())) {
            for (sensor=0; sensor < instance->num_sensors; sensor++) {
                if (!isin(inactive_sensors, sensor)) {
                    inverted_inactive_sensors.insert(sensor);
                }
            }
            //inactive_sensors = inverted_inactive_sensors;
        }

        // De-serialize the actual instance
        instance = new KCMC_Instance(
            p, s, k,
            area_s, cov_v, com_r,
            seed
        );
    }

    // Convert the instance to cx2 format and save it in a file
    std::string file_name = data_root + "/data/cx2/" + prefix + "kcmc_instance_"
                          + std::to_string(p) + "_" + std::to_string(s) + "_" + std::to_string(k)
                          + "_" + std::to_string(area_s) + "_" + std::to_string(cov_v) + "_" + std::to_string(com_r)
                          + "_" + std::to_string(seed) + ".cx2";
    std::ofstream cx2_file(file_name);
    if (!cx2_file.is_open()) {throw std::runtime_error("Unable to open output file: " + file_name);}
    cx2_file << instance->to_cx2(inactive_sensors);
    cx2_file.close();

    return 0;
}

