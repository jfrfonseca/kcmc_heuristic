/*
 * KCMC Instance validator
 * Validates the instances for the KCMC problem. Operates like an UNIX filter
 */


// STDLib Dependencies
#include <iostream>  // cin, cout, endl, printf

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
    std::cout << "s > 0 is the number of Sensors to be generated" << std::endl;
    std::cout << "k > 0 is the number of Sinks to be generated. If n=1, the sink will be placed at the center of the area" << std::endl;
    std::cout << "area > 0.0 is the int length of the square area where features will be placed" << std::endl;
    std::cout << "cov_r > 0.0 is the int radius around a Sensor where it can cover POIs" << std::endl;
    std::cout << "com_r > 0.0 is the int radius around a Sensor where it can communicate with other Sensors or Sinks" << std::endl << std::endl;
    std::cout << "seed is an integer number that is used as seed of the PRNG." << std::endl;
    std::cout << "kcmc_k > 0 is the K parameter for the KCMC problem" << std::endl;
    std::cout << "kcmc_m > 0 is the M parameter for the KCMC problem" << std::endl;
    std::cout << "s* is an optional list of inactive sensors (by their indices)" << std::endl;
    exit(0);
}


int main(int argc, char* const argv[]) {
    if ((argc != 2) && (argc < 9)) { help(argc, argv); }

    // Buffers
    int p, s, k, area_s, cov_v, com_r, kcmc_k, kcmc_m, arg_index, current_k, current_m;
    long long seed;
    std::unordered_set<int> inactive_sensors;
    KCMC_Instance* instance = nullptr;

    p = atoi(argv[1]);
    if (p == 0) {
        printf("Using the 'Sao Paulo' instance for K=2 M=1\n");
        kcmc_k = 2;
        kcmc_m = 1;
        instance = new KCMC_Instance("KCMC;1 6 1;10 4 6;0;PS;0 0;0 2;SS;0 1;0 4;2 3;4 5;1 3;SK;1 0;5 0;");

    } else {
        s = atoi(argv[2]);
        k = atoi(argv[3]);
        if (p <= 0 || s <= 0 || k <= 0) {throw std::invalid_argument("Invalid Pois, Sensors or Sinks parameters.");}
        area_s = atoi(argv[4]);
        cov_v = atoi(argv[5]);
        com_r = atoi(argv[6]);
        if (area_s <= 0 || cov_v <= 0 || com_r <= 0) {throw std::invalid_argument("Invalid area, coverage or communication parameters.");}
        seed = atoll(argv[7]);
        kcmc_k = atoi(argv[8]);
        kcmc_m = atoi(argv[9]);
        if (kcmc_k <= 0 || kcmc_m <= 0 || s <= kcmc_k) {throw std::invalid_argument("Invalid KCMC_K or KCMC_M parameters.");}

        // Parse the inactive sensors
        if (argc > 9) {for (arg_index=10; arg_index<argc; arg_index++){inactive_sensors.insert(atoi(argv[arg_index]));}}

        // De-serialize the actual instance
        instance = new KCMC_Instance(
            p, s, k,
            area_s, cov_v, com_r,
            seed
        );
    }

    // Validate it to K and M constraints, and the inactive sensors
    // Validate for the entire range, and throw an error if any configuration is invalid
    for (current_k = 1; current_k <= kcmc_k; current_k++) {
        for (current_m = 1; current_m <= kcmc_m; current_m++) {
            // Run even in useless M >= K configurations, that are nonsensical but valid

            // Try using the greedy validation method
            if (instance->validate(true, current_k, current_m, inactive_sensors, true)) {
                printf("KCMC;%s;END | (K%dM%d) | GREEDY:OK\n", instance->key().c_str(), current_k, current_m);
            }

            // Try using the dinitz validation method
            if (instance->validate(true, current_k, current_m, inactive_sensors, false)) {
                printf("KCMC;%s;END | (K%dM%d) | DINITZ:OK\n", instance->key().c_str(), current_k, current_m);
            }
        }
    }

    return 0;
}

