/*
 * KCMC Instance generator
 * Generates 1+ instances for the KCMC problem. INSTANCES MIGHT NOT BE VALID FOR ALL VALUES OF K AND M
 */


// STDLib Dependencies
#if defined(_WIN32)
#include <process.h>
#define getpid _getpid  //getpid on Windows
#else
#include <unistd.h>  //getpid on Linux
#endif
#include <string>    // to_string
#include <iostream>  // cin, cout, endl, printf, fprintf
#include <cstdlib>   // atoi, atoll, srand, rand
#include <ctime>     // time
#include <unordered_set>  // unordered_set

// Dependencies from this package
#include "kcmc_instance.h"


/* #####################################################################################################################
 * RUNTIME
 * */


void help(int argc, char* const argv[]) {
    std::cout << "RECEIVED LINE (" << argc << "): ";
    for (int i=0; i<argc; i++) {std::cout << argv[i] << " ";}
    std::cout << std::endl;
    std::cout << "Please, use the correct input for the KCMC instance generator:" << std::endl << std::endl;
    std::cout << "./instance_generator <p> <s> <k> <area_s> <cov_v> <com_r> <seed>+" << std::endl;
    std::cout << "  where:" << std::endl << std::endl;
    std::cout << "p > 0 is the number of POIs to be randomly generated" << std::endl;
    std::cout << "s > 0 is the number of Sensors to be generated" << std::endl;
    std::cout << "k > 0 is the number of Sinks to be generated. If n=1, the sink will be placed at the center of the area" << std::endl;
    std::cout << "area > 0.0 is the int length of the square area where features will be placed" << std::endl;
    std::cout << "cov_r > 0.0 is the int radius around a Sensor where it can cover POIs" << std::endl;
    std::cout << "com_r > 0.0 is the int radius around a Sensor where it can communicate with other Sensors or Sinks" << std::endl << std::endl;
    std::cout << "seed is an integer number that is used as seed of the PRNG." << std::endl;
    std::cout << "++ If more than one seed is provided, many instances will be generated" << std::endl;
    std::cout << "++ If a single instance is provided, its de-serialization will be tested" << std::endl;
    std::cout << "There is also fail-safe mode when the seed is 0" << std::endl;
    std::cout << "./instance_generator <p> <s> <k> <area_s> <cov_v> <com_r> 0 <kcmc_k> <kcmc_m>" << std::endl;
    std::cout << "In this case, the program will generate an instance using the fail-safe mode with the specified <kcmc_k> and <kcmc_m>" << std::endl;
    exit(0);
}



int main(int argc, char* const argv[]) {
    if (argc < 7) {help(argc, argv);}

    /* Prepare Buffers */
    bool success;
    int i, num_pois, num_sensors, num_sinks, area_side, coverage_radius, communication_radius, kcmc_k, kcmc_m;
    long long random_seed, previous_seed;
    std::unordered_set<int> emptyset, ignoredset;

    /* Parse CMD SETTINGS */
    num_pois    = atoi(argv[1]);
    num_sensors = atoi(argv[2]);
    num_sinks   = atoi(argv[3]);
    area_side   = atoi(argv[4]);
    coverage_radius = atoi(argv[5]);
    communication_radius = atoi(argv[6]);

    // Get a random previous seed
    srand(time(NULL) + getpid());  // Diferent seed in each run for each process
    previous_seed = 100000000 + std::abs((rand() % 100000000)) + std::abs((rand() % 100000000));  // LARGE but random-er number

    for (i=7; i<argc; i++) {
        random_seed = atoll(argv[i]);

        // MODE WHERE WE HAVE TO FIND A VALID INSTANCE (no seed given)
        if (random_seed == 0) {

            // Read K and M
            kcmc_k = atoi(argv[i+1]);
            kcmc_m = atoi(argv[i+2]);
            i += 2;

            // Try many times until get a valid instance
            for (int attempt = 0; attempt < MAX_GENERATION_ATTEMPTS; attempt++) {  // MANY ATTEMPTS!

                // Update the random seed
                random_seed = previous_seed + std::abs((rand() % 100000)) + 7;

                // Try with the current random seed
                success = false;
                auto *instance = new KCMC_Instance(num_pois, num_sensors, num_sinks,
                                                   area_side, coverage_radius, communication_radius,
                                                   random_seed);

                // If the instance is valid, stop trying and print it
                if (instance->validate(false, kcmc_k, kcmc_m, USE_GREEDY)) {
                    success = true;
                    printf("KCMC;%s;END | (K%dM%d)\n", instance->key().c_str(), kcmc_k, kcmc_m);
                    attempt = 2*MAX_GENERATION_ATTEMPTS;
                    break;
                }
            }

            // Raise an error if we were unable to generate a valid instance
            if (!success) {
                throw std::runtime_error(
                    "UNABLE TO GENERATE VALID INSTANCE WITH PARAMETERS " +
                    std::to_string(num_pois) + " " +
                    std::to_string(num_sensors) + " " +
                    std::to_string(num_sinks) + " " +
                    std::to_string(area_side) + " " +
                    std::to_string(coverage_radius) + " " +
                    std::to_string(communication_radius) + " 0 " +
                    std::to_string(kcmc_k) + " " +
                    std::to_string(kcmc_m));
                }

        }

        // MODE WHERE A SEED IS PROVIDED AND WE MUST COMPUTE THE INSTANCE FROM IT AND TEST IT
        else {
            try {
                auto *instance = new KCMC_Instance(num_pois, num_sensors, num_sinks,
                                                   area_side, coverage_radius, communication_radius,
                                                   random_seed);
                printf("%s\n", instance->serialize().c_str());

                /* FOR VERIFICATION */
                if (argc == 8) {
                    std::string serialized_instance = instance->serialize();
                    auto *new_instance = new KCMC_Instance(serialized_instance);
                    if (new_instance->serialize() == instance->serialize()) {
                        printf("%s\nEQUAL\n", new_instance->serialize().c_str());
                    } else { throw std::runtime_error("NOT EQUAL!"); }
                }

            } catch (const std::exception &exc) {
                if (argc == 8) { throw exc; }  // Only throw the exception if we're in DEBUG mode
                fprintf(stderr, "%lld\t%s\n", random_seed, exc.what());
            }
        }
    }

    return (0);
}
