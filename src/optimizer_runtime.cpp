

// STDLib dependencies
#include <vector>     // vector
#include <csignal>    // SIGINT and other signals
#include <queue>      // queue
#include <iostream>   // cin, cout, endl
#include <sstream>    // ostringstream
#include <chrono>     // time functions
#include <iomanip>    // setfill, setw
#include <cstring>    // strcpy
#include <algorithm>  // copy, fill, transform

// Dependencies from this package
#include "kcmc_instance.h"  // KCMC Instance class headers, exit_signal_handler


/* #####################################################################################################################
 * RUNTIME
 * */


void printout_short(
    KCMC_Instance *instance,
    const int kcmc_k,
    const int kcmc_m,
    const std::string pre_heuristic,
    const long duration,
    std::unordered_set<int> &solution
) {
    // Prepare the output buffer
    std::ostringstream out;
    std::vector<int> used_sensors;

    // Sort the used sensors in the solution
    used_sensors.resize((int)solution.size());
    std::copy(solution.begin(), solution.end(), used_sensors.begin());
    std::sort(used_sensors.begin(), used_sensors.end());

    // Print a line with:
    // - The key of the instance
    // - The name of the current operation
    // - The amount of microsseconds the method needed to run
    // - The number of used installation spots
    // - The resulting map of the instance, as a binary of num_sensors bits
    out << instance->key() << "\t" << kcmc_k << "\t" << kcmc_m
        << "\t" << pre_heuristic
        << "\t" << duration
        << "\t" << solution.size()
        << "\t";
    for (const int i : used_sensors) {out << " " << i;}
    // Flush
    std::cout << out.str() << std::endl;
}


 void help(int argc, char* const argv[]) {
    std::cout << "RECEIVED LINE (" << argc << "): ";
    for (int i=0; i<argc; i++) {std::cout << argv[i] << " ";}
    std::cout << std::endl;
    std::cout << "Please, use the correct input for the KCMC instance heuristic optimizer:" << std::endl << std::endl;
    std::cout << "./optimizer <p> <s> <k> <area_s> <cov_v> <com_r> <seed> <kcmc_k> <kcmc_m> <s>*" << std::endl;
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

    // Registers the signal handlers
    signal(SIGINT, exit_signal_handler);
    signal(SIGABRT, exit_signal_handler);
    signal(SIGTERM, exit_signal_handler);

    // The signals below are undefined in windows, exist only in linux
#ifndef _WIN32
    signal(SIGALRM, exit_signal_handler);
    signal(SIGSTOP, exit_signal_handler);
    signal(SIGKILL, exit_signal_handler);
#endif

    // Buffers
    int p, s, k, area_s, cov_v, com_r, kcmc_k, kcmc_m, arg_index;
    long long seed;
    std::unordered_set<int> inactive_sensors, solution;
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

    // Prepare the clock buffers
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    long duration;

    // Print the header
    printf("Instance\tKCMC_K\tKCMC_M\tPreHeuristic\tRuntime\tNumSensorsUsed\tUsedSensors\n");

    // GREEDY =====================================================================================

    // GREEDY DKOV
    solution.clear();
    start = std::chrono::high_resolution_clock::now();
    instance->greedy_dkov(kcmc_k, kcmc_m, inactive_sensors, solution);
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    printout_short(instance, kcmc_k, kcmc_m, "greedy_dkov", duration, solution);

    // GREEDY REUSE
    solution.clear();
    start = std::chrono::high_resolution_clock::now();
    instance->greedy_reuse(kcmc_k, kcmc_m, inactive_sensors, solution);
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    printout_short(instance, kcmc_k, kcmc_m, "greedy_reuse", duration, solution);

    // GREEDY BREADTH
    solution.clear();
    start = std::chrono::high_resolution_clock::now();
    instance->greedy_breadth(kcmc_k, kcmc_m, inactive_sensors, solution);
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    printout_short(instance, kcmc_k, kcmc_m, "greedy_breadth", duration, solution);

    // DINITZ =====================================================================================

    // DINITZ DKOV
    solution.clear();
    start = std::chrono::high_resolution_clock::now();
    instance->dinitz_dkov(kcmc_k, kcmc_m, inactive_sensors, solution);
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    printout_short(instance, kcmc_k, kcmc_m, "dinitz_dkov", duration, solution);

    // DINITZ REUSE
    solution.clear();
    start = std::chrono::high_resolution_clock::now();
    instance->dinitz_reuse(kcmc_k, kcmc_m, inactive_sensors, solution);
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    printout_short(instance, kcmc_k, kcmc_m, "dinitz_reuse", duration, solution);

    return 0;
}
