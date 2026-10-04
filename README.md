
# KCMC Heuristic




## Rebuilding the containers

It might be necessary to rebuild the containers if there are changes in the Dockerfiles, but usually not in the source code because most of the source code is dynamically mounted into the containers as volumes.
Normally, rebuilding the containers is only necessary when there are changes in dependencies.

To rebuild the Docker containers, use the following command:
```bash
docker compose build compiler instance_generator
```


## Compiler

Run the compiler with the following commands:

```bash
docker compose run compiler
docker compose down
```

The docker-compose down command cleans up the containers to avoid errors in further executions due to persistent files.


## Instance Generator

Run the instance generator with the following commands:

```bash
docker compose run instance_generator
docker compose down
```

The docker-compose down command cleans up the containers to avoid errors in further executions due to persistent files.

The first line of the file instance_classes.csv must be sacrificial, with invalid values.
It will result in a first line of the file instances.csv that will be automatically removed.

You must also verify the printed results to check that all generated instances are unique (all lines are different).
That can be seen by the first value printed after the string "LINES COUNT", that must always be 1


## Instance Evaluator

Run the instance evaluator with the following commands:

```bash
docker compose run instance_evaluator
docker compose down
```

The docker-compose down command cleans up the containers to avoid errors in further executions due to persistent files.

The instances evaluator reads the instances from the file `instances.csv`, evaluates them according to the KCMC problem constraints, and outputs the results to the standard output.

It uses both the greedy and the dinitz validation methods to check the feasibility of the instances. It also runs the same instance for all range of values between 1 and k and all range between 1 and m, and every combination of those two. Including the nonsensical m>k combinations, that are still valid for solving the problem although have no practical use.


## APPENDIX
General Information

### Login to GIT
Using a personal access token (PAT), you can issue Git commands after setting up with the following command:
```bash
git remote set-url origin https://<USERNAME>:<PERSONAL_ACCESS_TOKEN>@github.com/<USERNAME>/kcmc_heuristic.git
```
