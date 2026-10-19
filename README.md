
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


## Instance Validator

Run the instance validator with the following commands:

```bash
docker compose run instance_validator
docker compose down
```

The docker-compose down command cleans up the containers to avoid errors in further executions due to persistent files.

The instances validator reads the instances from the file `instances.csv`, validates them according to the KCMC problem constraints, and outputs the results to the standard output.

It uses both the greedy and the dinitz validation methods to check the feasibility of the instances. It also runs the same instance for all range of values between 1 and k and all range between 1 and m, and every combination of those two. Including the nonsensical m>k combinations, that are still valid for solving the problem although have no practical use.

The whole point is to make sure that the generated instances are valid - the KCMC problem can be solved on those.

The validator also runs the isomorphism validator, to make sure we have no pairs of instances that are isomorphic.
This is important to guarantee we are testing on many diferent instances.
The no-possible-isomorphism validation is easy - we count how many vertices of each degree a instance have (separating pois, sensors and sinks). The degree of a vertice is the number of edges connected to it. The quantity of vertices of each degree in an instance is its profile. If there are no two instances that have the same profile, it means that there is no possibility of having any pair of isomorphic instances.
It is important to say that two instances having the same profile does not necessarily mean they are isomorphic, but if the profiles are different, it necessarily means the instances are not isomorphic.


## Heuristics

Run the heuristics locally with the following commands:

```bash
docker compose run heuristics
docker compose down
```

## Fix-And-Optimize (Locally)

Run the fix-and-optimize locally with the following commands:

```bash
docker compose run fix_and_optimize
docker compose down
```

## In production, the heuristics should be run directly in a Debian-based LINUX environment, with the following command:

At the root of the project directory, run the following command to execute the heuristics:
```bash
sed -e 's/^KCKC//' \
    -e 's/END$//' \
    -e 's/;/ /g' \
    -e 's/[|()]//g' \
    -e 's/[KCMEND]/ /g' \
    -e 's/[[:space:]]\+/ /g' -e 's/^ //; s/ $//' \
    data/instances.csv \
    | xargs -r -L1 builds/heuristics \
    > data/heuristics_results.tsv
```





## APPENDIX
General Information

### Login to GIT
Using a personal access token (PAT), you can issue Git commands after setting up with the following command:
```bash
git remote set-url origin https://<USERNAME>:<PERSONAL_ACCESS_TOKEN>@github.com/<USERNAME>/kcmc_heuristic.git
```
