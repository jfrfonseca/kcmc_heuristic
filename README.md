
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

