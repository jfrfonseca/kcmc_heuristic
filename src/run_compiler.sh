# BUILD IN TWO STEPS
rm -rf /tmp/build
mkdir /tmp/build
cd /tmp/build
cmake -S /app -B .
cmake --build .

# COPY THE OUTPUT EXECUTABLES TO THE BUILDS DIRECTORY
rm -rf /app/builds/*
cp instance_generator /app/builds
cp instance_validator /app/builds
cp instance_visualizer /app/builds
cp isomorphism_validator /app/builds
cp heuristics /app/builds
chmod +x /app/builds/*