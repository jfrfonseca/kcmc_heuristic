# USING DEBIAN 13.7 "Trixie" OS
FROM python:trixie

# Install GNU PARALLEL & CLEANUP
RUN apt -y update \
 && apt install -y parallel \
 && apt -y autoremove && rm -rf /var/lib/apt/lists/*

# Add in the executables
COPY builds/instance_generator /app/instance_generator

# Work directory configurations
WORKDIR '/app'
