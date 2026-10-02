# USING DEBIAN 13.7 "Trixie" OS
FROM gcc:16.2-trixie

# INSTALL DEPS & CLEANUP
RUN apt -y update \
 && apt install -y cmake \
 && apt -y autoremove && rm -rf /var/lib/apt/lists/*

# USER CODE MUST BE PLACED IN /app
WORKDIR /app
