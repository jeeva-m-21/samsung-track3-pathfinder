# Dockerfile for Samsung Hackathon Track 3 "Path Finder"
# Team: VITC_SingleThread
# Image: Ubuntu 22.04 LTS with C++17 build environment

FROM ubuntu:22.04

# Prevent interactive prompts during apt installation
ENV DEBIAN_FRONTEND=noninteractive

# Install essential build tools (g++, make, time, python3)
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    g++ \
    make \
    time \
    python3 \
    ca-certificates \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy repository source code
COPY . /app

# Clean and build production C++ binary (outputs to src/cpp/build/main, src/main, and ./main)
RUN make clean && make -j$(nproc)

# Verify unit tests (1,233 C++ assertions)
RUN make test

# Set default entrypoint
ENTRYPOINT ["./main"]
CMD ["--help"]
