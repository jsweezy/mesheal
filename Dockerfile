# syntax=docker/dockerfile:1
# Mesheal on Linux (Ubuntu 24.04)
#
#   docker build -t mesheal .                        # build, run all tests, produce the CLI image
#   docker build --target build -t mesheal-build .   # build + test only (what CI runs)
#   docker run --rm -v "$PWD:/data" mesheal model.stl --fix-all
#
# The image runs the command-line interface only. See DOCKER.md.

ARG UBUNTU_VERSION=24.04

# ------------------------------------------------------------------------------
# deps: compiler, CMake and the system libraries Mesheal links against.
# CGAL itself is downloaded by CMake: Mesheal needs CGAL 6.2+ and Ubuntu 24.04 ships 5.6.
# ------------------------------------------------------------------------------
FROM ubuntu:${UBUNTU_VERSION} AS deps

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        git \
        ninja-build \
        libboost-dev \
        libgmp-dev \
        libmpfr-dev \
        libtbb-dev \
        libgl1-mesa-dev \
        libwayland-dev \
        libxkbcommon-dev \
        xorg-dev \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------------------------
# build: configure, compile (desktop app included, so Linux GUI builds are checked too)
# and run the full test suite (a failing test fails the build)
# ------------------------------------------------------------------------------
FROM deps AS build

# One compile job by default: CGAL-heavy files need ~2 GB each and Docker Desktop VMs are often 4 GB.
ARG JOBS=1
ARG TEST_JOBS=4

WORKDIR /src
COPY . .

# The build folder lives in a BuildKit cache mount, so it survives between image builds:
# dependencies are downloaded once and only changed files are recompiled.
RUN --mount=type=cache,target=/src/build,id=mesheal-build \
    cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build -j "${JOBS}" \
    && ctest --test-dir build -j "${TEST_JOBS}" --output-on-failure --timeout 600 \
    && install -D -m 755 build/src/app/mesheal-cli /out/mesheal

# ------------------------------------------------------------------------------
# runtime: small image with the command-line-only executable (mesheal-cli, no OpenGL/X11)
# ------------------------------------------------------------------------------
FROM ubuntu:${UBUNTU_VERSION} AS runtime

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        libgmp10 \
        libmpfr6 \
        libtbb12 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=build /out/mesheal /usr/local/bin/mesheal

WORKDIR /data
ENTRYPOINT ["mesheal"]
CMD ["--help"]
