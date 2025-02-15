# Fetch ubuntu image
FROM ubuntu:22.04

# gcovr wants to know your timezone, and to do that it needs the tzdata package
ARG DEBIAN_FRONTEND=noninteractive

# Selecting UTC time. Could be changed to Toronto time if we care enough. 
ENV TZ=Etc/UTC

# Install prerequisites
RUN apt-get update
RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-utils \
    curl \
    cmake \
    build-essential \
    cppcheck \
    ninja-build \
    gcc \
    g++-multilib \
    locales \
    make \
    sudo \
    tzdata \
    gcovr \
    wget \
    clang-format-14 \
    clang-tidy-14 \
    pkg-config \
    cpputest \
    git \
    gdb \
    protobuf-compiler \
    libprotobuf-dev \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# get the toolchain
RUN wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/10-2020q4/gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2 -O gcc-arm-none-eabi.tar.bz2

# unpack the archive to a neatly named target directory
RUN mkdir gcc-arm-none-eabi && tar xjfv gcc-arm-none-eabi.tar.bz2 -C gcc-arm-none-eabi --strip-components 1
# remove the archive
RUN rm gcc-arm-none-eabi.tar.bz2

# add the tools to the path
ENV PATH="/gcc-arm-none-eabi/bin:${PATH}"

# use the same directory naming as the vscode dev container when building from the command line. use /workspaces/cproject
# The issue is that you’re changing the path names that appear in the compile_commands.json, the CMake output files, and the ELF file. 
# This will screw up IntelliSense and incremental builds when you go back into the container.
VOLUME ["/workspaces/nucleo-g474-template"]
WORKDIR /workspaces/nucleo-g474-template

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# Install clang tools

RUN ln -s /usr/bin/clang-format-14 /usr/local/bin/clang-format
RUN ln -s /usr/bin/clang-tidy-14 /usr/local/bin/clang-tidy

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# Protobuf tools
RUN pip install protobuf==3.20.1  grpcio-tools==1.44.0

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# cleanup and vulnerability fixes

# check all installed packages with "apt list", maybe remove packages.
RUN apt remove -y \
    wget

# Fix file ownership and permissions issues between WSL and Devcontainer
# Create a user in the container that matches the user in WSL
ARG USERNAME
RUN groupadd --gid 1000 $USERNAME && \
        useradd --uid 1000 --gid 1000 -G plugdev,dialout --shell /bin/bash -m $USERNAME && \
        echo "$USERNAME ALL=(ALL:ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME

# run as that user in the container by default, instead of root
USER $USERNAME
