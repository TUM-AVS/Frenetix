# Use the manylinux1 image from PyPA as the base
FROM quay.io/pypa/manylinux_2_28_x86_64

# Set the working directory in the Docker container
WORKDIR /app

# Copy your source code into the Docker container
COPY . /app

# Create symbolic links to Python 3.10 and pip3.10
RUN ln -s /opt/python/cp310-cp310/bin/python /usr/bin/python3
RUN ln -s /opt/python/cp310-cp310/bin/pip /usr/bin/pip

# Install the C++ dependencies
RUN yum install -y boost-devel
RUN yum install -y nano
RUN yum install -y gcc gcc-c++ make cmake git wget unzip boost-devel eigen3-devel libgomp

# Compile the driveablility library
RUN setup_script1.sh

# Compile the frenetPlannerHelper library and make a wheel including the dynamic library from the driveablity_checker
RUN setup_script.sh
