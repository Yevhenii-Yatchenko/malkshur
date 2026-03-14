FROM ubuntu:22.04

# Prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Install system dependencies and Gazebo repository
RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    lsb-release \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.key | gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y \
    # Python and build tools
    python3 \
    python3-pip \
    python3-dev \
    python3-opencv \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    # SSH server
    openssh-server \
    sudo \
    # Network tools
    net-tools \
    iputils-ping \
    telnet \
    # Gazebo transport and dependencies needed for camera bridge
    python3-gz-transport13 \
    python3-gz-msgs11 \
    libignition-transport11-dev \
    libignition-msgs8-dev \
    libprotobuf-dev \
    protobuf-compiler \
    # OpenCV dependencies
    libopencv-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    # Additional libraries
    libboost-all-dev \
    libeigen3-dev \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --upgrade pip && \
    pip3 install numpy==1.24.4 && \
    pip3 install -r /tmp/requirements.txt

# Install additional Python packages for Gazebo integration
RUN pip3 install \
    protobuf==3.20.3 \
    Pillow \
    scipy

# Install pygazebo from source (for classic Gazebo support)
RUN git clone https://github.com/jpieper/pygazebo.git /tmp/pygazebo && \
    cd /tmp/pygazebo && \
    python3 setup.py install && \
    rm -rf /tmp/pygazebo

# Configure SSH
RUN mkdir /var/run/sshd
RUN echo 'root:drone' | chpasswd
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
RUN sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config

# Create a user for development (optional, you can use root)
RUN useradd -m -s /bin/bash drone && \
    echo 'drone:drone' | chpasswd && \
    usermod -aG sudo drone

# Create necessary directories for logs (will be in mounted volume)
RUN mkdir -p /drone_project

# Set working directory (will be mounted)
WORKDIR /drone_project

# Fix pygazebo for Python 3.10+ compatibility
COPY gazebo/fix_pygazebo.py /fix_pygazebo.py
RUN python3 /fix_pygazebo.py

# Expose SSH and Telnet ports
EXPOSE 22
EXPOSE 2323

# Start SSH service by default
CMD ["/usr/sbin/sshd", "-D"]
