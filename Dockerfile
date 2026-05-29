FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1

# Default to headless OSMesa rendering; override at runtime with MUJOCO_GL=glfw + DISPLAY
ENV MUJOCO_GL=osmesa

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    python3-tk \
    libosmesa6-dev \
    libgl1-mesa-dev \
    libgl1-mesa-glx \
    libglew-dev \
    libglfw3-dev \
    build-essential \
    git \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip
RUN pip3 install --upgrade pip

WORKDIR /opt/manipulation_vision

# Pin mujoco before robosuite/robocasa can pull a different version
RUN pip3 install mujoco==3.3.1

# Missing from both setup.py files but used in scripts
RUN pip3 install matplotlib trimesh

# Copy and install robosuite fork (editable so fork changes are live)
COPY includes/robosuite includes/robosuite
RUN pip3 install -e includes/robosuite

# Copy and install robocasa fork (editable; pulls tianshou, lerobot, etc.)
COPY includes/robocasa includes/robocasa
RUN pip3 install -e includes/robocasa

# Copy the rest of the repo
COPY scripts scripts
COPY src src
COPY includes/gain_tuner includes/gain_tuner
COPY Documentation Documentation

ENV PYTHONPATH=/opt/manipulation_vision

CMD ["bash"]
