#!/bin/bash

# Docker setup script for Jetson Orin NX
# This script prepares the environment for Docker deployment

set -e

echo "Setting up Docker environment for Jetson Orin NX..."

# Create X11 auth file for GUI support
XAUTH_FILE=/tmp/.docker.xauth
if [ ! -f $XAUTH_FILE ]; then
    touch $XAUTH_FILE
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH_FILE nmerge -
    chmod 666 $XAUTH_FILE
    echo "Created X11 auth file: $XAUTH_FILE"
else
    echo "X11 auth file already exists: $XAUTH_FILE"
fi

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed. Please install Docker first."
    echo "For Jetson devices, you can install using:"
    echo "  curl -fsSL https://get.docker.com -o get-docker.sh"
    echo "  sudo sh get-docker.sh"
    exit 1
fi

# Check if docker-compose is installed
if ! command -v docker-compose &> /dev/null && ! docker compose version &> /dev/null; then
    echo "Warning: docker-compose is not installed."
    echo "You can install it using:"
    echo "  sudo apt-get install docker-compose"
    echo "Or use 'docker compose' (newer Docker versions include compose as a plugin)"
fi

# Check for NVIDIA Container Toolkit (required for GPU support on Jetson)
if ! docker info 2>/dev/null | grep -q nvidia; then
    echo ""
    echo "Warning: NVIDIA Container Toolkit may not be configured."
    echo "For GPU support on Jetson, you may need to install nvidia-container-toolkit:"
    echo ""
    echo "  # For Jetson devices, install nvidia-container-toolkit:"
    echo "  distribution=\$(. /etc/os-release;echo \$ID\$VERSION_ID)"
    echo "  curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -"
    echo "  curl -s -L https://nvidia.github.io/nvidia-docker/\$distribution/nvidia-docker.list | \\"
    echo "    sudo tee /etc/apt/sources.list.d/nvidia-docker.list"
    echo "  sudo apt-get update"
    echo "  sudo apt-get install -y nvidia-container-toolkit"
    echo "  sudo systemctl restart docker"
    echo ""
    echo "Or for Jetson-specific installation, refer to NVIDIA documentation."
fi

# Check architecture
ARCH=$(uname -m)
if [ "$ARCH" != "aarch64" ]; then
    echo "Warning: This script is designed for ARM64 (aarch64) architecture."
    echo "Current architecture: $ARCH"
fi

echo ""
echo "Setup complete!"
echo ""
echo "To build and start the container, run:"
echo "  ./docker-run.sh rebuild"
echo ""
echo "Or manually:"
echo "  docker-compose -f docker/docker-compose.yml up -d --build"
echo ""
echo "To enter the container:"
echo "  ./docker-run.sh shell"
echo ""
echo "To view logs:"
echo "  ./docker-run.sh logs"
echo ""
echo "To stop the container:"
echo "  ./docker-run.sh down"

