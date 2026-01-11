#!/bin/bash

# Quick start script for Docker deployment
# Usage: ./docker-run.sh [command]

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

# Check if docker-compose or docker compose is available
if command -v docker-compose &> /dev/null; then
    DOCKER_COMPOSE="docker-compose"
elif docker compose version &> /dev/null; then
    DOCKER_COMPOSE="docker compose"
else
    echo "Error: docker-compose is not installed"
    exit 1
fi

# Set docker-compose file path
DOCKER_COMPOSE_FILE="docker/docker-compose.yml"

# Get current user ID and group ID for permission fixing
export HOST_UID=${HOST_UID:-$(id -u)}
export HOST_GID=${HOST_GID:-$(id -g)}

# Parse command
COMMAND=${1:-"up"}

case "$COMMAND" in
    build)
        # Build Docker image using Dockerfile specified in docker-compose.yml
        echo "Building Docker image (using docker/Dockerfile via docker-compose.yml)..."
        $DOCKER_COMPOSE -f $DOCKER_COMPOSE_FILE build
        ;;
    up)
        echo "Starting container (UID: $HOST_UID, GID: $HOST_GID)..."
        HOST_UID=$HOST_UID HOST_GID=$HOST_GID $DOCKER_COMPOSE -f $DOCKER_COMPOSE_FILE up -d
        echo ""
        echo "Container started! To enter the container, run:"
        echo "  ./docker-run.sh shell"
        echo ""
        echo "To view logs, run:"
        echo "  ./docker-run.sh logs"
        ;;
    down)
        echo "Stopping container..."
        $DOCKER_COMPOSE -f $DOCKER_COMPOSE_FILE down
        ;;
    restart)
        echo "Restarting container..."
        $DOCKER_COMPOSE -f $DOCKER_COMPOSE_FILE restart
        ;;
    shell)
        echo "Entering container..."
        $DOCKER_COMPOSE -f $DOCKER_COMPOSE_FILE exec autonomy_stack bash
        ;;
    logs)
        $DOCKER_COMPOSE -f $DOCKER_COMPOSE_FILE logs -f autonomy_stack
        ;;
    rebuild)
        echo "Rebuilding from scratch..."
        $DOCKER_COMPOSE -f $DOCKER_COMPOSE_FILE down
        $DOCKER_COMPOSE -f $DOCKER_COMPOSE_FILE build --no-cache
        HOST_UID=$HOST_UID HOST_GID=$HOST_GID $DOCKER_COMPOSE -f $DOCKER_COMPOSE_FILE up -d
        ;;
    status)
        $DOCKER_COMPOSE -f $DOCKER_COMPOSE_FILE ps
        ;;
    *)
        echo "Usage: $0 {build|up|down|restart|shell|logs|rebuild|status}"
        echo ""
        echo "Commands:"
        echo "  build    - Build the Docker image"
        echo "  up       - Start the container (default)"
        echo "  down     - Stop the container"
        echo "  restart  - Restart the container"
        echo "  shell    - Enter the container shell"
        echo "  logs     - View container logs"
        echo "  rebuild  - Rebuild from scratch and start"
        echo "  status   - Show container status"
        exit 1
        ;;
esac

