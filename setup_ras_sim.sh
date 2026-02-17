#!/bin/bash
# Docker Setup for development off raspberry pi Setup Script
set -e

# Build and start container
docker compose up -d embr-sim

# Access container & Source 
docker compose exec embr-sim bash -c "source install/setup.bash && bash"