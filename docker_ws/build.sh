#!/bin/bash
echo "Costruzione dell'immagine Docker per il progetto Mobile Robotics..."
docker build --rm -f Dockerfile.MR05 -t mr_project_image .