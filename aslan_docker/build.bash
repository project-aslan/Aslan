#!/bin/bash
echo "Script executed from: ${PWD}"

BASEDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
echo "Script location: ${BASEDIR}"

# Go to Aslan root directory
pushd $(dirname $BASEDIR)

# Ensure latest version of Melodic base image is available
docker pull ros:melodic

# Build Docker image
docker build -t aslan_docker -f aslan_docker/Dockerfile .

# Go back to where we came from
popd
