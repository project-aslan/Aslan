#!/bin/bash
echo "Script executed from: ${PWD}"

BASEDIR=$(dirname $0)
echo "Script location: ${BASEDIR}"

cd $BASEDIR
cp -ar ../src ./src
cp ../run ./run

sudo docker build -t aslan_docker .

rm -r src
rm run
