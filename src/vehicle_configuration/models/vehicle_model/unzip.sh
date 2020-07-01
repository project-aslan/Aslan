#!/bin/bash

dpkg -s unzip &> /dev/null

if [ $? -ne 0 ]

        then
            echo "unzip not installed"  
            sudo apt-get install $name

        else
            echo    "unzip installed"
fi

DIRECTORY=`dirname $0`
echo $DIRECTORY
unzip $DIRECTORY/mesh/$1.stl.zip -d $DIRECTORY/mesh/

roslaunch vehicle_model sd_vehicle.launch model:=$1


