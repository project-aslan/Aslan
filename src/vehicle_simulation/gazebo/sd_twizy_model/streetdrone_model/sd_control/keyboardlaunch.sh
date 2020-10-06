#!/bin/bash

dpkg -s ros-melodic-teleop-twist-keyboard &> /dev/null

if [ $? -ne 0 ]

        then
            echo "keyboard control not installed"
            sudo apt-get install ros-melodic-teleop-twist-keyboard

        else
            echo    "keyboard control installed"
fi

x-terminal-emulator -e rosrun sd_control sd_teleop_keyboard.py

exit 0
