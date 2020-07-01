#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import TwistStamped
from aslan_msgs.msg import SDControl

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to SD_Control!
uses "w, a, s , d, x " keys or numpad
---------------------------
Apply throttle:
   'w' or '8'
Ease off throttle:
    's' or '5'
Turn steering left:
    'a' or '4'
Turn steering right:
    'd' or '6'
 Apply brakes:
    'x' or '2'
    
CTRL-C to quit

Notes:
The twizy has a deadband of throttle, and requires >25% throttle to begin moving.
Steering will centre upon braking
All requests ramp upon sustain key press. Depress key to maintain steady request
"""
#First number controls rate of increase of brake, steer or throttle etc when button press is held
#First number is a placeholder for overrun, steering centalling functionality. 
throttleKeys={
    'w':(1,0),
    '8':(1,0)
}

overrunKeys={
    's':(1,0),
    '5':(1,0)
}

brakeKeys={
    'x':(5,0),
    '2':(5,0),
}

leftKeys={
    'a':(4,0),
    '4':(4,0),
}

rightKeys={
    'd':(4,0),
    '6':(4,0),
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    print(msg)

    pub = rospy.Publisher('/sd_control', SDControl, queue_size = 1)
    control_msg = SDControl()

    rospy.init_node('teleop_twist_keyboard')

    overrun = 0
    brake = 0
    torque = 0
    steer = 0
    
    try:
        while(1):
        
            key = getKey()

            if key in throttleKeys.keys():
                torque += throttleKeys[key][0]
                brake = 0
                
            elif key in overrunKeys.keys():
                torque -= overrunKeys[key][0]
                brake = 0
 
            elif key in brakeKeys.keys():
                brake -= brakeKeys[key][0]
                steer = 0
                    
            elif key in leftKeys.keys():
                steer += leftKeys[key][0]
                    
            elif key in rightKeys.keys():
                steer -= rightKeys[key][0]

            else:
                brake = 0
                if (key == '\x03'):
                    break
                
            torque = min(100, torque)
            torque = max(0, torque)
            steer = max(-100, steer)
            steer = min(100, steer)
            brake = max(-100, brake)

            
            if(brake):
                control_msg.torque = brake
            else:
                control_msg.torque = torque
            control_msg.steer = steer
            print("Throttle " , torque, " brake ", brake, " steer ", steer)
            pub.publish(control_msg)

    except Exception as e:
        print(e)


    finally:
        control_msg.torque = 0
        control_msg.steer = 0
        pub.publish(control_msg)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
