#!/usr/bin/env python

import serial
from time import sleep
from optparse import OptionParser
import rospy
from std_msgs.msg import Empty
from ros_workshop_py_cpp.msg import Angles


# Global ros message storage
return2home = False
angles = { "A" : None,
                  "B" : None, }


def getSerialArmCommand(angleA, angleB):
    # Generates a serial command compatable with
    # the Arduino-based arm, using angles A and B
    return "A" + str(angleA) + "B" + str(angleB) + "*"

def return2homeCallback(msg):
    print ("RTH")
    global return2home
    return2home = True

def anglesCallback(msg):
    # Callback to recieve custom ros Angles message
    global angles
    angles["A"] = msg.A
    angles["B"] = msg.B

def main():
    # Options
    parser = OptionParser()
    parser.add_option("-d", "--device", dest = "device",
        default = "/dev/ttyACM0",
        help = "Path to device to connect to Arduino")
    parser.add_option("-p", "--port", dest = "port",
        default = "9600",
        help = "Port to connect to Arduino")
    parser.add_option("-a", "--home_angle_a", dest = "home_angle_a",
        default = "87",
        help = "Angle A of arm's home position")
    parser.add_option("-b", "--home_angle_b", dest = "home_angle_b",
        default = "85",
        help = "Angle B of arm's home position")
    (options, args) = parser.parse_args()
    device = options.device
    port = int(options.port)

    # Home position
    home_A = int(options.home_angle_a)
    home_B = int(options.home_angle_b)

    # Initialize angles to home
    angles["A"] = home_A
    angles["B"] = home_B

    # Set up ros connection
    rospy.init_node("arm_sender", anonymous = True)
    rospy.Subscriber("/angles", Angles, anglesCallback)
    rospy.Subscriber("/rth", Empty, return2homeCallback)

    # Set ros refresh rate
    rate = rospy.Rate(100) # 10hz

    # Set up serial connection
    ser = serial.Serial(device, port)
    if ser.is_open:
        ser.close()
    ser.open()

    print("Before While Loop")

    # Communication loop
    halt = False
    while(halt != True):
        # Ensure safe angles
        if angles["A"] < 0 or angles["A"] > 180 or \
           angles["B"] < 0 or angles["B"] > 180:
            return2home = True

        # Check state of callback-populated variables
        global return2home
        if return2home == True:
            angles["A"] = home_A
            angles["B"] = home_B
            return2home = False # Reset

        print(angles)
        # Generate serial command using data in global angles
        sCommand = getSerialArmCommand(angles["A"], angles["B"])

        # Log command
        print ("Sendng serial command %s on %s:%d",
                (sCommand, device, port))

        # Send command to Arduino
        try:
            ser.write(sCommand)
            rospy.sleep(0.2);
        except KeyboardInterrupt:
        #    print("Interrupted")
            break

        # Pause
        rate.sleep()

    # Clean up
    ser.close()

if __name__ == '__main__':
    main()
