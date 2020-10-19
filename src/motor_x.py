#!/usr/bin/python3

# import the necessary packages
import rospy

import serial
import QboCmd

# import the necessary msgs. Example with msg type String_Int_Arrays:
from std_msgs.msg import Int16MultiArray

class motor_x_controler():
    """ Class motor_x_controler.

    Bridge class to control with ROS the x motor of the robot
    """

    def __init__(self):
        """Class constructor

        It is the constructor of the class. It does:
            - Subscribe to /motor_x topic
            - Set the ID of the x motor as 1
            - Set the minimum and maximum values for the msg
            - Set the minimum and maximum values for the motor
        """

        #Subscribe to ROS topics
        self.sub_x = rospy.Subscriber("motor_x", Int16MultiArray, self.motor_x_cb)

        self.motor_id=1 #Set as motor x

        #angular msg limits
        self.msg_ang_min = -100
        self.msg_ang_max = 100

        #angular motor limits
        self.real_ang_min = 290
        self.real_ang_max = 725

        self.serial_configuration()

        print("[INFO] Node started")

    def serial_configuration(self):
        """Void serial_configuration

        Configure the serial communication with the Qbo PCB.
        Set the serial port, the baudrate and similar things"""
        port = '/dev/serial0'
        ser = serial.Serial(port, baudrate = 115200, bytesize = serial.EIGHTBITS, stopbits = serial.STOPBITS_ONE, parity = serial.PARITY_NONE, rtscts = False, dsrdtr = False, timeout = 0)
        self.QBO = QboCmd.Controller(ser)

    def move_x(self, angle, speed):
        """Void move_x

        Void to send the commands to the robot. It is sent the motor ID, the angle to reach and the speed"""
        self.QBO.SetServo(self.motor_id, angle, speed)

    def run_loop(self):
        """ Infinite loop.

        When ROS is closed, it exits.
        """
        while not rospy.is_shutdown():
            #functions to repeat until the node is closed
            rospy.spin()

    def stopping_node(self):
        """ROS closing node

        Is the function called when ROS node is closed."""
        print("\n\nBye bye! :)\n\n")

    def motor_x_cb(self, data):
        """ROS callback

        This void is executed when a message is received"""

        # Set the angular value if it is out of the limits
        if data.data[0] < self.msg_ang_min:
            data.data[0] = self.msg_ang_min
        elif data.data[0] > self.msg_ang_max:
            data.data[0] = self.msg_ang_max

        try:
            ang = data.data[0]
            spe = data.data[1]
            # Scale the angular value from the msg limits to the real limits
            ang = ang * (self.real_ang_max-self.real_ang_min) / (self.msg_ang_max - self.msg_ang_min) + ((self.real_ang_max+self.real_ang_min)/2)
            self.move_x(int(ang), spe)
        except:
            print ("[ERROR]: Wrong data sent in motor_x")

if __name__=='__main__':
    """ Main void.

    Is the main void executed when started. It does:
    - Start the node
    - Create an object of the class
    - Run the node

    """
    try:
        rospy.init_node('motor_x_node')       # Init ROS node

        motors = motor_x_controler()
        rospy.on_shutdown(motors.stopping_node)   #When ROS is closed, this void is executed

        motors.run_loop()

    except rospy.ROSInterruptException:
        pass
