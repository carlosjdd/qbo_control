#!/usr/bin/python3

# import the necessary packages
import rospy

import serial
import QboCmd

# import the necessary msgs. Example with msg type String_Int_Arrays:
from std_msgs.msg import Int16MultiArray

class motor_y_controler():
    """ Class motor_y_controler.

    Info about the class
    """

    def __init__(self):
        """Class constructor

        It is the constructor of the class. It does:
        """

        #Subscribe to ROS topics
        self.sub_y = rospy.Subscriber("motor_y", Int16MultiArray, self.motor_y_cb)

        self.motor_id=2 #Set as motor y

        self.msg_ang_min = -100
        self.msg_ang_max = 100

        self.real_ang_min = 150
        self.real_ang_max = 630

        self.serial_configuration()

        print("[INFO] Node started")

    def serial_configuration(self):
        """Void serial_configuration

        Configure the serial communication with the Qbo PCB.
        Set the serial port, the baudrate and similar things"""
        port = '/dev/serial0'
        ser = serial.Serial(port, baudrate = 115200, bytesize = serial.EIGHTBITS, stopbits = serial.STOPBITS_ONE, parity = serial.PARITY_NONE, rtscts = False, dsrdtr = False, timeout = 0)
        self.QBO = QboCmd.Controller(ser)

    def move_y(self, angle, speed):
        """Void move_y

        Void to send the commands to the robot. It is sent the motor ID, the angle to reach and the speed"""
        self.QBO.SetServo(self.motor_id, angle, speed)

    def run_loop(self):
        """ Infinite loop.

        When ROS is closed, it exits.
        """
        while not rospy.is_shutdown():
            #functions to repeat until the node is closed
            pass

    def stopping_node(self):
        """ROS closing node

        Is the function called when ROS node is closed."""
        print("\n\nBye bye! :)\n\n")

    def motor_y_cb(self, data):
        """ROS callback

        This void is executed when a message is received"""
        try:
            ang = data.data[0]
            spe = data.data[1]
            ang = ang * (self.real_ang_max-self.real_ang_min) / (self.msg_ang_max - self.msg_ang_min) + ((self.real_ang_max+self.real_ang_min)/2)
            self.move_y(int(ang), spe)
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
        rospy.init_node('motor_y_node')       # Init ROS node

        motors = motor_y_controler()
        rospy.on_shutdown(motors.stopping_node)   #When ROS is closed, this void is executed

        motors.run_loop()

    except rospy.ROSInterruptException:
        pass
