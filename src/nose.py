#!/usr/bin/python3

# import the necessary packages
import rospy

import serial
import QboCmd

# import the necessary msgs. Example with msg type String_Int_Arrays:
from std_msgs.msg import UInt8

class nose_controler():
    """ Class motor_x_controler.

    Info about the class
    """

    def __init__(self):
        """Class constructor

        It is the constructor of the class. It does:
        """

        #Subscribe to ROS topics
        self.sub_nose = rospy.Subscriber("nose", UInt8, self.nose_cb)

        self.serial_configuration()

        print("[INFO] Node started")

    def serial_configuration(self):
        """Void serial_configuration

        Configure the serial communication with the Qbo PCB.
        Set the serial port, the baudrate and similar things"""
        port = '/dev/serial0'
        ser = serial.Serial(port, baudrate = 115200, bytesize = serial.EIGHTBITS, stopbits = serial.STOPBITS_ONE, parity = serial.PARITY_NONE, rtscts = False, dsrdtr = False, timeout = 0)
        self.QBO = QboCmd.Controller(ser)

    def nose(self, color):
        """Void move_x

        Void to send the commands to the robot. It is sent the motor ID, the angle to reach and the speed"""
        self.QBO.SetNoseColor(color)

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

    def motor_x_cb(self, data):
        """ROS callback

        This void is executed when a message is received"""

        # Qbo colores: 0 y 2 apagado. 1 y 3 azul. 4 y 6 verde. 5 cian
        try:
            if data.data == 1:
                col = 1
            elif data.data == 2:
                col = 4
            elif data.data == 3:
                col = 5
            else:
                col = 0
            self.nose(col)
        except:
            print ("[ERROR]: Wrong data sent in nose")

if __name__=='__main__':
    """ Main void.

    Is the main void executed when started. It does:
    - Start the node
    - Create an object of the class
    - Run the node

    """
    try:
        rospy.init_node('nose_node')       # Init ROS node

        nose_color = nose_controler()
        rospy.on_shutdown(nose_color.stopping_node)   #When ROS is closed, this void is executed

        nose_color.run_loop()

    except rospy.ROSInterruptException:
        pass
