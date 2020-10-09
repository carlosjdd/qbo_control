#!/usr/bin/python3

# import the necessary packages
import rospy

import serial
import QboCmd

# import the necessary msgs. Example with msg type String_Int_Arrays:
from std_msgs.msg import UInt64
from std_msgs.msg import UInt8

# Declare constants with the leds matrix for every expression. Values are in hexadecimal
OFF = 0                     # 0 in msg
HAPPY = 0x110E00            # 1 in msg
SAD = 0x0E1100              # 2 in msg
SERIOUS = 0x1F1F00          # 3 in msg
LOVE = 0x1B1F0E04           # 4 in msg
TONGUE_OUT = 0x1F0A0A04     # 5 in msg
DISGUST = 0x31C1F           # 6 in msg
SURPRISE_1 = 0x40A0A04      # 7 in msg
SURPRISE_2 = 0xE11110E      # 8 in msg
FEAR = 0xA1500              # 9 in msg
HOLD_LAUGH = 0x150A00       # 10 in msg
INDIFERENCE_1 = 0x1F000000  # 11 in msg
INDIFERENCE_2 = 0xFF        # 12 in msg
WHAT = 0x3030000            # 13 in msg

# Include the expression constants in the constant Array EXPRESSIONS
EXPRESSIONS = [OFF, HAPPY, SAD, SERIOUS, LOVE, TONGUE_OUT, DISGUST, SURPRISE_1, SURPRISE_2, FEAR, HOLD_LAUGH, INDIFERENCE_1, INDIFERENCE_2, WHAT]

class mouth_controler():
    """ Class mouth_controler.

    Bridge class to control with ROS the mouth of the robot
    """

    def __init__(self):
        """Class constructor

        It is the constructor of the class. It does:
            - Subscribe to /set_mouth topic
            - Subscribe to /set_expression topic
            - Call serial_configuration
        """

        #Subscribe to ROS topics
        self.sub_mouth = rospy.Subscriber("set_mouth", UInt64, self.mouth_cb)
        self.sub_expression = rospy.Subscriber("set_expression", UInt8, self.expression_cb)

        self.serial_configuration()

        print("[INFO] Node started")

    def serial_configuration(self):
        """Void serial_configuration

        Configure the serial communication with the Qbo PCB.
        Set the serial port, the baudrate and similar things"""
        port = '/dev/serial0'
        ser = serial.Serial(port, baudrate = 115200, bytesize = serial.EIGHTBITS, stopbits = serial.STOPBITS_ONE, parity = serial.PARITY_NONE, rtscts = False, dsrdtr = False, timeout = 0)
        self.QBO = QboCmd.Controller(ser)

    def mouth(self, matrix):
        """Void mouth

        Void to send the commands to the robot. It is sent the value of the LED matrix"""
        self.QBO.SetMouth(matrix)

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

    def mouth_cb(self, data):
        """ROS callback for set_mouth

        This void ask the mouth to switch on the leds indicated in the integer received."""

        self.mouth(data.data)

    def expression_cb(self, data):
        """ROS callback for set_expression topic

        This void ask the mouth to set an expression"""
        try:
            self.mouth(EXPRESSIONS[data.data])
        except:
            print ("[ERROR]: Wrong data value received")

if __name__=='__main__':
    """ Main void.

    Is the main void executed when started. It does:
    - Start the node
    - Create an object of the class
    - Run the node

    """
    try:
        rospy.init_node('mouth_node')       # Init ROS node

        mouth = mouth_controler()
        rospy.on_shutdown(mouth.stopping_node)   #When ROS is closed, this void is executed

        mouth.run_loop()

    except rospy.ROSInterruptException:
        pass
