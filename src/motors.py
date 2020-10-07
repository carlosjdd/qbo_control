#!/usr/bin/python3

# import the necessary packages
import rospy

# import the necessary msgs. Example with msg type String_Int_Arrays:
from custom_msgs.msg import String_Int_Arrays

class motor_controler():
    """ Class motor_controler.

    Info about the class
    """

    def __init__(self):
        """Class constructor

        It is the constructor of the class. It does:
        """

        #Subscribe to ROS topics
        self.name_subscriber = rospy.Subscriber("topic_sub", String_Int_Arrays, self.callback)

        print("[INFO] Node started")


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

    def callback(self, data):
        """ROS callback

        This void is executed when a message is received"""
        pass


if __name__=='__main__':
    """ Main void.

    Is the main void executed when started. It does:
    - Start the node
    - Create an object of the class
    - Run the node

    """
    try:
        rospy.init_node('ROSnode_name')       # Init ROS node

        motors = motor_controler()
        rospy.on_shutdown(motors.stopping_node)   #When ROS is closed, this void is executed

        motors.run_loop()

    except rospy.ROSInterruptException:
        pass
