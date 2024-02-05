#!/usr/bin/python3

# import the necessary packages
import rospy
import math

#from playsound import playsound

# import the necessary msgs. Example with msg type String_Int_Arrays:
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import UInt8
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt8MultiArray
 
class joy_teleop():
    """ Class class_name.

    Info about the class
    """

    def __init__(self):
        """Class constructor

        It is the constructor of the class. It does:
        """

        #Subscribe to ROS topics
        self.joy_sub = rospy.Subscriber("joy", Joy, self.callback)

        #Define object as msg type and initialize empty
        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0.0
        self.vel_msg.linear.y = 0.0
        self.vel_msg.linear.z = 0.0
        self.vel_msg.angular.x = 0.0
        self.vel_msg.angular.y = 0.0
        self.vel_msg.angular.z = 0.0

        #Define object as go_to_goal msg type and initialize as true
        self.go_to_goal_msg = Bool()
        self.go_to_goal_msg.data = True

        #Define object as paint msg type
        self.paint_msg = UInt8MultiArray()
        self.paint_msg.data = [0,0]

        #Define the object with the reajet parameters
        self.rj_param_msg = Float32MultiArray()
        self.rj_param_msg.data=[200,20]

        self.rj_freq_min = 0.1
        self.rj_freq_max = 1000

        self.paint = False

        #Define variables
        self.max_linear_vel = 0.7
        self.max_angular_vel = 0.4
        self.limit_linear_vel = 2.0
        self.limit_angular_vel = 2.0
        self.increment=0.05

        self.stop_robot_msg = False

        self.define_joy()

#        self.bocina = "afilador.mp3"

        print("[INFO] Node started: " + rospy.get_name())

    def define_joy(self):

        self.ax_up_down = 1
        self.ax_left_right = 0
        self.bt_select = 8
        self.bt_start = 9

        self.bt_l3 = 11
        self.bt_r3 = 12

        self.bt_x = 0
        self.bt_circle = 1
        self.bt_triangle = 2
        self.bt_square = 3

        self.bt_l1 = 4
        self.bt_r1 = 5
        self.bt_l2 = 6
        self.bt_r2 = 7

        self.ax_l3_left = 0
        self.ax_l3_up = 1
        self.ax_l2 = 2

        self.ax_r3_left = 3
        self.ax_r3_up = 4
        self.ax_r2 = 5

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
        print("\n[" + rospy.get_name() + "] Bye bye! :)\n")

    def callback(self, data):
        """ROS callback

        This void is executed when a message is received"""
        print('BOTONES', data.buttons)
        print('EJES', data.axes)
        
        #Modify the speed of the robot depending on the trigger pushed
        self.max_linear_vel = self.max_linear_vel + data.buttons[self.bt_r1]*self.increment
        self.max_linear_vel = self.max_linear_vel - data.buttons[self.bt_l1]*self.increment
        self.max_angular_vel = self.max_angular_vel + data.buttons[self.bt_r2]*self.increment
        self.max_angular_vel = self.max_angular_vel - data.buttons[self.bt_l2]*self.increment

        #If the speed is going to be increased from its maximum value, set it to the maximum
        if self.max_linear_vel > self.limit_linear_vel:
            self.max_linear_vel = self.limit_linear_vel
        if self.max_angular_vel > self.limit_angular_vel:
            self.max_angular_vel = self.limit_angular_vel

        #If the speed is going to be decreased from zero, it remains as zero.
        if self.max_linear_vel < 0:
            self.max_linear_vel = 0
        if self.max_angular_vel < 0:
            self.max_angular_vel = 0

        #Print the new speeds if they have been changed
        if (data.buttons[self.bt_r1]!=0 or data.buttons[self.bt_l1]!=0 or data.buttons[self.bt_r2]!=0 or data.buttons[self.bt_l2]!=0):
            print ("Linear speed: " + str(self.max_linear_vel) + "\nAngular speed: " + str(self.max_angular_vel))

        #Set x and angular speeds depending on the left joystick
        self.vel_msg.linear.x = data.axes[self.ax_l3_up] * self.max_linear_vel
        self.vel_msg.angular.z = data.axes[self.ax_l3_left] * self.max_angular_vel

        #If the joystick is not used, take the speeds from the arrows
        if self.vel_msg.linear.x == 0.0 and self.vel_msg.angular.z == 0.0:

            #Calculate with direction means positive and negative linear and angular speeds to drive with arrows
            x_direction = data.axes[self.ax_up_down]
            y_direction = data.axes[self.ax_left_right] 

            if x_direction > 0:
                self.vel_msg.linear.x = self.max_linear_vel
            elif x_direction < 0:
                self.vel_msg.linear.x = - self.max_linear_vel
            else:
                self.vel_msg.linear.x = 0

            if y_direction > 0:
                self.vel_msg.angular.z = self.max_angular_vel
            elif y_direction < 0:
                self.vel_msg.angular.z = - self.max_angular_vel
            else:
                self.vel_msg.angular.z = 0


if __name__=='__main__':
    """ Main void.

    Is the main void executed when started. It does:
    - Start the node
    - Create an object of the class
    - Run the node

    """
    try:
        rospy.init_node('joy_teleop_node')       # Init ROS node

        joy = joy_teleop()
        rospy.on_shutdown(joy.stopping_node)   #When ROS is closed, this void is executed

        joy.run_loop()

    except rospy.ROSInterruptException:
        pass
