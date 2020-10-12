# QBO_CONTROL
This package allows to integrate ROS in the robot Qbo. It will allow to control the movement of the head motors (x and y), the led for the nose and the led matrix for the mouth.


## Install ROS

To control Qbo with ROS, the first step needed is installing ROS in the microSD of Qbo's Raspberry. To do that, you can follow the steps given to install ROS in its wiki. In my case, I installed ROS Melodic in Raspbian following this instructions:

http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi

After that, I create a new workspace, added the sources to .bashrc and also exported my ROS_WORKSPACE.

## Launch nodes

Once installed ROS in Qbo, just clone this package into your workspace and compile. In the folder "src", you can find all the nodes made to control the movement of the x and y motors of the head, the led for the nose and the matrix for the mouth. Appart from that, the script QboCmd.py is the library imported in the rest of nodes and used to control the serial communication with the actuators. You can test the different nodes found in src by running them:

```bash
rosrun qbo_control motor_x.py

rosrun qbo_control motor_y.py

rosrun qbo_control nose.py

rosrun qbo_control mouth.py
```

Other option to run all the scripts is simply launching the qbo_control launcher as follows:

```bash
roslaunch qbo_control qbo_control.launch
```

However, I have not tested the launcher because in my Qbo raspberry I have an error when I try to locate any package. Because of that I cannot use the rosrun or the roslaunch functions. I don't know if I am the only one facing this problem because of the ROS installation, but if you have the same problem, you can run the scripts using python, and everything works alright. To do that, yo can use the next commands:

```bash
roscd

python src/qbo_control/src/motor_x.py

python src/qbo_control/src/motor_y.py

python src/qbo_control/src/nose.py

python src/qbo_control/src/mouth.py
```

Note that with roscd you should have been located in your workspace, in which qbo_control has been cloned and compiled.

## Topics

Once everything is working, to control Qbo you can send msgs to each topic. The topics are:

- `/motor_x` is the topic to set the angular position between -100 and 100 and the speed to move the x motor

- `/motor_y` is the topic to set the angular position between -100 and 100 and the speed to move the y motor

- `/nose` is the topic to set the color of the nose

- `/set_expression` is the topic to set some defined expressions for the mouth

- `/set_mouth` is the topic to set any expression on the mouth.

To know how every topic work, you must have a look in the folder ``docs``. There, the full documentation is in the file ``nodes and topics.xlsx`` but you can also find the next images for every node and topic:

 ![Alt text](docs/node motor_x.png?raw=true "motor_x node") 
 
 <a href="url"><img src="docs/node motor_x.png" align="center"></a>
