"""
template for python scripts:

this file should help you to write clean code...
Please use this file as a guide to create python code for KogRob

This file is a modified version of the ROS tutorial "Writing a Simple Publisher and Subscriber (Python)"

https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
"""

"""
import all python packages you need here:
(But only the required packages)
"""


import rclpy
from rclpy.node import Node                             # Nodes can be used
from geometry_msgs.msg import Pose                      # import Pose for publishers
from math import sin, cos, pi

"""
define all non class functions you write here:
"""


"""
Here comes the Node class definition:
"""


class BodyPublisher(Node):                          
    """
    class functions can be defined below the constructor
    """

    def __init__(self) -> None:
        """
        defnition of the constructor of the class
        """
        super().__init__('body_publisher_node')     # Node init using super() function of Node class #super().__init__ calls the Node class's constructor and gives it your node name, in this case body_publisher_node.
        self.shoulder  = self.create_publisher(Pose, 'shoulder', 10)    # create a publisher 1 shoulder
        self.upper_arm = self.create_publisher(Pose, 'upper_arm', 10)   # create a publisher 2 upper arm
        self.lower_arm = self.create_publisher(Pose, 'lower_arm', 10)   # create a publisher 3 lower arm
        
        '''Next, a timer is created with a callback to execute every 0.1 seconds. self.i is a counter used in the callback.'''
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
          
        

    def timer_callback(self) -> None:
        
        '''Define shoulder position'''
        msg_shoulder = Pose()                   # create a message with the data of the Pose
        msg_shoulder.position.x = 0.3           # shoulder position in x
        msg_shoulder.position.y = 0.0           # shoulder position in y
        msg_shoulder.position.z = 1.0           # shoulder position in z           

        self.shoulder.publish(msg_shoulder)     # publishes the message to the topic
        

        '''Function turns degree into radiant'''
        def degree_to_radiant(degree):
            radiant = (degree / 180) * pi
            return radiant
        
        degree_upper_arm = 6     # degree 
        degree_lower_arm = 40    # degree
        
        radiant_upper_arm = degree_to_radiant(degree_upper_arm)  # using function to convert degree to radiant
        radiant_lower_arm = degree_to_radiant(degree_lower_arm)  # using function to convert degree to radiant

        time = self.get_clock().now().nanoseconds                # getting the node time
        time = time / 10**9

        f = 1                           # frequency in Hz
        circular_f = (2 * pi) * f       # circular frequency 

            


        '''Implement harmonic oscillation'''
        oscillation_angle_upper_arm = radiant_upper_arm * cos(circular_f * time) + (pi/2)
        oscillation_angle_lower_arm = radiant_lower_arm * cos(circular_f * time)

        

        '''Define upper arm position depending on phi'''
        msg_upper_arm = Pose()                                                              # create a message with the data of the Pose
        upper_arm_length = 0.5                                                              # fixed upper arm length with 0.5m
        msg_upper_arm.position.x = upper_arm_length * sin(oscillation_angle_upper_arm)      # calculate x position 
        msg_upper_arm.position.y = 0.0                                                      # y position is fixed
        msg_upper_arm.position.z = upper_arm_length * cos(oscillation_angle_upper_arm)      # calculate z position

        self.upper_arm.publish(msg_upper_arm)                                               # publsihes the message to the topic

        '''Define lower arm position depending on phi'''
        msg_lower_arm = Pose()                                                              # create a message with the data of the Pose
        lower_arm_length = 0.5                                                              # fixed lower arm length with 0.5m
        msg_lower_arm.position.x = lower_arm_length * sin(oscillation_angle_lower_arm)      # calculate x position 
        msg_lower_arm.position.y = 0.0                                                      # y position is fixed
        msg_lower_arm.position.z = lower_arm_length * cos(oscillation_angle_lower_arm)      # calculate z position

        self.lower_arm.publish(msg_lower_arm)                                               # publishes the message to the topic
    

"""
define the main function here:
"""


def main(args=None):
    rclpy.init(args=args)

    body_publisher = BodyPublisher()

    rclpy.spin(body_publisher)                  # spins the node 'body_publisher' so the programm works all the time

    body_publisher.destroy_node()
    rclpy.shutdown()


"""
calls main function only if you call this script directly.
If this script is imported to another python script it will not call main function here.
"""

if __name__ == "__main__":
    main()

