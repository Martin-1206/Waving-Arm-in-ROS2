"""
Please take a look at:
https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html
https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html
https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html

Have fun
"""

"""load python packages here"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros import TransformBroadcaster





class BodyBroadcaster(Node):
    """
    class functions can be defined below the constructor
    """

    def __init__(self) -> None:
        """
        defnition of the constructor of the class
        """
        super().__init__('body_broadcaster_node')                                                        # Node init using super() function of Node class
        self.tf_broadcaster = TransformBroadcaster(self)                                                 # initialize the transform broadcaster
        self.shoulder = self.create_subscription(Pose, 'shoulder', self.shoulder_callback, 10)           # create a subscriber 1 shoulder
        self.upper_arm = self.create_subscription(Pose, 'upper_arm', self.upper_arm_callback, 10)        # create a subscriber 2 upper arm
        self.lower_arm = self.create_subscription(Pose, 'lower_arm', self.lower_arm_callback, 10)        # create a subscriber 3 lower arm

    
    def shoulder_callback(self, msg: Pose):                 # create a callback for subscriber 1 shoulder
        t = TransformStamped()                              # create a TransformStamped object and give it the appropriate metadata (appropriate = entsprechenden)
        t.header.stamp = self.get_clock().now().to_msg()    # give the transform being published a timestamp
        t.header.frame_id = 'hip'                           # set the name of the parent frame of the link
        t.child_frame_id = 'shoulder'                       # set the name of the child

        '''gets translation coordinates from the message'''
        t.transform.translation.x = msg.position.x 
        t.transform.translation.y = msg.position.y
        t.transform.translation.z = msg.position.z

        '''rotations'''
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)                # send the transformation


    def upper_arm_callback(self, msg: Pose):                # create a callback for subscriber 2 upper arm
        t = TransformStamped()                              # create a TransformStamped object and give it the appropriate metadata (appropriate = entsprechenden)
        t.header.stamp = self.get_clock().now().to_msg()    # give the transform being published a timestamp
        t.header.frame_id = 'shoulder'                      # set the name of the parent frame of the link
        t.child_frame_id = 'upper_arm'                      # set the name of the child

        '''gets translation coordinates from the message'''
        t.transform.translation.x = msg.position.x 
        t.transform.translation.y = msg.position.y
        t.transform.translation.z = msg.position.z

        '''rotations'''
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)                # Send the transformation

    def lower_arm_callback(self, msg: Pose):                # create a callback for subscriber 3 lower arm
        t = TransformStamped()                              # create a TransformStamped object and give it the appropriate metadata (appropriate = entsprechenden)
        t.header.stamp = self.get_clock().now().to_msg()    # give the transform being published a timestamp
        t.header.frame_id = 'upper_arm'                     # set the name of the parent frame of the link
        t.child_frame_id = 'lower_arm'                      # set the name of the child

        '''gets translation coordinates from the message'''
        t.transform.translation.x = msg.position.x 
        t.transform.translation.y = msg.position.y
        t.transform.translation.z = msg.position.z

        '''rotations'''
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)                # Send the transformation    




       


       


"""
define the main function here:
"""


def main(args=None):
    rclpy.init(args=args)

    body_broadcaster = BodyBroadcaster()

    rclpy.spin(body_broadcaster)
    body_broadcaster.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
