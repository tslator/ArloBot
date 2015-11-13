#!/usr/bin/env python

# This node will be responsible for
#    1. launching the webserver (we need sudo if we put the webserver on port 80, otherwise, we can use a different port, e.g. 8080)
#    2. launching the webif node which will listen for joy stick messages (coming from the web page) and pusblishing them as twist messages
#
# So, this will be a ros node and run in its own process.  It will subscribe to joy stick messages and publish twist messages
# Typically, a node like this will do all its work in the __init__ call


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class XlateJoystickToVelocity:
    def __init__(self):
        rospy.init_node("XlateJoystickToVelocity")

        self._pub_rate = rospy.get_param("~pub_rate", 0.067)
        self._linear_scale = rospy.get_param("~linear_scale", 1)
        self._angular_scale = rospy.get_param("~angular_scale", 1)
        self._last_time = rospy.Time.now()

        # Subscribe to the joy stick messages coming from the web page
        rospy.Subscriber("html_joy_stick", Joy, self.JoystickCallback)

        # Publish Twist messages on /cmd_vel
        self._cmd_vel_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

        rospy.spin()

    def JoystickCallback(self, data):
        if rospy.Time.now() - self._last_time > rospy.Duration(self._pub_rate):

            rospy.loginfo("Received joystick msg: " + str(data))
            self._last_time = rospy.Time.now()

            # axes 0 and 1 are assigned to the drive train
            # axes 2 and 3 are assigned to the pan/tilt

            # Translate the joystick messages to twist commands
            twist = Twist()
            twist.linear.x = self._linear_scale*data.axes[0] # forward/backward speed
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self._angular_scale*data.axes[1] # turning speed

            self._cmd_vel_publisher.publish(twist)

if __name__ == '__main__':
    try:
        xlate = XlateJoystickToVelocity()
    except rospy.ROSInterruptException:
        pass
