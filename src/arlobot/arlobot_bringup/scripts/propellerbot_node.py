#!/usr/bin/env python
# Using PEP 8: http://wiki.ros.org/PyStyleGuide
# pylint: disable=line-too-long
# Software License Agreement (BSD License)
#
# Author: Chris L8 https://github.com/chrisl8
# URL: https://github.com/chrisl8/ArloBot
#
# Derived from \opt\ros\hydro\lib\create_node\turtlebot_node.py
# This is based on turtlebot_node adapted to run on a Propeller Activity Board based ArloBot
#
# When upgrading to new versions of ROS,
# or when attempting to integrate new TurtleBot functions,
# please look at and compare turtlebot_node.py to the new version
# to see what you may need to add/improve/replace
# to make things work.
#
# Special thanks to arduino.py by Dr. Rainer Hessmer
# https://code.google.com/p/drh-robotics-ros/
# Much of my code below is based on or copied from his work.
#
# NOTE: This script REQUIRES parameters to be loaded from param/encoders.yaml!

import sys
sys.path.append("~/catkin_ws/src/ArloBot/src/arlobot/arlobot_bringup/scripts")

import rospy
import tf
from math import sin, cos, pi
import time
import subprocess
import os

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Bool
from arlobot_msgs.msg import usbRelayStatus, arloStatus, arloSafety
from arlobot_msgs.srv import FindRelay, ToggleRelay


from SerialDataGateway import SerialDataGateway
from OdomStationaryBroadcaster import OdomStationaryBroadcaster
from SerialMessage import SerialMessage, SerialMessageError


class PropellerComm(object):
    """
    Helper class for communicating with a Propeller board over serial port
    """

    def __init__(self):

        rospy.init_node('arlobot')

        self.r = rospy.Rate(1) # 1hz refresh rate
        self._Counter = 0  # For Propeller code's _HandleReceivedLine and _write_serial
        self._motorsOn = False  # Set to 1 if the motors are on, used with USB Relay Control board
        self._motorsOn = True
        self._safeToGo = False  # Use arlobot_safety to set this
        self._safeToGo = True
        self._SafeToOperate = False  # Use arlobot_safety to set this
        self._SafeToOperate = True
        self._acPower = True # Track AC power status internally
        self._acPower = False
        self._unPlugging = False # Used for when arlobot_safety tells us to "UnPlug"!
        self._wasUnplugging = False # Track previous unplugging status for motor control
        self._SwitchingMotors = False  # Prevent overlapping calls to _switch_motors
        self._serialAvailable = False
        self._serialTimeout = 0
        self._leftMotorPower = False
        self._rightMotorPower = False
        self._laptop_battery_percent = 100
        # Store last x, y and heading for reuse when we reset
        # I took off off the ~, because that was causing these to reset to default on every restart
        # even if roscore was still up.
        self.lastX = rospy.get_param("lastX", 0.0)
        self.lastY = rospy.get_param("lastY", 0.0)
        self.lastHeading = rospy.get_param("lastHeading", 0.0)
        self.alternate_heading = self.lastHeading
        self.track_width = rospy.get_param("~driveGeometry/trackWidth", "0")
        self.distance_per_count = rospy.get_param("~driveGeometry/distancePerCount", "0")
        self.ignore_proximity = rospy.get_param("~ignoreProximity", False);
        self.ignore_cliff_sensors = rospy.get_param("~ignoreCliffSensors", False);
        self.ignore_ir_sensors = rospy.get_param("~ignoreIRSensors", False);
        self.ignore_floor_sensors = rospy.get_param("~ignoreFloorSensors", False);
        self.robotParamChanged = False
        self._op_state_set = False


        # Get motor relay numbers for use later in _HandleUSBRelayStatus if USB Relay is in use:
        self.relayExists = rospy.get_param("~usbRelayInstalled", False)
        if self.relayExists:
            # I think it is better to get these once than on every run of _HandleUSBRelayStatus
            self.usbLeftMotorRelayLabel = rospy.get_param("~usbLeftMotorRelayLabel", "")
            self.usbRightMotorRelayLabel = rospy.get_param("~usbRightMotorRelayLabel", "")
            rospy.loginfo("Waiting for USB Relay find_relay service to start . . .")
            rospy.wait_for_service('/arlobot_usbrelay/find_relay')
            rospy.loginfo("USB Relay find_relay service started.")
            try:
                find_relay = rospy.ServiceProxy('/arlobot_usbrelay/find_relay', FindRelay)
                self.leftMotorRelay = find_relay(self.usbLeftMotorRelayLabel)
                self.rightMotorRelay = find_relay(self.usbRightMotorRelayLabel)
                if self.leftMotorRelay.foundRelay and self.leftMotorRelay.foundRelay:
                    rospy.loginfo("Left = " + str(self.leftMotorRelay.relayNumber) + " & Right = " + str(
                        self.rightMotorRelay.relayNumber))
                else:
                    self.relayExists = False
            except rospy.ServiceException as e:
                rospy.loginfo("Service call failed: %s" % e)
            rospy.Subscriber("arlobot_usbrelay/usbRelayStatus", usbRelayStatus,
                             self._handle_usb_relay_status)  # Safety Shutdown
        # Subscriptions
        #rospy.Subscriber("cmd_vel", Twist, self._handle_velocity_command)  # Is this line or the below bad redundancy?
        rospy.Subscriber("mobile_base/commands/velocity", Twist, self._handle_velocity_command)
        #rospy.Subscriber("arlobot_safety/safetyStatus", arloSafety, self._safety_shutdown)  # Safety Shutdown

        # Publishers
        self._SerialPublisher = rospy.Publisher('serial', String, queue_size=10)
        self._pirPublisher = rospy.Publisher('~pirState', Bool, queue_size=1)  # for publishing PIR status
        self._arlo_status_publisher = rospy.Publisher('arlo_status', arloStatus, queue_size=1)
        self._cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # IF the Odometry Transform is done with the robot_pose_ekf do not publish it,
        # but we are not using robot_pose_ekf, because it does nothing for us if you don't have a full IMU!
        self._OdometryTransformBroadcaster = tf.TransformBroadcaster()  # REMOVE this line if you use robot_pose_ekf
        self._OdometryPublisher = rospy.Publisher("odom", Odometry, queue_size=10)
        # We don't need to broadcast a transform, as it is static and contained within the URDF files
        # self._SonarTransformBroadcaster = tf.TransformBroadcaster()
        self._UltraSonicPublisher = rospy.Publisher("ultrasonic_scan", LaserScan, queue_size=10)
        self._InfraredPublisher = rospy.Publisher("infrared_scan", LaserScan, queue_size=10)
        # You can use the ~/metatron/scripts/find_propeller.sh script to find this, and
        # You can set it by running this before starting this:
        # rosparam set /arlobot/port $(~/metatron/scripts/find_propeller.sh)
        port = rospy.get_param("~port", "/dev/ttyUSB0")
        baud_rate = int(rospy.get_param("~baudRate", 115200))
        rospy.loginfo("Starting with serial port: " + port + ", baud rate: " + str(baud_rate))
        self._SerialDataGateway = SerialDataGateway(port, baud_rate, self._handle_received_line)


    def _handle_received_line(self, line):  # This is Propeller specific
        """
        This will run every time a line is received over the serial port (USB)
        from the Propeller board and will send the data to the correct function.
        """
        self._Counter += 1
        self._serialTimeout = 0

        msg = None
        try:
            msg = SerialMessage(data = line)
        except SerialMessageError:
            rospy.logwarn("Invalid line: " + line)
            return

        self._SerialPublisher.publish(String(str(self._Counter) + ", in:  " + line))
        if msg.msg_class == SerialMessage.STATUS_CLASS:
            if msg.msg_type == SerialMessage.STATUS_ODOMETRY_MESSAGE:
                self._broadcast_odometry_info(msg)
            elif msg.msg_type == SerialMessage.STATUS_OP_STATE_MESSAGE:
                # The operational state message contains a lot of information, but during startup
                # only portions of it are used
                # First, the robot needs to be configured with the drive geometry and operational state
                # There is a message for drive geometry and one for operational state

                if not msg.drive_geometry_received:
                    rospy.loginfo("Need to config drive geometry")
                    self._initialize_drive_geometry()

                elif not msg.op_state_received:
                    rospy.loginfo("Need to config operational state")
                    self._initialize_op_state(msg)

                elif not self._op_state_set and msg.drive_geometry_received and msg.op_state_received:
                    self._op_state_set = True

                elif self._op_state_set:
                    self._broadcast_arlo_status(msg)
            elif msg.msg_type == SerialMessage.STATUS_US_SENSOR_MESSAGE:
                # Publish Ultrasonic data
                # Note: the mapping of sensors to higher-level constructs, lower deck, upper deck, front lower, back lower
                # needs to be done here as well.  Maybe there should be a mapping param that gives a label to indicies, 
                # e.g., "front lower deck" : 0 .. 4, "rear lower deck" : 5 .. 9, then that information could be used
                # to publish the raw sensor data into those contructs.
                self._handle_us_sensors(msg.payload)
                
            elif msg.msg_type == SerialMessage.STATUS_US_SENSOR_MESSAGE:
                # Publish Infrared data
                self._handle_ir_sensors(msg.payload)
                
        elif msg.msg_class == SerialMessage.DEBUG_CLASS:
            # Map the levels from the debug message to rospy log levels, e.g. info, warn, debug, fatal, etc
            # and then write them to rospy.logxxx.
            pass
        else:
            # Log something about this bad message
            rospy.logwarn("Unknown message class: " + str(msg))


    def _broadcast_arlo_status(self, msg):
        arlo_status = arloStatus()
        # Order from ROS Interface for ArloBot.c
        arlo_status.safeToProceed = msg.safe_to_proceed
        arlo_status.safeToRecede = msg.safe_to_recede
        arlo_status.Escaping = msg.escaping
        arlo_status.abd_speedLimit = msg.max_forward_speed
        arlo_status.abdR_speedLimit = msg.max_reverse_speed
        arlo_status.Heading = self.lastHeading
        arlo_status.gyroHeading = self.alternate_heading
        arlo_status.minDistanceSensor = msg.min_distance_sensor

        # Note: Just faking the motor voltages for now, remember to change this when there is an actual motor voltage measurement
        left_motor_voltage = (15 / 4.69) * msg.left_motor_voltage
        right_motor_voltage = (15 / 4.69) * msg.right_motor_voltage


        arlo_status.robotBatteryLevel = 12.0
        if left_motor_voltage < 1:
            arlo_status.leftMotorPower = False
        else:
            arlo_status.leftMotorPower = True
            arlo_status.robotBatteryLevel = left_motor_voltage
        self._leftMotorPower = arlo_status.leftMotorPower
        if right_motor_voltage < 1:
            arlo_status.rightMotorPower = False
        else:
            arlo_status.rightMotorPower = True
            arlo_status.robotBatteryLevel = right_motor_voltage
        self._rightMotorPower = arlo_status.rightMotorPower
        # 11.6 volts is the cutoff for an SLA battery.
        if arlo_status.robotBatteryLevel < 12:
            arlo_status.robotBatteryLow = True
        else:
            arlo_status.robotBatteryLow = False
        arlo_status.laptopBatteryPercent = self._laptop_battery_percent
        arlo_status.acPower = self._acPower
        arlo_status.cliff = msg.cliff_detected
        arlo_status.floorObstacle = msg.floor_obstacle_detected
        self._arlo_status_publisher.publish(arlo_status)


    def _handle_usb_relay_status(self, status):
        """
        Update motor status based on real data from arlobot_usbrelay topic
        """
        # This should never get called otherwise, but just in case the relay is in use but the motors are not.
        if self.relayExists:
            # print (status.relayOn)
            # print (status.relayOn[self.leftMotorRelay.relayNumber])
            # print (status.relayOn[self.leftMotorRelay.relayNumber])
            # Zero indexed arrays!
            if (status.relayOn[self.leftMotorRelay.relayNumber - 1] and
                    status.relayOn[self.rightMotorRelay.relayNumber - 1]):
                self._motorsOn = True
            else:
                self._motorsOn = False


    def _safety_shutdown(self, status):
        """
        Shut down the motors if the SafeToOperate topic goes false.
        Set unPlugging variable to allow for safe unplug operation.
        """
        rospy.loginfo("_safety_shutdown - " + str(status))
        self._unPlugging = status.unPlugging
        self._SafeToOperate = status.safeToOperate
        self._safeToGo = status.safeToGo

        old_ac_power = self._acPower
        self._acPower = status.acPower
        if not old_ac_power == self._acPower:
            self.robotParamChanged = True

        self._laptop_battery_percent = status.laptopBatteryPercent
        if not self._SafeToOperate:
            if self._motorsOn:
                rospy.loginfo("Safety Shutdown initiated")
                self._reset_serial_connection()


    def _reset_serial_connection(self):
        if self._motorsOn:
            self._switch_motors(False)
            # Wait for the motors to shut off
            while self._motorsOn:
                time.sleep(1)
        # Reset the propeller board, otherwise there are problems
        # if you bring up the motors again while it has been operating
        self._serialAvailable = False
        rospy.loginfo("Serial Data Gateway stopping . . .")
        try:
            self._SerialDataGateway.Stop()
        except AttributeError:
            rospy.loginfo("Attempt to start nonexistent Serial device.")
        rospy.loginfo("Serial Data Gateway stopped.")
        rospy.loginfo("5 second pause to let Activity Board settle after serial port reset . . .")
        time.sleep(5)  # Give it time to settle.
        self.startSerialPort()


    def _broadcast_odometry_info(self, msg):
        """
        Broadcast all data from propeller monitored sensors on the appropriate topics.
        """
        # If we got this far, we can assume that the Propeller board is initialized and the motors should be on.
        # The _switch_motors() function will deal with the _SafeToOperate issue
        if not self._motorsOn:
            self._switch_motors(True)

        x = msg.x
        y = msg.y
        # 3 is odom based heading and 4 is gyro based
        theta = msg.theta  # On ArloBot odometry derived heading works best.
        alternate_theta = msg.alternate_theta

        vx = msg.vx
        omega = msg.omega

        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(theta / 2.0)
        quaternion.w = cos(theta / 2.0)

        ros_now = rospy.Time.now()

        # First, we'll publish the transform from frame odom to frame base_link over tf
        # Note that sendTransform requires that 'to' is passed in before 'from' while
        # the TransformListener' lookupTransform function expects 'from' first followed by 'to'.
        # This transform conflicts with transforms built into the Turtle stack
        # http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29
        # This is done in/with the robot_pose_ekf because it can integrate IMU/gyro data
        # using an "extended Kalman filter"
        # REMOVE this "line" if you use robot_pose_ekf
        self._OdometryTransformBroadcaster.sendTransform(
            (x, y, 0),
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            ros_now,
            "base_footprint",
            "odom"
        )

        # next, we will publish the odometry message over ROS
        odometry = Odometry()
        odometry.header.frame_id = "odom"
        odometry.header.stamp = ros_now
        odometry.pose.pose.position.x = x
        odometry.pose.pose.position.y = y
        odometry.pose.pose.position.z = 0
        odometry.pose.pose.orientation = quaternion

        odometry.child_frame_id = "base_link"
        odometry.twist.twist.linear.x = vx
        odometry.twist.twist.linear.y = 0
        odometry.twist.twist.angular.z = omega

        # Save last X, Y and Heading for reuse if we have to reset:
        self.lastX = x
        self.lastY = y
        self.lastHeading = theta
        self.alternate_heading = alternate_theta

        # robot_pose_ekf needs these covariances and we may need to adjust them.
        # From: ~/turtlebot/src/turtlebot_create/create_node/src/create_node/covariances.py
        # However, this is not needed because we are not using robot_pose_ekf
        # odometry.pose.covariance = [1e-3, 0, 0, 0, 0, 0,
        # 0, 1e-3, 0, 0, 0, 0,
        #                         0, 0, 1e6, 0, 0, 0,
        #                         0, 0, 0, 1e6, 0, 0,
        #                         0, 0, 0, 0, 1e6, 0,
        #                         0, 0, 0, 0, 0, 1e3]
        #
        # odometry.twist.covariance = [1e-3, 0, 0, 0, 0, 0,
        #                          0, 1e-3, 0, 0, 0, 0,
        #                          0, 0, 1e6, 0, 0, 0,
        #                          0, 0, 0, 1e6, 0, 0,
        #                          0, 0, 0, 0, 1e6, 0,
        #                          0, 0, 0, 0, 0, 1e3]

        self._OdometryPublisher.publish(odometry)

    def _create_sensor_scan(self, sensor_input, sensor_config):
        # Joint State for Turtlebot stack
        # Note without this transform publisher the wheels will be white, stuck at 0, 0, 0 and RVIZ 
        # will tell you that there is no transform from the wheel_links to the base. However, instead 
        # of publishing a stream of pointless transforms, I have made the joint static in the URDF like this:
        #   create.urdf.xacro:
        #       <joint name="right_wheel_joint" type="fixed">
        #       NOTE This may prevent Gazebo from working with this model
        #       Here is the old Joint State in case you want to restore it:
        #       js = JointState(name=["left_wheel_joint", "right_wheel_joint", "front_castor_joint", "back_castor_joint"],
        #       position=[0, 0, 0, 0], velocity=[0, 0, 0, 0], effort=[0, 0, 0, 0])
        #       js.header.stamp = ros_now
        #       self.joint_states_pub.publish(js)

        # Fake laser from "PING" Ultrasonic Sensor and IR Distance Sensor input: http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF
        # Use: roslaunch arlobot_rviz_launchers view_robot.launch to view this well for debugging and testing.

        # The purpose of this is two fold:
        # 1. It REALLY helps adjusting values in the Propeller and ROS when I can visualize the sensor output in RVIZ!
        #       For this purpose, a lot of the parameters are a matter of personal taste.  Whatever makes it easiest to visualize is best.
        # 2. I want to allow AMCL to use this data to avoid obstacles that the Kinect/Xtion miss.
        #       For the second purpose, some of the parameters here may need to be tweaked, to adjust how large an object looks to AMCL.
        # Note that we should also adjust the distance at which AMCL takes this data into account either here or elsewhere.

        # Transform: http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29

        # We do not need to broadcast a transform, because it is static and contained within the URDF files now.
        # Here is the static transform for reference anyway:
        #   self._SonarTransformBroadcaster.sendTransform(
        #     (0.1, 0.0, 0.2),
        #     (0, 0, 0, 1),
        #     ros_now,
        #     "sonar_laser",
        #     "base_link"
        #   )

        # Some help:
        # http://goo.gl/ZU9XrJ

        # TODO: I'm doing this all in degrees and then converting to Radians later.
        #   Is there any way to do this in Radians?
        #   I just don't know how to create and fill an array with "Radians" since they are not rational numbers, but multiples of PI, thus the degrees.
        num_readings = 360  # How about 1 per degree?
        laser_frequency = 100  # I'm not sure how to decide what to use here.
        # This is the fake distance to set all empty slots, and slots we consider "out of range"
        artificial_far_distance = 10
        # Fill array with artificial_far_distance (not 0) and then overlap with real readings
        ranges = [artificial_far_distance] * num_readings

        # New idea here:
        # First, I do not think that this can be used for reliable for map generation.
        # If your room has objects that the Kinect cannot map, then you will probably need to modify 
        # the room (cover mirrors, etc.) or try other laser scanner options.
        # So, since we only want to use it for cost planning, we should modify the data, because
        # it is easy for it to get bogged down with a lot of "stuff" everywhere.

        # From:
        # http://answers.ros.org/question/11446/costmaps-obstacle-does-not-clear-properly-under-sparse-environment/
        # "When clearing obstacles, costmap_2d only trusts laser scans returning a definite range.  Indoors, that makes 
        # sense. Outdoors, most scans return max range, which does not clear intervening obstacles. A fake scan with 
        # slightly shorter ranges can be constructed that does clear them out."
        # So, we need to set all "hits" above the distance we want to pay attention to to a distance very far away,
        # but just within the range_max (which we can set to anything we want), otherwise costmap will not clear items!
        # Also, 0 does not clear anything! So if we rotate, then it gets 0 at that point, and ignores it, so we need to 
        # fill the unused slots with long distances.
        # NOTE: This does cause a "circle" to be drawn around the robot at the "artificalFarDistance", but it shouldn't 
        # be a problem because we set artificial_far_distance to a distance greater than the planner uses.
        # So while it clears things, it shouldn't cause a problem, and the Kinect should override it for things
        # in between.

        # Use:
        #   roslaunch arlobot_rviz_launchers view_robot.launch to view this well for debugging and testing.

        # max_range_accepted Testing:
        # TODO: More tweaking here could be done.
        # I think it is a trade-off, so there is no end to the adjustment that could be done. I did a lot of testing with gmapping while building a map.
        # Obviously this would be slightly different from using a map we do not update. It seems there are so many variables here that testing is difficult.
        # We could find one number works great in one situation but is hopeless in another. Having a comprehensive test course to test in multiple modes 
        # for every possible value would be great, but I think it would take months! :)
        # REMEMBER, the costmap only pays attention out to a certain set for obstacle_range in costmap_common_params.yaml anyway.
        # Here are my notes for different values of "max_range_accepted":
        #   1 - looks good, and works ok, but I am afraid that the costmap gets confused with things popping in and out of sight all of the time,
        #       causing undue wandering.
        #   2 - This producing less wandering due to things popping in and out of the field of view, BUT it also shows that we get odd affects at longer distances. i.e.
        #       A doorframe almost always has a hit right in the middle of it.
        #       In a hallway there is often a hit in the middle about 1.5 meters out.
        #  .5 - This works very well to have the PING data ONLY provide obstacle avoidance, and immediately forget about said obstacles.
        #       This prevents the navigation stack from fighting with the Activity Board code's built in safety stops, and instead navigate around obstacles before 
        #       the Activity Board code gets involved (other than to set speed reductions).
        #       The only down side is if you tell ArloBot to go somewhere that he cannot due to low obstacles, he will try forever. He won't just bounce off of the obstacle,
        #       but he will keep trying it and then go away, turn around, and try again over and over. He may even start wandering around the facility trying to find another way in,
        #       but he will eventually come back and try it again.  I'm not sure what the solution to this is though, because avoiding low lying obstacles and adding low lying
        #       features to the map are really two different things.  I think if this is well tuned to avoid low lying obstacles it probably will not work well for mapping features.
        #       IF we could map features with the PING sensors, we wouldn't need the 3D sensor. :)
        # TODO: One option may be more PING sensors around back. Right now when the robot spins, it clears the obstacles behind it, because there are fewer sensors on the back side.
        # If the obstacle was seen all of the way around the robot, in the same spot, it may stop returning to the same location as soon as it turns around?

        # NOTE: The bump sensors on Turtlebot mark but do not clear. I'm not sure how that works out. It seems like every bump would end up being a "blot" in the landscape never to 
        # be returned to, but maybe there is something I am missing?

        # NOTE: Could this be different for PING vs. IR?
        # Currently I'm not using IR! Just PING. The IR is not being used by costmap. It is here for seeing in RVIZ, and the Propeller board uses it for emergency stopping,
        # but costmap isn't watching it at the moment. I think it is too erratic for that. Convert cm to meters and add offset
        
        sensors = [for x in sensor_input if (x/100.0) + sensor_config["offset"] > sensor_config["threshold"] * 100 else (x/100.0) + sensor_config["offset"]]

        # Single Point code:
        for offset, position in dict["front"]:
            ranges[position] = sensors[offset]
            
        for pos in dict["rear"]:
            ranges[position] = sensors[offset]
        
        # LaserScan: http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
        scan = LaserScan()
        scan.header.stamp = ros_now
        scan.header.frame_id = sensor_config["frame id"]
        # For example:
        #       scan.angle_min = -45 * M_PI / 180; // -45 degree
        #       scan.angle_max = 45 * M_PI / 180;   // 45 degree
        # if you want to receive a full 360 degrees scan, you should try setting min_angle to -pi/2 and max_angle to 3/2 * pi.
        # Radians: http://en.wikipedia.org/wiki/Radian#Advantages_of_measuring_in_radians
        scan.angle_min = 0
        # Make sure the part you divide by num_readings is the same as your angle_max!
        # Might even make sense to use a variable here?
        scan.angle_increment = (2 * pi) / len(sensor_input)
        scan.time_increment = (1 / laser_frequency) / len(sensor_input)
        scan.range_min = sensor_config["range min"]
        scan.range_max = sensor_config["range max"]
        # This has to be above our "artificial_far_distance", otherwise "hits" at artificial_far_distance will be ignored,
        # which means they will not be used to clear the cost map!
        # in Meters Distances above this will be ignored
        scan.range_max = artificial_far_distance + 1
        scan.ranges = ranges
        
        return scan

        
    def _publish_sensor_data(self, scan, publisher):
        # "intensity" is a value specific to each laser scanner model.
        # It can safely be ignored
        publisher.publish(scan)
        
        
    def _handle_us_sensors(self, sensor_input):
        # A total of 16 ultrasonic sensors are attached to Arlobot.  They are mapped as follows:
        #   1-10 - lower deck (5 on front and 5 on back), used as fake laser
        #   11-16 - top deck (3 on front and 3 on back), used for safety
        # There are different number of sensors, 10 ultrasonic and 8 ir.  
        # Rather than directly accessing the list, it would be better to create a mapping based on
        # sensor separation.
        #            |
        #       |          |
        #  |                     |
        #
        # -------------------------
        #
        #  |                     |
        #       |          |
        #            |
        
        # Note that sensor orientation is important here!
        # If you have a different number or aim them differently this will not work!
        # TODO: Tweak this value based on real measurements!
        # TODO: Use both IR and PING sensors?
        # The offset between the pretend sensor location in the URDF and real location needs to be added to these values. This may need to be tweaked.
        #sensor_offset = 0.217 # Measured, Calculated: 0.22545
        # The sensors are 11cm from center to center at the front of the base plate.
        # The radius of the base plate is 22.545 cm
        # = 28 degree difference (http://ostermiller.org/calc/triangle.html)
        #sensor_separation = 28
        # From: http://www.parallax.com/product/28015
        # Range: approximately 1 inch to 10 feet (2 cm to 3 m)
        # This should be adjusted based on the imaginary distance between the actual laser
        # and the laser location in the URDF file.
        # in Meters Distances below this number will be ignored REMEMBER the offset!
        
        config = { "offset" : 0.217,
                   "threshold" : 0.5,
                   "range min" : 0.02, # in meters
                   "range max" : 3, # in meters
                   "frame id" : "ping_sensor_array",
                   "front" : [(5, 180 + 28*2), (6, 180 + 28), (7, 180), (8, 180 - 28), (9, 180 - 28*2)],, 
                   "rear"  : [(4, 360-28*2), (3,360-28), (2, 0), (1, 28), (0, 28*2)] 
                 }
        
        scan = self._create_sensor_scan(sensor_input, config)
        self._publish_sensor_data(scan, self._UltraSonicPublisher)
        
        
    def _handle_ir_sensors(self, sensor_input):
        config = { "offset" : 0.217,
                   "threshold" : 1000000,
                   "range min" : 0.1, # in meters
                   "range max" : 0.8, # in meters
                   "frame id" : "ir_sensor_array",
                   "front" : [(5, 180 + 28*2), (6, 180 + 28), (7, 180), (8, 180 - 28), (9, 180 - 28*2)],, 
                   "rear"  : [(4, 360-28*2), (3,360-28), (2, 0), (1, 28), (0, 28*2)] 
                 }
        
        scan = self._create_sensor_scan(sensor_input, config)
        self._publish_sensor_data(scan, self._InfraredPublisher)
        
        


    def _write_serial(self, message):
        self._SerialPublisher.publish(String(str(self._Counter) + ", out: " + message))
        self._SerialDataGateway.Write(message)


    def start(self):
        self.startSerialPort()
        self._serialTimeout = 0
        while not self._op_state_set:
            pass


    def startSerialPort(self):
        rospy.loginfo("Serial Data Gateway starting . . .")
        try:
            self._SerialDataGateway.Start()
        except:
            rospy.loginfo("ARLO SERIAL PORT Start Error")
            reset_usb_script = os.path.expanduser("~/metatron/scripts/callRestUSB.sh")
            if os.path.isfile(reset_usb_script):
                rospy.loginfo("RESETING USB PORTS")
                rospy.loginfo(reset_usb_script)
                devnull = open(os.devnull, 'w')
                subprocess.call(["/bin/bash", reset_usb_script], stdout=devnull, stderr=devnull)
                # The reset should have shut off the motors,
                # and allowing the stop method to try turning off the motors after the
                # USB port reset will make the arlobot_usbrelay node hang
                self._motorsOn = False
                # At this point we have to restart the node.
                # The respawn atribute in the launch file should handle this.
                raise SystemExit(0)
        rospy.loginfo("Serial Data Gateway started.")
        self._serialAvailable = True


    def stop(self):
        """
        Called by ROS on shutdown.
        Shut off motors, record position and reset serial port.
        """
        rospy.loginfo("Stopping")
        self._SafeToOperate = False  # Prevent threads fighting
        if self._motorsOn:
            # arlobot_usbrelay will also shut off all relays after a delay, but better safe than sorry!
            self._switch_motors( False)
        # Save last position in parameter server in case we come up again without restarting roscore!
        rospy.set_param('lastX', self.lastX)
        rospy.set_param('lastY', self.lastY)
        rospy.set_param('lastHeading', self.lastHeading)
        time.sleep(3)  # Give the motors time to shut off
        self._serialAvailable = False
        rospy.loginfo("_SerialDataGateway stopping . . .")
        try:
            self._SerialDataGateway.Stop()
        except AttributeError:
            rospy.loginfo("Attempt to start nonexistent Serial device.")
        rospy.loginfo("_SerialDataGateway stopped.")


    def _handle_velocity_command(self, twist_command):  # This is Propeller specific
        """ Handle movement requests. """
        # NOTE: turtlebot_node has a lot of code under its cmd_vel function
        # to deal with maximum and minimum speeds,
        # which are dealt with in ArloBot on the Activity Board itself in the Propeller code.
        #rospy.loginfo("_handle_velocity_command")
        if not self._op_state_set:
            return
        if self._clear_to_go("forGeneralUse"):
            v = twist_command.linear.x  # m/s
            omega = twist_command.angular.z  # rad/s
            rospy.logdebug("Handling twist command: " + str(v) + "," + str(omega))

            msg = SerialMessage(SerialMessage.ACTION_MOVE_COMMAND, [v, omega])
            self._write_serial(msg.msg)
        elif self._clear_to_go("to_stop"):
            rospy.loginfo("_clear_to_go - to_stop")
            # WARNING! If you change this check the buffer length in the Propeller C code!
            msg = SerialMessage(SerialMessage.ACTION_MOVE_COMMAND, [0.0,0.0])
            rospy.logdebug("Sending speed command message: " + msg.msg)
            self._write_serial(msg.msg)


    def _initialize_drive_geometry(self):
        rospy.loginfo("Sending drive geometry params message")
        msg = SerialMessage(SerialMessage.CONFIG_DRIVE_GEOMETRY_COMMAND, [self.track_width, self.distance_per_count])
        self._write_serial(msg.msg)


    def _initialize_op_state(self, msg):
        """ Send parameters from YAML file to Propeller board. """
        if self._SafeToOperate:
            if (self.ignore_proximity):
                ignore_proximity = 1
            else:
                ignore_proximity = 0
            if (self.ignore_cliff_sensors):
                ignore_cliff_sensors = 1
            else:
                ignore_cliff_sensors = 0
            if (self.ignore_ir_sensors):
                ignore_ir_sensors = 1
            else:
                ignore_ir_sensors = 0
            if (self.ignore_floor_sensors):
                ignore_floor_sensors = 1
            else:
                ignore_floor_sensors = 0
            if (self._acPower):
                ac_power = 1
            else:
                ac_power = 0

            rospy.loginfo("Sending operational state params message")
            msg = SerialMessage(SerialMessage.CONFIG_OP_STATE_COMMAND,
                                [ignore_proximity, ignore_cliff_sensors, ignore_ir_sensors, ignore_floor_sensors, ac_power, self.lastX, self.lastY, self.lastHeading])
            self._write_serial(msg.msg)

        else:
            self._pirPublisher.publish(msg.motion_detected)


    def _switch_motors(self, state):
        """ Switch Motors on and off as needed. """
        # Relay control was moved to its own package
        if self.relayExists:
            if not self._SwitchingMotors:  # Prevent overlapping runs
                self._SwitchingMotors = True
                # Switch "on" to "off" if not safe to operate,
                # then we can just pass state to arlobot_usbrelay
                if not self._SafeToOperate:
                    state = False
                rospy.wait_for_service('/arlobot_usbrelay/toggle_relay')
                rospy.loginfo("Switching motors.")
                try:
                    toggle_relay = rospy.ServiceProxy('/arlobot_usbrelay/toggle_relay', ToggleRelay)
                    left_relay_result = toggle_relay(self.usbLeftMotorRelayLabel, state)
                    right_relay_result = toggle_relay(self.usbRightMotorRelayLabel, state)
                    if left_relay_result.toggleSuccess and right_relay_result.toggleSuccess:
                        self._motorsOn = True
                    else:
                        self._motorsOn = False
                except rospy.ServiceException as e:
                    rospy.loginfo("Service call failed: %s" % e)
                self._SwitchingMotors = False
        else:  # If no automated motor control exists, just set the state blindly.
            self._motorsOn = state


    def watchDog(self):
        while not rospy.is_shutdown():

            #msg = SerialMessage(SerialMessage.ACTION_MOVE_COMMAND, [0.5, 0.0])
            #self._write_serial(msg.msg)

            if self._serialAvailable:
                self._serialTimeout += 1
            else:
                self._serialTimeout = 0
            #rospy.loginfo("Serial Timeout = " + str(self._serialTimeout))
            if self._serialTimeout > 19:
                rospy.loginfo("Watchdog Timeout Reset initiated")
                self._reset_serial_connection()

            if self._unPlugging or self._wasUnplugging:
                self.UnplugRobot()

            old_track_width = self.track_width
            self.track_width = rospy.get_param("~driveGeometry/trackWidth", "0.403")
            if not old_track_width == self.track_width:
                self.robotParamChanged = True

            old_distance_per_count = self.distance_per_count
            self.distance_per_count = rospy.get_param("~driveGeometry/distancePerCount", "0.00676")
            if not old_distance_per_count == self.distance_per_count:
                self.robotParamChanged = True

            old_ignore_proximity = self.ignore_proximity
            self.ignore_proximity = rospy.get_param("~ignoreProximity", False);
            if not old_ignore_proximity == self.ignore_proximity:
                self.robotParamChanged = True

            old_ignore_cliff_sensors = self.ignore_cliff_sensors
            self.ignore_cliff_sensors = rospy.get_param("~ignoreCliffSensors", False);
            if not old_ignore_cliff_sensors == self.ignore_cliff_sensors:
                self.robotParamChanged = True

            old_ignore_ir_sensors = self.ignore_ir_sensors
            self.ignore_ir_sensors = rospy.get_param("~ignoreIRSensors", False);
            if not old_ignore_ir_sensors == self.ignore_ir_sensors:
                self.robotParamChanged = True

            old_ignore_floor_sensors = self.ignore_floor_sensors
            self.ignore_floor_sensors = rospy.get_param("~ignoreFloorSensors", False);
            if not old_ignore_floor_sensors == self.ignore_floor_sensors:
                self.robotParamChanged = True

            if self.robotParamChanged:
                if (self.ignore_proximity):
                    ignore_proximity = 1
                else:
                    ignore_proximity = 0
                if (self.ignore_cliff_sensors):
                    ignore_cliff_sensors = 1
                else:
                    ignore_cliff_sensors = 0
                if (self.ignore_ir_sensors):
                    ignore_ir_sensors = 1
                else:
                    ignore_ir_sensors = 0
                if (self.ignore_floor_sensors):
                    ignore_floor_sensors = 1
                else:
                    ignore_floor_sensors = 0
                if (self._acPower):
                    ac_power = 1
                else:
                    ac_power = 0
                    
                msg = SerialMessage(SerialMessage.CONFIG_DRIVE_GEOMETRY_COMMAND,
                                    [self.track_width, self.distance_per_count])
                self._write_serial(msg.msg)
                msg = SerialMessage(SerialMessage.CONFIG_OP_STATE_COMMAND,
                                    [ignore_proximity, ignore_cliff_sensors, ignore_ir_sensors, ignore_floor_sensors, ac_power, self.lastX, self.lastY, self.lastHeading])
                self._write_serial(msg.msg)
                self.robotParamChanged = False

            self.r.sleep()


    def UnplugRobot(self):
        rospy.loginfo("UnplugRobot")
        if self._unPlugging and \
                not self._wasUnplugging and \
                self._clear_to_go("forUnplugging"):
            # We will only do this once, and let it continue until AC power is disconnected
            self._wasUnplugging = True
            # Slow backup until unplugged
            # This should be a slow backward crawl
            # -0.01 is about as slow as possible
            # -0.02 works more reliably
            rospy.loginfo("Unplugging!")
            msg = SerialMessage(SerialMessage.ACTION_MOVE_COMMAND, [-0.02,0.0])
            self._write_serial(msg.msg)
        # Once we are unplugged, stop the robot before returning control to handle_velocity_command
        # And we only need permission to stop at this point.
        if self._wasUnplugging and \
                not self._acPower and \
                self._serialAvailable:
            rospy.loginfo("Unplugging complete")
            self._wasUnplugging = False
            msg = SerialMessage(SerialMessage.ACTION_MOVE_COMMAND, [0.0,0.0])
            self._write_serial(msg.msg)
        # Finally, if we were unplugging, but something went wrong, we should stop the robot
        # Since no one else will do this while we have the "_wasUnplugging" variable
        # set.
        if self._wasUnplugging and \
                not self._clear_to_go("forUnplugging") and \
                self._serialAvailable:
            self._wasUnplugging = False
            msg = SerialMessage(SerialMessage.ACTION_MOVE_COMMAND, [0.0,0.0])
            self._write_serial(msg.msg)

    # Considlate "clear to go" requirements here.
    def _clear_to_go(self, forWhat):
        return_value = False

        return True
        rospy.loginfo(str(self._serialAvailable) + "," + str(self._SafeToOperate) + "," + str(self._safeToGo) + "," + str(self._motorsOn) + "," + str(self._leftMotorPower) + "," + str(self._rightMotorPower))

        # Required for all operations
        if self._serialAvailable and \
           self._SafeToOperate and \
           self._safeToGo and \
           self._motorsOn and \
           self._leftMotorPower and \
           self._rightMotorPower:
               return_value = True

        rospy.loginfo(str(self._acPower) + "," + str(self._wasUnplugging))
        # Negations by use case
        if forWhat == "forUnplugging":
            # Unpugging should only happen if AC is connected
            if not self._acPower:
               return_value = False
        if forWhat == "forGeneralUse":
            # The handle_velocity_command should only operate if the robot is unplugged,
            # and the unplugging function of the Watchdog process is not in control
            if self._acPower or self._wasUnplugging:
                return_value = False
        # Special Cases
        if forWhat == "to_stop":
            # handle_velocity_command can send a STOP if it is unsafe to do anythging else,
            # but the serial connection is up,
            # as long as the unplugging operation is not in progress
            # This should bring the robot to a halt in response to any other requset,
            # if anything is amiss.
            if self._serialAvailable and \
                    not self._wasUnplugging:
                return_value = True
            else:
                return_value = False
        return return_value


if __name__ == '__main__':

    propellercomm = PropellerComm()
    rospy.on_shutdown(propellercomm.stop)
    try:
        propellercomm.start()
        rospy.loginfo("Propellerbot_node has started.")
        propellercomm.watchDog()

    except rospy.ROSInterruptException:
        propellercomm.stop()
