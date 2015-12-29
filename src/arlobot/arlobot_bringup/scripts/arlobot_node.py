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


import rospy
import tf
from math import sin, cos, pi
import time
import json
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

from serial_msg import SerialMessage, SerialMessageError
from prop_comms import PropellerComms, PropellerCommsError
from relay_bank import RelayBank, RelayBankError
from op_state import OperationalState, OperationalStateError

class SafetyCheckError(Exception):
    pass

class SafetyCheck:
    '''
    I think this should be inside the safety check class.  Here in ArlobotNode, safety
    should be GO-NOGO.  So, at critical points we will perform safety checks and either
    they pass or fail.  If they fail, we stop (or perform some sort of recovery).  How
    we get to failure will be hidden in the safety check object and can depend on a lot of factors.
    '''
    def Evaluate(self):
        return True

    def Passed(self):
        return True

    def Details(self):
        return "These are the safety check details"

    '''
    I'm not really sure what is going here.  Obviously it has to do with handling
    when the robot is connected to the charger.  Shouldn't all of this be part of
    safety?  Isn't the docking and undocking of the robot a safety issue?  How about
    power level?  Is it unsafe to run out of power?  Potentially, someone could trip
    over the robot!!!
    It would be nice to decide what is a safety concern based on parameters and to
    do that when the safety check is instantiated.  Maybe safety should be an interface
    that can be implemented in the relevant classed/objects.  When an object discovers
    it is an unsafe state is reports via the safety interface.  The safety check just
    evaluates all of the existing conditions.

    class SafetyBase:
        def __init__(self):
            self._is_safe = True
            self._details = ""
            pass

        def IsSafe(self):
            # return either True or False
            # return True it is safe; otherwise, False
            return self._is_safe

        def Unsafe(self, details):
            self._is_safe = False
            self._details = details

        def Details(self):
            # Return details of the safety check
            return self._details

    class PowerDock(SafetyBase):
        def __init__(self):
            pass

        def IsSafe(self):
            return SafetyBase.IsSafe()

        def RequestDock(self):
            # Call SafetyBase.Unsafe with details as appropriate
            pass

    class SafetyCheck:
        def __init__(self):
            self._checks = []
        def Add(self, check):
            # Add a new object to the list of checks.  Each object implements the SafetyBase interface
            # Check is object is instance of SafetyBase
            self._checks.append(check)

        def Evaluate(self):
            for check in self._checks:
                if not check.IsSafe():
                    self._failed_check = check
                    return False

            return True

        def Query(self):
            return self._failed_check.Details()
    '''




class ArlobotNodeError(Exception):
    pass


class ArlobotNode(object):
    '''
    Node for managing and communicating with the robot hardware
    '''

    def __init__(self):

        enable_debug = rospy.get_param('debug', False)
        if enable_debug:
            rospy.init_node('arlobot', log_level=rospy.DEBUG)
        else:
            rospy.init_node('arlobot')

        self.r = rospy.Rate(1) # 1hz refresh rate

        # Store last x, y and heading for reuse when we reset
        # I took off the ~, because that was causing these to reset to default on every restart even if roscore was still up.
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


        self._op_state = OperationalState()

        try:
            # If you don't want a relay, this call will be successful and
            # return a mock object which successfully goes through the motions.
            self._relay_bank = RelayBank(self._relay_status)
        except RelayBankError as rbe:
            rospy.logwarn(rbe)
            # Note, if you wanted a relay and we can't give one to you, we're done.
            raise ArlobotNodeError("ERROR: Unable to create relay bank.")

        try:
            self._prop_comms = PropellerComms(self._handle_received_message)
        except PropellerCommsError as pce:
            rospy.logwarn(pce)
            # Note, we cannot continue if we don't have communications
            raise ArlobotNodeError("ERROR: Unable to establish serial communications.")

        try:
            # If you don't want a safety check, this call will be successful and
            # return a mock object which successfully goes through the motions
            self._safety_check = SafetyCheck()
        except SafetyCheckError as sce:
            rospy.logwarn(sce)
            raise ArlobotNodeError("ERROR: Unable to create safety check.")


        # Subscriptions
        rospy.Subscriber("cmd_vel", Twist, self._handle_velocity_command)

        # Publishers
        self._pirPublisher = rospy.Publisher('~pirState', Bool, queue_size=1)
        self._arlo_status_publisher = rospy.Publisher('arlo_status', arloStatus, queue_size=1)

        # If the Odometry Transform is done with the robot_pose_ekf do not publish it,
        # but we are not using robot_pose_ekf, because it does nothing for us if you don't have a full IMU!
        self._OdometryTransformBroadcaster = tf.TransformBroadcaster()  # REMOVE this line if you use robot_pose_ekf
        self._OdometryPublisher = rospy.Publisher("odom", Odometry, queue_size=10)

        # We don't need to broadcast a transform, as it is static and contained within the URDF files
        # self._SonarTransformBroadcaster = tf.TransformBroadcaster()
        self._UltraSonicPublisher = rospy.Publisher("ultrasonic_scan", LaserScan, queue_size=10)
        self._InfraredPublisher = rospy.Publisher("infrared_scan", LaserScan, queue_size=10)

        try:
            self._op_state.Update(initialize = True)
        except OperationalStateError as ose:
            rospy.logfatal(ose)
            raise ArlobotNodeError("FATAL: ", ose)

    def _relay_status(self, data):
        # publish the arlobot status on status change in relay state
        pass

    def _handle_received_message(self, msg):
        """
        This will run every time a message is received from the Propeller board
        and will send the data to the correct function.
        """

        if msg.msg_class == SerialMessage.STATUS_CLASS:
            if msg.msg_type == SerialMessage.STATUS_ODOMETRY_MESSAGE:
                self._broadcast_odometry_info(msg)

            elif msg.msg_type == SerialMessage.STATUS_OP_STATE_MESSAGE:
                # First, the robot needs to be configured with the drive geometry and operational state
                # There is a message for drive geometry and one for operational state
                if msg.drive_geometry_received:
                    try:
                        self._op_state.Update(drive_geometry_received = msg.drive_geometry_received)
                    except OperationalStateError as ose:
                        rospy.logfatal(ose)
                        raise ArlobotNodeError("FATAL: ", ose)
                else:
                    rospy.logwarn("Need to config drive geometry")
                    self._initialize_drive_geometry()

                if msg.op_state_received:
                    try:
                        self._op_state.Update(op_state_received = msg.op_state_received)
                    except OperationalStateError as ose:
                        rospy.logfatal(ose)
                        raise ArlobotNodeError("FATAL: ", ose)
                else:
                    rospy.logwarn("Need to config operational state")
                    self._initialize_op_state()

                if msg.motion_detected:
                    self._pirPublisher.publish(msg.motion_detected)

                # We are now operational
                if self._op_state.IsOperational():
                    self._broadcast_arlo_status(msg)

            elif msg.msg_type == SerialMessage.STATUS_ULTRASONIC_SENSOR_MESSAGE:
                # Publish Ultrasonic data
                # Note: the mapping of sensors to higher-level constructs, lower deck, upper deck, front lower, back lower
                # needs to be done here as well.  Maybe there should be a mapping param that gives a label to indicies, 
                # e.g., "front lower deck" : 0 .. 4, "rear lower deck" : 5 .. 9, then that information could be used
                # to publish the raw sensor data into those contructs.
                self._handle_us_sensors(msg.payload)
                
            elif msg.msg_type == SerialMessage.STATUS_ANALOG_IR_SENSOR_MESSAGE:
                # Publish Infrared data
                self._handle_ir_sensors(msg.payload)

        elif msg.msg_class == SerialMessage.DEBUG_CLASS:
            # Map the levels from the debug message to rospy log levels, e.g. info, warn, debug, fatal, etc
            # and then write them to rospy.logxxx.
            '''
            if msg.level == SerialMessage.DEBUG_LEVEL_INFO:
                rospy.loginfo(msg.msg)
            elif msg.level == SerialMessage.DEBUG_LEVEL_WARN:
                rospy.loginfo(msg.msg)
            elif msg.level == SerialMessage.DEBUG_LEVEL_DEBUG:
                rospy.logdebug(msg.msg)
            elif msg.level == SerialMessage.DEBUG_LEVEL_FATAL:
                rospy.logfatal(msg.msg)
            '''
            pass

        else:
            # Log something about this bad message
            rospy.logwarn("Unknown message class: " + str(msg))

    def _broadcast_arlo_status(self, msg):
        '''
        Description - broadcasts the ArlobotNode status
        :param msg:
        :return: None
        '''

        arlo_status = arloStatus()

        arlo_status.safeToProceed = msg.safe_to_proceed
        arlo_status.safeToRecede = msg.safe_to_recede
        arlo_status.Escaping = msg.escaping
        arlo_status.abd_speedLimit = msg.max_forward_speed
        arlo_status.abdR_speedLimit = msg.max_reverse_speed
        arlo_status.Heading = self.lastHeading
        arlo_status.gyroHeading = self.alternate_heading
        arlo_status.minDistanceSensor = msg.min_distance_sensor
        arlo_status.leftMotorVoltage = msg.left_motor_voltage
        arlo_status.rightMotorVoltage = msg.right_motor_voltage
        arlo_status.leftMotorCurrent = msg.left_motor_current
        arlo_status.rightMotorCurrent = msg.left_motor_current
        arlo_status.cliffDetected = msg.cliff_detected
        arlo_status.floorObstacleDetected = msg.floor_obstacle_detected

        # Who need to use ArloStatus?
        self._arlo_status_publisher.publish(arlo_status)

    def _broadcast_odometry_info(self, msg):
        """
        Broadcast all data from propeller monitored sensors on the appropriate topics.
        """
        x = msg.x
        y = msg.y
        # theta is odom based heading and alternate_theta is gyro based
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

        sensors = []
        for value in sensor_input:
            adj_value = (float(value)/100.0) + sensor_config["offset"]
            if adj_value > sensor_config["threshold"] * 100:
                sensors.append(float(value))
            else:
                sensors.append(adj_value)

        #sensors = [for x in sensor_input if (x/100.0) + sensor_config["offset"] > sensor_config["threshold"] * 100 else (x/100.0) + sensor_config["offset"]]

        # Fill the ranges array with the sensor values
        for offset, position in sensor_config["front"]:
            ranges[position] = sensors[offset]
            
        for offset, position in sensor_config["rear"]:
            ranges[position] = sensors[offset]
        
        # LaserScan: http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html

        ros_now = rospy.Time.now()

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


        # I was thinking that maybe it would be a good idea to apply some calibration adjustments here.  Each ultrasonic
        # sensor is slightly different.  There are limited resources on the Activity board to store calibration, but
        # there is plenty of room on the PC.  There could be a robot function that would take multiple reading from
        # each sensor where an obstruction is applied to each sensor at a known distance.  The calibration can be
        # stored in a file, read in on startup, and applied to sensor_input before creating the scan
        
        config = { "offset" : 0.217,
                   "threshold" : 0.5,
                   "range min" : 0.02, # in meters
                   "range max" : 3, # in meters
                   "frame id" : "ping_sensor_array",
                   # The layout of the ultrasonic sensors is as follows:
                   # Front (forward facing):
                   #     0 - Lower Far Left
                   #     1 - Lower Left
                   #     2 - Lower Mid
                   #     3 - Lower Right
                   #     4 - Lower Far Right
                   #     Note: The front has 3 ultrasonic sensors on the middle tier, indecies 5 through 7
                   # Rear (backward facing):
                   #     8 - Lower Far Left
                   #     9 - Lower Left
                   #    10 - Lower Mid
                   #    11 - Lower Right
                   #    12 - Lower Far Right
                   #    Note: The rear has 3 ultrasonic sensors on the middle tier, indecies 13 through 15
                   # Note: There are 28 degrees between each sensor
                   "front" : [(12, 180 + 28*2), (11, 180 + 28), (10, 180), (9, 180 - 28), (8, 180 - 28*2)],
                   "rear"  : [(4, 360 - 28*2), (3, 360 - 28), (2, 0), (1, 28), (0, 28*2)]
                 }
        
        scan = self._create_sensor_scan(sensor_input, config)
        self._publish_sensor_data(scan, self._UltraSonicPublisher)

    def _handle_ir_sensors(self, sensor_input):
        config = { "offset" : 0.217,
                   "threshold" : 1000000,
                   "range min" : 0.1, # in meters
                   "range max" : 0.8, # in meters
                   "frame id" : "ir_sensor_array",
                   # The layout of the infrared sensors is as follows:
                   # Front (forward facing):
                   #     Note: Indecies 0 and 1 are for floor obstacle and cliff detection.  There are no other IR
                   #           sensors on the lower level
                   #     2 - Mid Far Left
                   #     3 - Mid Left
                   #     4 - Mid Right
                   #     5 - Mid Far Right
                   #     6 - Upper Left
                   #     7 - Upper Right
                   # Rear (backward facing):
                   #     Note: Indecies 8 and 9 are for floor obstacle and cliff detection.  There are no other IR
                   #           sensors on the lower level
                   #    10 - Mid Far Left
                   #    11 - Mid Left
                   #    12 - Mid Right
                   #    13 - Mid Far Right
                   #    14 - Upper Left
                   #    15 - Upper Right
                   "front" : [(5, 180 + 28*2), (6, 180 + 28), (7, 180), (8, 180 - 28), (9, 180 - 28*2)],
                   "rear"  : [(4, 360 - 28*2), (3, 360 - 28), (2, 0), (1, 28), (0, 28*2)]
                 }
        
        scan = self._create_sensor_scan(sensor_input, config)
        self._publish_sensor_data(scan, self._InfraredPublisher)

    def _handle_velocity_command(self, twist_command):
        """ Handle movement requests. """
        # NOTE: turtlebot_node has a lot of code under its cmd_vel function to deal with maximum and minimum speeds,
        # which are dealt with in ArloBot on the Activity Board itself in the Propeller code.

        v = 0.0  # m/s
        omega = 0.0  # rad/s

        if self._op_state.IsOperational():
            v = twist_command.linear.x  # m/s
            omega = twist_command.angular.z  # rad/s

        try:
            #rospy.loginfo("Sending move: " + str(v) + "," + str(omega))
            msg = SerialMessage(SerialMessage.ACTION_MOVE_COMMAND, [v, omega])
        except SerialMessageError as sme:
            rospy.logfatal("FATAL: " + str(sme))
            raise ArlobotNodeError("FATAL: Unable to create move serial message:", str(sme))
        self._prop_comms.SendMessage(msg)

    def _initialize_drive_geometry(self):
        rospy.loginfo("Sending drive geometry params message")
        try:
            msg = SerialMessage(SerialMessage.CONFIG_DRIVE_GEOMETRY_COMMAND, [self.track_width, self.distance_per_count])
        except SerialMessageError as sme:
            rospy.logfatal("FATAL: " + str(sme))
            raise ArlobotNodeError("FATAL: Unable to create drive geometry serial message:", str(sme))

        self._prop_comms.SendMessage(msg)

    def _initialize_op_state(self, disable_safety = False):
        rospy.loginfo("Sending operational state params message")
        if (disable_safety):
            rospy.loginfo("Disabling safety")
            self.ignore_proximity = True
            self.ignore_cliff_sensors = True
            self.ignore_ir_sensors = True
            self.ignore_floor_sensors = True

        try:
            msg = SerialMessage(SerialMessage.CONFIG_OP_STATE_COMMAND,
                                [int(self.ignore_proximity),
                                 int(self.ignore_cliff_sensors),
                                 int(self.ignore_ir_sensors),
                                 int(self.ignore_floor_sensors),
                                 int(True),
                                 self.lastX,
                                 self.lastY,
                                 self.lastHeading])
        except SerialMessageError as sme:
            rospy.logfatal("FATAL: " + str(sme))
            raise ArlobotNodeError("FATAL: Unable to create op state serial message:", str(sme))

        self._prop_comms.SendMessage(msg)

    def _save_state(self):
        # Save last position in parameter server in case we come up again without restarting roscore!
        rospy.set_param('lastX', self.lastX)
        rospy.set_param('lastY', self.lastY)
        rospy.set_param('lastHeading', self.lastHeading)
        # Should we save the operational state as a parameter?

    def _check_param_update(self):
        param_update = False
        old_track_width = self.track_width
        self.track_width = rospy.get_param("~driveGeometry/trackWidth", "0.403")
        if not old_track_width == self.track_width:
            param_update = True

        old_distance_per_count = self.distance_per_count
        self.distance_per_count = rospy.get_param("~driveGeometry/distancePerCount", "0.00676")
        if not old_distance_per_count == self.distance_per_count:
            param_update = True

        old_ignore_proximity = self.ignore_proximity
        self.ignore_proximity = rospy.get_param("~ignoreProximity", False)
        if not old_ignore_proximity == self.ignore_proximity:
            param_update = True

        old_ignore_cliff_sensors = self.ignore_cliff_sensors
        self.ignore_cliff_sensors = rospy.get_param("~ignoreCliffSensors", False)
        if not old_ignore_cliff_sensors == self.ignore_cliff_sensors:
            param_update = True

        old_ignore_ir_sensors = self.ignore_ir_sensors
        self.ignore_ir_sensors = rospy.get_param("~ignoreIRSensors", False)
        if not old_ignore_ir_sensors == self.ignore_ir_sensors:
            param_update = True

        old_ignore_floor_sensors = self.ignore_floor_sensors
        self.ignore_floor_sensors = rospy.get_param("~ignoreFloorSensors", False)
        if not old_ignore_floor_sensors == self.ignore_floor_sensors:
            param_update = True

        return param_update

    def Start(self):
        '''
        Description - Responsible for starting the components of the ArlobotNode.
            The Activity board micro-controller must be powered up and
            Then the serial communications gateway needs to be started.
        '''
        rospy.loginfo("ArlobotNode: starting ...")

        try:
            rospy.loginfo("ArlobotNode: reseting on Activity board.")
            self._relay_bank.ResetActivityBoard()

        except PropellerCommsError:
            raise ArlobotNodeError("FATAL: Failed to reset Activity board.")

        try:
            rospy.loginfo("ArlobotNode: starting serial communication.")
            self._prop_comms.Startup()
        except PropellerCommsError:
            raise ArlobotNodeError("FATAL: Failed to start serial communication.")

        rospy.loginfo("ArlobotNode: started")

        # No point in doing anything until we are operational
        # Maybe we need to logging something if this goes on for too long
        while not self._op_state.IsOperational():
            if self._op_state.IsConfigured():
                try:
                    self._op_state.Update(running = True)
                    break
                except OperationalStateError as ose:
                    rospy.logfatal(ose)
                    raise ArlobotNodeError("FATAL: ", ose)


            # Add some type of time checking here.  If we haven't progressed after some amount of time then
            # raise an exception, reset the comms, reset the Activity board, or worst case shutdown and let
            # ROS restart us.
            time.sleep(1)
            #rospy.logwarn("Waiting for operational state")
            continue

        rospy.logwarn("Arlobot is operational: " + str(self._op_state.IsOperational()))


    def Stop(self, reason):
        """
        Called by ROS on shutdown.
        Shut off motors, record position and reset serial port.
        """
        rospy.loginfo("ArlobotNode: stopping due to " + str(reason))

        self._op_state.Update(shutdown = True)
        self._save_state()
        self._prop_comms.Shutdown()
        self._relay_bank.StopActivityBoard()

        rospy.loginfo("ArlobotNode: stopped")

    def WatchDog(self):
        while not rospy.is_shutdown():

            if self._prop_comms.Timeout():
                self._prop_comms.Reset()

            if not self._safety_check.Evaluate():
                # Maybe we should publish the safety state when its bad and then the espeak node can give notice
                #self._safety_publisher.publish( ... )
                rospy.logfatal("Houston, we have a problem!")
                raise ArlobotNodeError("FATAL: Safety check failed: ", self._safety_check.Details())


            updated = self._check_param_update()
            if updated:
                op_state = [int(self.ignore_proximity),
                            int(self.ignore_cliff_sensors),
                            int(self.ignore_ir_sensors),
                            int(self.ignore_floor_sensors),
                            int(True)]

                try:
                    msg = SerialMessage(SerialMessage.CONFIG_DRIVE_GEOMETRY_COMMAND,
                                        [self.track_width, self.distance_per_count])
                except SerialMessageError as sme:
                    rospy.logfatal("FATAL: " + str(sme))
                    raise ArlobotNodeError("FATAL: Unable to create drive geometry serial message:", str(sme))

                self._prop_comms.SendMessage(msg)

                try:
                    msg = SerialMessage(SerialMessage.CONFIG_OP_STATE_COMMAND,
                                        op_state + [self.lastX, self.lastY, self.lastHeading])
                except SerialMessageError as sme:
                    rospy.logfatal("FATAL: " + str(sme))
                    raise ArlobotNodeError("FATAL: Unable to create op state serial message:", str(sme))

                self._prop_comms.SendMessage(msg)

            self.r.sleep()

    # Replace UnplugRobot and all related plugging and unplugging with a Dock class
    # So, either the robot is docked or undocked.  Maybe make this a service, since
    # we make requests and then wait for a response, either a status or motion.
    # Also, as a service, it can be decoupled and included only if you want or need it.
    # class ArlobotDock(object):
    #     def __init__(self):
    #         pass
    #     def IsDocked(self):
    #         pass
    #     def IsUndocked(self):
    #         pass
    #     def IsDocking(self):
    #         pass
    #     def RequestDock(self):
    #         pass
    #     def RequestUndock(self):
    #         pass
    # It would be nice to encapsulate everything about docking within the ArlobotDock class
    # Push all of the motion commands to parameters and let ArlobotDock get the values from
    # params, and then let ArlobotDock give ArlobotNode the values via a callback


if __name__ == '__main__':
    '''
    The main routine for the ArlobotNode
    Here we do the following:
        * create the ArlobotNode,
        * register a routine to be called by ROS shutdown
        * start the node and loop until there is an error or a shutdown
    '''
    arlobot = ArlobotNode()
    rospy.on_shutdown(arlobot.Stop)
    try:
        arlobot.Start()
        arlobot.WatchDog()
    except (ArlobotNodeError, rospy.ROSInterruptException) as err:
        arlobot.Stop(err)
