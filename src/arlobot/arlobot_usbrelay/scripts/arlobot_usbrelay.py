#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Author: Chris L8 https://github.com/chrisl8
# URL: https://github.com/chrisl8/ArloBot

import rospy
import tf
import sys
import time

from std_msgs.msg import String
from std_msgs.msg import Bool
from arlobot_msgs.msg import usbRelayStatus
from arlobot_msgs.srv import *

from usb_to_gpio import UsbToGpio, UsbToGpioError


class UsbRelayError(Exception):
    pass

class UsbRelay(object):
    '''
    Helper class for communicating with a Propeller board over serial port
    '''

    class __RelayIo:
        RelayPin = -1
        StatusPin = -1
        CommandedState = False
        ActualState = False
        LastState = False

        def __init__(self, relay_pin, status_pin, initial_state):
            self.RelayPin = relay_pin
            self.StatusPin = status_pin
            self.CommandedState = initial_state
            self.ActualState = initial_state
            self.LastState = initial_state


    # Define the relays and status that are supported by this service
    #
    #   * Activity board power
    #   * Left Motor power
    #   * Right Motor power
    #   * Activity board status
    #   * Left Motor status
    #   * Right Motor status

    NUM_RELAYS = 8

    ACTIVITY_BOARD_RELAY_PIN = UsbToGpio.PIN_C0
    ACTIVITY_BOARD_STATUS_PIN = UsbToGpio.PIN_C1
    LEFT_MOTOR_RELAY_PIN = UsbToGpio.PIN_C2
    RIGHT_MOTOR_RELAY_PIN = UsbToGpio.PIN_C2
    LEFT_MOTOR_STATUS_PIN = UsbToGpio.PIN_C3
    RIGHT_MOTOR_STATUS_PIN = UsbToGpio.PIN_C4



    def __init__(self):
        rospy.init_node('arlobot_usbrelay')

        # http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber
        self.r = rospy.Rate(0.25) # 4hz refresh rate

        # Wait for the arlobot_bringup launch file to initiate the usbRelayInstalled parameter before starting:
        while not rospy.has_param('/arlobot/usbRelayInstalled') and not rospy.is_shutdown():
            rospy.loginfo("arlobot_bringup not started yet, waiting . . .")
            rospy.sleep(1)

        self._relay_exists = rospy.get_param("/arlobot/usbRelayInstalled", False)
        if self._relay_exists:

            try:
                self._usb_to_gpio = UsbToGpio()
            except UsbToGpioError as utge:
                raise UsbRelayError("FATAL: Unable to instantiate UsbToGpio", utge)

            self._relay_io_map = {"Activity Board" : self.__RelayIo(self.ACTIVITY_BOARD_RELAY_PIN, self.ACTIVITY_BOARD_STATUS_PIN, False),
                                  "Left Motor"     : self.__RelayIo(self.LEFT_MOTOR_RELAY_PIN, self.LEFT_MOTOR_RELAY_PIN, False),
                                  "Right Motor"    : self.__RelayIo(self.RIGHT_MOTOR_RELAY_PIN, self.RIGHT_MOTOR_RELAY_PIN, False)
                                 }

            # Create a service that can be called to toggle any relay by name:
            # http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
            # http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29
            relayToggle = rospy.Service('~relay_toggle', RelayToggle, self._RelayToggle)
            relaySetState = rospy.Service('~relay_set_state', RelaySetState, self._RelaySetState)
            relayResetSeq = rospy.Service('~relay_reset_seq', RelayResetSequence, self._RelayResetSequence)
            relayEnum = rospy.Service('~enum_relay', RelayEnumerate, self._RelayEnumerate)

            # Publishers
            self._usbRelayStatusPublisher = rospy.Publisher('~usbRelayStatus', usbRelayStatus, queue_size=1)
        else:
            rospy.loginfo("No USB Relay board installed.")

    def __set_state(self, io, state):
        '''
        Descript: Internal routine to set the relay state, update the state variables, and return success or failure
        :param io: the Relay IO state variable
        :param state: the state to which the Relay is to be set
        :return: True if the actual and commanded states are equal; otherwise False
        '''
        io.LastState = io.CommandedState
        io.CommandedState = state
        self._usb_to_gpio.SetOutput(io.RelayPin, io.CommandedState)
        io.ActualState = self._usb_to_gpio.GetInput(io.StatusPin)

        if io.ActualState == io.CommandedState:
            return True
        else:
            return False

    def _RelayToggle(self, req):
        '''
        Description: Exposes a service toggle method for changing the state of the relay
        :param req: the relay description
        :return: the actual state and whether the change was successful
        '''
        if self._relay_exists:
            io = self._relay_io_map[req.Name]
            success = self.__set_state(io, io.CommandedState ^ True)
            return (io.ActualState, success)
        else:
            return (False, True)

    def _RelaySetState(self, req):
        '''
        Description: Exposes a service set state method for setting the state of the relay
        :param req: the relay description
        :return: the actual state and whether the change was successful
        '''
        if self._relay_exists:
            io = self._relay_io_map[req.Name]
            success = self.__set_state(io, req.State)
            return (io.ActualState, success)
        else:
            return (False, True)

    def _RelayResetSequence(self, req):
        '''
        Description: Exposes a service relay reset sequence for resetting a device connected to a relay
        :param req: the relay description and a delay time
        :return: the actual state and whether the change was successful
        '''
        if self._relay_exists:
            io = self._relay_io_map[req.Name]

            success = self.__set_state(io, False)
            if success:
                rospy.sleep(req.Delay)
                success = self.__set_state(io, True)

            return (io.ActualState, success)

        else:
            return (False, True)


    def _RelayEnumerate(self):
        '''
        Description: Exposes a service relay enumeration method for getting the list of supported relays
        :return: a list of relay names (keys)
        '''
        return self._relay_io_map.keys()


    def Run(self):
        '''
        Description: main loop for the service node which performs checking on the change state of the relays and
                     publishes a message when there is a change
        :return: None
        '''
        while not rospy.is_shutdown():
            relay_status = usbRelayStatus()

            if self._relay_exists:
                for key, io in self._relay_io_map.iteritems():
                    if not io.LastState is io.CommandedState:
                        relay_status.Name.append(key)
                        relay_status.States.append(io.ActualState)
            else:
                relay_status.Names = ["Unused"] * self.NUM_RELAYS
                relay_status.States = [False] * self.NUM_RELAYS

            self._usbRelayStatusPublisher.publish(relay_status) # Publish USB Relay status
            self.r.sleep() # Sleep long enough to maintain the rate set in __init__


    def Stop(self):
        '''
        Description: stop service method called when ROS is shutting down.  Turns off all relays before exiting.
        :return: None
        '''
        #rospy.sleep(5) # Give other nodes a chance to clean up before shutting down our services.
        rospy.loginfo("Shutting off all enabled relays . . .")

        for key, io in self._relay_io_map:
            self.__set_state(io, False)


if __name__ == '__main__':
    node = UsbRelay()
    rospy.on_shutdown(node.Stop)
    try:
        node.Run()
    except (UsbRelayError, rospy.ROSInterruptException):
        node.Stop()
