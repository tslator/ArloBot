from __future__ import print_function

import rospy
from arlobot_msgs.msg import usbRelayStatus
from arlobot_msgs.srv import RelaySetState, RelayResetSequence

class RelayBankError(Exception):
    pass

class RelayBank:
    '''
    The purpose of the RelayBank is to setup a proxy between the Arlobot and Relay Service.  Relays are a global resource
    that should be available to all that need to know about either status or controlling relays

    Services needed from Arlobot
        1. Start, Stop, Reset Activity board
        2. Status of motor relays, i.e., are they on or off

    This class needs to provide an abstraction for AlobotNode so that is can perform the above.  So, we expose the following
    methods:

        StartActivityBoard
        StopActivityBoard
        ResetActivityBoard
        GetMotorsStatus (this could all be a callback so we are notified when it happens can react)
    '''
    def __init__(self, on_status_change):
        self.on_status_change = on_status_change

        # Get motor relay numbers for use later in _HandleUSBRelayStatus if USB Relay is in use:
        self._relay_exists = rospy.get_param("~usbRelayInstalled", False)
        if self._relay_exists:

            '''
            Create a service proxy for setting the relay state
            '''
            try:
                rospy.wait_for_service('/arlobot_usbrelay/relay_set_state')

                self._relay_set_state = rospy.ServiceProxy('/arlobot_usbrelay/relay_set_state', RelaySetState, self._handle_usb_relay_status)
            except rospy.ServiceException as se:
                raise RelayBankError(se)

            '''
            Create a service proxy for resetting the relay
            '''
            try:
                rospy.wait_for_service('/arlobot_usbrelay/relay_reset_sequence')

                self._relay_reset_sequence = rospy.ServiceProxy('arlobot_usbrelay/relay_reset_sequence', RelayResetSequence, self._handle_usb_relay_status)
            except rospy.ServiceException as se:
                raise RelayBankError(se)

            rospy.Subscriber("arlobot_usbrelay/usbRelayStatus", usbRelayStatus, self._handle_usb_relay_status)
        else:
            rospy.loginfo("Relays are not supported")


    def _handle_usb_relay_status(self, status):
        '''
        Description: receives the updated status of any supported relay when there is a change in state, converts status
                     into a dictionary and passes the information on to the callback.
        :param status: contains two fields: Names and States which are correlated by index
        :return: None
        '''
        if self._relay_exists:
            self.on_status_change(dict(zip(status.Names, status.States)))
        else:
            rospy.loginfo("Relays are supported")


    def StartActivityBoard(self):
        '''
        Description: starts the Activity board by clearing and setting the Activity board relay
        :return: None
        '''
        if self._relay_exists:
            self._reset_sequence("Activity Board", 2.0)
        else:
            rospy.loginfo("Relays are supported")

    def StopActivityBoard(self):
        '''
        Description: stops the Activity board by clearing the Activity board relay
        :return: None
        '''
        if self._relay_exists:
            self._set_relay_state("Activity Board", False)
        else:
            rospy.loginfo("Relays are supported")

    def ResetActivityBoard(self):
        '''
        Description: resets the Activity board by clearing and setting the Activity board relay
        :return: None
        '''
        if self._relay_exists:
            self._reset_sequence("Activity Board", 2.0)
        else:
            rospy.loginfo("Relays are supported")


if __name__ == "__main__":
    def on_status_change(status):
        for key, value in status.iteritems:
            print("key: ", key, ", value: ", value)
    rb = RelayBank(on_status_change)

    state, success = rb.StopActivityBoard()
    print("State: ", state, " Success: ", success)

    state, success = rb.StartActivityBoard()
    print("State: ", state, " Success: ", success)

    state, success = rb.ResetActivityBoard()
    print("State: ", state, " Success: ", success)

