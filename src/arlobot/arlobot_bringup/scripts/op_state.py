__author__ = 'hal'


class OperationalStateError(Exception):
    pass

class OperationalState:
    OP_STATE_SHUTDOWN = -2
    OP_STATE_FAULT = -1
    OP_STATE_START = 0
    OP_STATE_INITIALIZED = 1
    OP_STATE_DRIVE_GEO_RECEIVED = 2
    OP_STATE_OP_STATE_RECEIVED = 3
    OP_STATE_CONFIGURED = 4
    OP_STATE_OPERATIONAL = 5

    def __init__(self):
        self._state = self.OP_STATE_START

    def Update(self, **kwevents):
        '''
        Description - make this a state machine that tracks the robot operational state

        As each event arrives the state will change

        :param kwevents:
        :return:
        '''

        import rospy

        #rospy.logwarn("State (in): " + str(self._state))

        if (kwevents.get("initialize")):
            #rospy.logwarn("Event: initialize")
            if self._state == self.OP_STATE_START:
                self._state = self.OP_STATE_INITIALIZED
            else:
                # Houston we have a problem
                self._state = self.OP_STATE_FAULT
                raise OperationalStateError("FATAL: invalid state for initialized event.")
            #rospy.logwarn("State (out): " + str(self._state))

        elif (kwevents.get("drive_geometry_received")):
            #rospy.logwarn("Event: drive_geometry_received")
            if self._state == self.OP_STATE_INITIALIZED:
                self._state = self.OP_STATE_DRIVE_GEO_RECEIVED
            elif self._state == self.OP_STATE_OP_STATE_RECEIVED:
                self._state = self.OP_STATE_CONFIGURED
            elif self._state == self.OP_STATE_DRIVE_GEO_RECEIVED or self._state == self.OP_STATE_OPERATIONAL:
                pass
            else:
                self._state = self.OP_STATE_FAULT
                raise OperationalStateError("FATAL: invalid state for drive_geometry_received event.")
            #rospy.logwarn("State (out): " + str(self._state))

        elif (kwevents.get("op_state_received")):
            #rospy.logwarn("Event: op_state_received")
            if self._state == self.OP_STATE_INITIALIZED:
                self._state = self.OP_STATE_OP_STATE_RECEIVED
            elif self._state == self.OP_STATE_DRIVE_GEO_RECEIVED:
                self._state = self.OP_STATE_CONFIGURED
            elif self._state == self.OP_STATE_OP_STATE_RECEIVED or self._state == self.OP_STATE_OPERATIONAL:
                pass
            else:
                self._state = self.OP_STATE_FAULT
                raise OperationalStateError("FATAL: invalid state for op_state_received event.")
            #rospy.logwarn("State (out): " + str(self._state))

        elif (kwevents.get("running")):
            if self._state == self.OP_STATE_CONFIGURED:
                self._state = self.OP_STATE_OPERATIONAL
            else:
                self._state = self.OP_STATE_FAULT
                raise OperationalStateError("FATAL: invalid state for running event")
            #rospy.logwarn("State (out): " + str(self._state))

        elif (kwevents.get("shutdown")):
            self._state = self.OP_STATE_SHUTDOWN
            rospy.logwarn("State (out): " + str(self._state))

        else:
            last_state = self._state
            self._state = self.OP_STATE_FAULT
            raise OperationalStateError("ERROR: Mongo not know! {}".format(last_state))

    def IsInitialized(self):
        return self._state == self.OP_STATE_INITIALIZED

    def IsConfigured(self):
        return self._state == self.OP_STATE_CONFIGURED

    def IsOperational(self):
        return self._state == self.OP_STATE_OPERATIONAL
