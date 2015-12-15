import math

class SerialMessageError(Exception):
    pass

class SerialMessage():
    """
    This is an alternate approach to handling the serial data from the propeller board.  Rather
    than expose the raw string data, it would be more object-oriented (and a more extensible) approach
    to wrap the serial data in a message class where the class can be responsible for parsing the message
    and just exposing the relevant properties
    """

    NUM_ARGUMENTS = 3

    ACTION_CLASS = 1
    CONFIG_CLASS = 2
    STATUS_CLASS = 3
    DEBUG_CLASS = 4

    UNKNOWN = 0
    ACTION_MOVE_COMMAND = 1
    CONFIG_OP_STATE_COMMAND = 2
    CONFIG_DRIVE_GEOMETRY_COMMAND = 3

    STATUS_ODOMETRY_MESSAGE = 1
    STATUS_OP_STATE_MESSAGE = 2
    STATUS_ULTRASONIC_SENSOR_MESSAGE = 3
    STATUS_ANALOG_IR_SENSOR_MESSAGE = 4
    STATUS_DIGITAL_IR_SENSOR_MESSAGE = 5

    __ACTION_MESSAGE_CLASS = 'a'
    __STATUS_MESSAGE_CLASS = 's'
    __CONFIG_MESSAGE_CLASS = 's'

    __MOVE_MESSAGE_TYPE = 'm'
    __DRIVE_GEOMETRY_MESSAGE_TYPE = 'c'
    __OP_STATE_MESSAGE_TYPE = 'c'

    STATUS_ULTRASONIC_SENSOR_PAYLOAD_LENGTH = 16
    STATUS_ANALOG_IR_SENSOR_PAYLOAD_LENGTH = 16
    STATUS_DIGITAL_IR_SENSOR_PAYLOAD_LENGTH = 6
    STATUS_OP_STATE_PAYLOAD_LENGTH = 15


    def __str__(self):
        return "Class: {}, Type: {}, Payload: {}".format(self.msg_class, self.msg_type, str(self.payload))

    def __init__(self, msg_type=UNKNOWN, data=None):
        """
        This class is dynamic and creates the class attributes based on the message received
        """

        # The message class represents that class of message, i.e., STATUS, CONFIG, DEBUG, ACTION
        self.msg_class = self.UNKNOWN
        # The message type represents the sub-class of message, i.e., ODOMETRY, MOVE, etc.  Note: the same message
        # type can appear under different classes, e.g., Operational State is both a Config and Status message
        self.msg_type = msg_type
        # The payload is a list representation of the data contained in the message.  Design decision is to keep the
        # type as list because it is easy to convert it to a string as necessary, typically for printing.
        self.payload = None

        # We do this part if we are creating a message from parameters, e.g. a list
        # and we need to create a string that can be sent over the serial port
        if type(data) is list:
            self._parse_list_msg(data)

        # We do this part if we are creating a message from a string received over the serial port
        # e.g., the data received is a string that needs to be parsed
        elif type(data) is str or type(data) is unicode: # This can be either 'str' or 'unicode'.  Both are handled basically as string
            self.msg = data.lstrip().rstrip()
            result = self._is_valid(self.msg)
            if result:
                self._parse_str_msg(self.msg)
            else:
                raise SerialMessageError("Failed to validate data: ", data)
        else:
            raise SerialMessageError("Unknown handled for data: ", type(data))

    def __float_convert(self, value):
        new_value = 0.0
        try:
            new_value = float(value)
        except ValueError:
            new_value = 0.0
        if math.isnan(new_value):
            return 0.0
        return new_value

    def _parse_str_msg(self, data):
        try:
            msg_class, msg_type, msg_data = data.split(':')
        except ValueError:
            raise SerialMessageError("Insufficient separators: ", data)

        if msg_class == 's':
            self.msg_class = self.STATUS_CLASS
            
            # Parse the Odometry data
            if msg_type == 'o':
                self.msg_type = self.STATUS_ODOMETRY_MESSAGE

                # Create default attributes in case there is a conversion problem below
                # This should prevent the dreaded attribute error and ensure every
                # attribute has a value
                self.x = 0.0
                self.y = 0.0
                self.theta = 0.0
                self.alternate_theta = 0.0
                self.vx = 0.0
                self.omega = 0.0

                self.payload = msg_data.split(',')
                if len(self.payload) in range(5,7):
                    offset = 0
                    self.x = self.__float_convert(self.payload[offset])
                    offset += 1
                    self.y = self.__float_convert(self.payload[offset])
                    offset += 1
                    self.theta = self.__float_convert(self.payload[offset])  # On ArloBot odometry derived heading works best.
                    offset += 1
                    if len(self.payload) < 5:
                        self.alternate_theta = self.__float_convert(self.payload[offset])
                        offset += 1

                    self.vx = self.__float_convert(self.payload[offset])
                    offset += 1
                    self.omega = self.__float_convert(self.payload[offset])
                else:
                    raise SerialMessageError("Wrong number of parameters for Odometry message")

            # Parse the Operational State data
            elif msg_type == 'p':
                self.msg_type = self.STATUS_OP_STATE_MESSAGE

                self.drive_geometry_received = 0
                self.op_state_received = 0
                self.motion_detected = 0
                self.safe_to_proceed = 0
                self.safe_to_recede = 0
                self.escaping = 0
                self.max_forward_speed = 0
                self.max_reverse_speed = 0
                self.min_distance_sensor = 0
                self.left_motor_voltage = 0.0
                self.right_motor_voltage = 0.0
                self.cliff_detected = 0
                self.floor_obstacle_detected = 0

                self.payload = msg_data.split(',')
                if len(self.payload) == self.STATUS_OP_STATE_PAYLOAD_LENGTH:
                    offset = 0
                    self.drive_geometry_received = int(self.payload[offset])
                    offset += 1
                    self.op_state_received = int(self.payload[offset])
                    offset += 1
                    self.motion_detected = int(self.payload[offset])
                    offset += 1
                    self.safe_to_proceed = int(self.payload[offset])
                    offset += 1
                    self.safe_to_recede = int(self.payload[offset])
                    offset += 1
                    self.escaping = int(self.payload[offset])
                    offset += 1
                    self.max_forward_speed = int(self.payload[offset])
                    offset += 1
                    self.max_reverse_speed = int(self.payload[offset])
                    offset += 1
                    self.min_distance_sensor = self.__float_convert(self.payload[offset])
                    offset += 1
                    self.left_motor_voltage = self.__float_convert(self.payload[offset])
                    offset += 1
                    self.right_motor_voltage = self.__float_convert(self.payload[offset])
                    offset += 1
                    self.left_motor_current = self.__float_convert(self.payload[offset])
                    offset += 1
                    self.right_motor_current = self.__float_convert(self.payload[offset])
                    offset += 1
                    self.cliff_detected = int(self.payload[offset])
                    offset += 1
                    self.floor_obstacle_detected = int(self.payload[offset])
                    offset += 1
                else:
                    raise SerialMessageError("Wrong number of parameters for Op State message")

            # Parse the Infrared Sensor data
            elif msg_type == 'i':
                self.msg_type = self.STATUS_ANALOG_IR_SENSOR_MESSAGE

                self.payload = msg_data.split(',')
                if len(self.payload) == self.STATUS_ANALOG_IR_SENSOR_PAYLOAD_LENGTH:
                    pass
                else:
                    raise SerialMessageError("Wrong number of parameters for Analog IR message")
                pass
            
            # Parse the Ultrasonic Sensor data
            elif msg_type == 'u':
                self.msg_type = self.STATUS_ULTRASONIC_SENSOR_MESSAGE
                
                self.payload = msg_data.split(',')
                if len(self.payload) == self.STATUS_ULTRASONIC_SENSOR_PAYLOAD_LENGTH:
                    # Nothing else to do, unless there is a need to create other representations
                    # of the sensors values, e.g., a dictionary
                    pass
                else:
                    raise SerialMessageError("Wrong number of parameters for Ultrasonic message")
                pass

            elif msg_type == 'g':
                self.msg_type = self.STATUS_DIGITAL_IR_SENSOR_MESSAGE

                self.payload = msg_data.split(',')
                if len(self.payload) == self.STATUS_DIGITAL_IR_SENSOR_PAYLOAD_LENGTH:
                    pass
                else:
                    raise SerialMessageError("Wrong number of parameters for Analog IR message")
                pass


    def _parse_list_msg(self, data):
        if self.msg_type == self.CONFIG_DRIVE_GEOMETRY_COMMAND:
            self.msg_class = self.CONFIG_CLASS
            # There should be a validation of the number of elements in data for this message
            self.msg = 'c:d:'+','.join(map(str, data))+'\r'

        elif self.msg_type == self.CONFIG_OP_STATE_COMMAND:
            self.msg_class = self.CONFIG_CLASS
            # There should be a validation of the number of elements in data for this message
            self.msg = 'c:p:'+','.join(map(str, data))+'\r'

        elif self.msg_type == self.ACTION_MOVE_COMMAND:
            self.msg_class = self.ACTION_CLASS
            # There should be a validation of the number of elements in data for this message
            self.msg = 'a:m:'+','.join(map(str, data))+'\r'

    def _is_valid(self, data):
        # Add a regular expression to test the format <group1>:<group2>:<group3>
        # Matches string that
        #     starts with a newline followed by
        #     one or more letters a-z followed by
        #     a colon followed by
        #     one or more letters a-z followed by
        #     a colon followed by
        #     either an integer or floating point number followed by
        #     zero or more comma separated integers or floating point numbers
        import re
        match = re.match(r'^([a-z]):([a-z]):(([-+]?\d+(\.\d+)*))(,([-+]?\d+(\.\d+)*))*$', data)
        return match != None


