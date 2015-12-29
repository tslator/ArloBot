
import time
import rospy
from serial_data_gateway import SerialDataGateway
from std_msgs.msg import String
from serial_msg import SerialMessage, SerialMessageError


class PropellerCommsError(Exception):
    pass

class PropellerComms(object):
    def __init__(self, message_callback):
        self._serial_available = False
        self._serial_timeout = 0
        self._counter = 0
        self._message_callback = message_callback

        port = rospy.get_param("~port", "/dev/ttyUSB0")
        baud_rate = int(rospy.get_param("~baudRate", 115200))

        rospy.loginfo("PropellerComms: configured with serial port: {}, baud rate: {}".format(port, str(baud_rate)))

        self._serial_gateway = SerialDataGateway(port, baud_rate, self._handle_received_line)
        self._serial_publisher = rospy.Publisher('serial', String, queue_size=10)
        pass

    def _handle_received_line(self, line):
        self._counter += 1
        self._serial_timeout = 0

        #rospy.logwarn("Incoming: " + line)

        msg = None
        try:
            msg = SerialMessage(data = line)
        except SerialMessageError as sme:
            rospy.logwarn(sme)
            rospy.logwarn("Invalid line: " + line)
            return

        # Publish the message to the serial topic
        self._serial_publisher.publish(String("{} , in:  {}".format(self._counter, line)))
        # Deliver the message to Arlobot for processing
        self._message_callback(msg)

    def Timeout(self):
        if self._serial_available:
            self._serial_timeout += 1
        else:
            self._serial_timeout = 0
        if self._serial_timeout > 19:
            rospy.loginfo("PropellerComms: serial timeout {}".format(self._serial_timeout))
            return True

    def Shutdown(self):
        self._serial_available = False
        rospy.loginfo("PropellerComms.Shutdown: stopping serial gateway ...")
        try:
            self._serial_gateway.Stop()
        except AttributeError:
            rospy.loginfo("PropellerComms.Shutdown: failed attempt to stop serial gateway.")
            raise PropellerCommsError("serial gateway error during shutdown")
        rospy.loginfo("PropellerComms.Shutdown: serial gateway stopped.")

    def Startup(self):
        rospy.loginfo("PropellerComms.Startup: starting serial gateway ...")
        try:
            self._serial_gateway.Start()
        except:
            rospy.loginfo("PropellerComms.Startup: error starting serial gateway.")
            raise PropellerCommsError("Unable to start Propeller communication")
        rospy.loginfo("PropellerComms.Startup: serial gateway started.")
        self._serial_available = True

    def Reset(self, settle_time=5):
        # Probably this is not needed because both the PC and the Propeller board are on the same power distribution
        # But, it doesn hurt anything to have it.
        rospy.loginfo("PropellerComms.Reset: resetting serial gateway ...")
        self.Shutdown()
        rospy.loginfo("PropellerComms.Reset: pausing {} seconds before restart ...".format(settle_time))
        time.sleep(settle_time)
        self.Startup()
        rospy.loginfo("PropellerComms.Reset: serial gateway reset.")

    def Running(self):
        return self._serial_available

    def SendMessage(self, msg):
        #rospy.logwarn("Outgoing: " + msg.msg)
        self._serial_publisher.publish(String("{} , out: {}".format(self._counter, msg.msg)))
        self._serial_gateway.Write(msg.msg)
