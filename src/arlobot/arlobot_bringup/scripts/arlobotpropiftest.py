from __future__ import print_function
__author__ = 'Tim'

import time
from SerialDataGateway import SerialDataGateway
from SerialMessage import SerialMessage, SerialMessageError

from sys import platform as _platform


if _platform == "linux" or _platform == "linux2":
    PORT = "/dev/ttyUSB0"
elif _platform == "darwin":
    # MAC OS X
    pass
elif _platform == "win32":
    PORT = "COM7"

try:
    input = raw_input
except:
    pass

done = False

op_state = ""
odometry = ""
analog_ir_sensor = ""
digital_ir_sensor = ""
ultrasonic_sensor = ""

#c:d:0.403,0.00676
#c:p:0,0,0,0,0,0.00,0.00,0.00

drive_geometry = SerialMessage(SerialMessage.CONFIG_DRIVE_GEOMETRY_COMMAND, [0.403,0.00676])
operational_state = SerialMessage(SerialMessage.CONFIG_OP_STATE_COMMAND, [0,0,0,0,0,0.00,0.00,0.00])

dataReceiver = None
configured = False
safety_enable = True
enable_op_state_print = False
enable_odometry_print = False
enable_analog_ir_sensor_print = False
enable_digital_ir_sensor_print = False
enable_ultrasonic_sensor_print = False

def ShowOpState():
    global enable_op_state_print
    if enable_op_state_print:
        print(op_state, end='\r')

def ShowOdometry():
    global enable_odometry_print
    if enable_odometry_print:
        print(odometry, end='\r')

def ShowAnalogIRSensors():
    global enable_analog_ir_sensor_print
    if enable_analog_ir_sensor_print:
        print(analog_ir_sensor, end='\r')

def ShowDigitalIRSensors():
    global enable_digital_ir_sensor_print
    if enable_digital_ir_sensor_print:
        print(digital_ir_sensor, end='\r')

def ShowUltrasonicSensors():
    global enable_ultrasonic_sensor_print
    if enable_ultrasonic_sensor_print:
        print(ultrasonic_sensor, end='\r')

def ToggleSafety():
    global safety_enable
    ignore_proximity = False
    ignore_cliff_sensors = False
    ignore_dist_sensors = False
    ignore_floor_sensors = False
    ac_power = False

    parsed_odom = odometry.split(':')[2].split(',')
    # clear ignore_proximity in op_state message to enable safety
    ignore_proximity = False
    if (safety_enable):
        safety_enable = False
        ignore_proximity = True
        ignore_proximity = True
        ignore_cliff_sensors = True
        ignore_dist_sensors = True
        ignore_floor_sensors = True
        ac_power = True
        print("Safety is disabled")
    else:
        # set ignore_proximity in op_state message to disable safety
        ignore_proximity = False
        ignore_cliff_sensors = False
        ignore_dist_sensors = False
        ignore_floor_sensors = False
        ac_power = False
        safety_enable = True
        print("Safety is enabled")

    op_state = SerialMessage(SerialMessage.CONFIG_OP_STATE_COMMAND,
                             [int(ignore_proximity),
                              int(ignore_cliff_sensors),
                              int(ignore_dist_sensors),
                              int(ignore_floor_sensors),
                              int(ac_power),
                              parsed_odom[0], parsed_odom[1], parsed_odom[2]])
    print(op_state.msg)
    dataReceiver.Write(op_state.msg)

def DisableSafety():
    global safety_enable
    # Set ignore_proximity in op_state message
    parsed_odom = odometry.split(':')[2].split(',')
    op_state = SerialMessage(SerialMessage.CONFIG_OP_STATE_COMMAND, [1, 0, 0, 0, 0, parsed_odom[0], parsed_odom[1], parsed_odom[2]])
    #print(op_state.msg)
    dataReceiver.Write(op_state.msg)
    safety_enable = False
    PrintSafetyState()

def DoMove(vel):
    lin_vel, ang_vel = vel.split(',')
    move = SerialMessage(SerialMessage.ACTION_MOVE_COMMAND, [float(lin_vel), float(ang_vel)])
    print(move.msg)
    dataReceiver.Write(move.msg)

def Test(num):
    global enable_odometry_print
    global enable_ultrasonic_sensor_print

    print("\n")

    if num == 1:
        print("Move forward, Move backward")
        print("Moving forward ...")
        enable_odometry_print = True
        DoMove("0.3,0.0")
        time.sleep(1.0)
        DoMove("0.3,0.0")
        time.sleep(1.0)
        DoMove("0.0,0.0")
        print("Moving backward ...")
        DoMove("-0.3,0.0")
        time.sleep(1.0)
        DoMove("-0.3,0.0")
        time.sleep(1.0)
        enable_odometry_print = False
    elif num == 2:
        print("Rotate Left, Rotate Right")
        print("Rotating left ...")
        enable_odometry_print = True
        DoMove("0.0,1.0")
        time.sleep(1.0)
        DoMove("0.0,1.0")
        time.sleep(1.0)
        DoMove("0.0,0.0")
        print("Rotating Right ...")
        DoMove("0.0,-1.0")
        time.sleep(1.0)
        DoMove("0.0,-1.0")
        time.sleep(1.0)
        enable_odometry_print = False

last_time = int(round(time.time() * 1000))
delta = 0;

def _handle_received_line(line):
    global odometry
    global op_state
    global analog_ir_sensor
    global digital_ir_sensor
    global ultrasonic_sensor
    global dataReceiver
    global configured
    global last_time
    global delta

    delta = int(round(time.time() * 1000)) - last_time
    last_time = int(round(time.time() * 1000))

    #print(delta, ": ", line)

    try:
        msg = SerialMessage(data=line)
    except SerialMessageError as sme:
        print(sme)
        print(line)
        return

    if msg.msg_class == SerialMessage.STATUS_CLASS:
        #print("Class: ", msg.msg_class, " Type: ", msg.msg_type, " Msg: ", msg.msg)
        if msg.msg_type == SerialMessage.STATUS_OP_STATE_MESSAGE:
            op_state = line
            ShowOpState()

            if not msg.drive_geometry_received:
                print("Sending drive geometry")
                dataReceiver.Write(drive_geometry.msg)

            if not msg.op_state_received:
                print("Sending operational state")
                dataReceiver.Write(operational_state.msg)

            if not configured:
                if msg.drive_geometry_received:
                    print("Drive Geometry received")

                if msg.op_state_received:
                    print("Op State received")

            if msg.drive_geometry_received and msg.op_state_received:
                configured = True


        elif msg.msg_type == SerialMessage.STATUS_ODOMETRY_MESSAGE:
            odometry = line
            ShowOdometry()
        elif msg.msg_type == SerialMessage.STATUS_ANALOG_IR_SENSOR_MESSAGE:
            analog_ir_sensor = line
            ShowAnalogIRSensors()
        elif msg.msg_type == SerialMessage.STATUS_DIGITAL_IR_SENSOR_MESSAGE:
            digital_ir_sensor = line
            ShowDigitalIRSensors()
        elif msg.msg_type == SerialMessage.STATUS_ULTRASONIC_SENSOR_MESSAGE:
            ultrasonic_sensor = line
            ShowUltrasonicSensors()
        else:
            print("Unknown type: ", line)

    elif msg.msg_class == SerialMessage.DEBUG_CLASS:
        #print("DEBUG: {}: {}".format(msg_type, data))
        print("Debug message")
        pass
    else:
        print("Unknown class: ", line)


def main():
    global done
    global dataReceiver
    global configured
    global odometry
    global op_state
    global analog_ir_sensor
    global digital_ir_sensor
    global ultrasonic_sensor
    global enable_op_state_print
    global enable_odometry_print
    global enable_analog_ir_sensor_print
    global enable_digital_ir_sensor_print
    global enable_ultrasonic_sensor_print

    dataReceiver = SerialDataGateway(PORT, 115200, _handle_received_line)
    dataReceiver.Start()

    DoMove("0.0,0.0")

    while not done:

        while not configured:
            time.sleep(1)


        print("The following commands are supported:")
        print("    Show Op State - s")
        print("    Show Odometry - o")
        print("    Show Analog IR - i")
        print("    Show Digital IR - g")
        print("    Show Ultrasonic - u")
        print("    Toggle Safety - y")
        print("    Move - m:<linear velocity>,<angular velocity>")
        print("    Test - t:<number>")
        print("    Exit - x")
        result = input("Enter a command: ")
        if result == 's':
            enable_op_state_print = True
        elif result == 'o':
            enable_odometry_print = True
        elif result == 'i':
            enable_analog_ir_sensor_print = True
        elif result == 'g':
            enable_digital_ir_sensor_print = True
        elif result == 'u':
            enable_ultrasonic_sensor_print = True
        elif result == 'y':
            ToggleSafety()
            print(op_state)
        elif 'm' in result:
            cmd, vel = result.split(':')
            enable_odometry_print = True
            DoMove(vel)
            enable_odometry_print = False
        elif 't' in result:
            cmd, num = result.split(':')
            Test(int(num))
        elif result == 'x':
            DoMove("0.0,0.0")
            done = True
        elif result == 'q':
            enable_odometry_print = False
            enable_analog_ir_sensor_print = False
            enable_digital_ir_sensor_print = False
            enable_ultrasonic_sensor_print = False


    dataReceiver.Stop()


if __name__ == "__main__":
    main()
