__author__ = 'Tim'

import time
from SerialDataGateway import SerialDataGateway
from SerialMessage import SerialMessage, SerialMessageError

import sys
if sys.version[0]=="2": input=raw_input

done = False

op_state = ""
odometry = ""

#c:d:0.403,0.00676
#c:p:0,0,0,0,0,0.00,0.00,0.00

drive_geometry = SerialMessage(SerialMessage.CONFIG_DRIVE_GEOMETRY_COMMAND, [0.403,0.00676])
operational_state = SerialMessage(SerialMessage.CONFIG_OP_STATE_COMMAND, [0,0,0,0,0,0.00,0.00,0.00])

dataReceiver = None
configured = False

def ShowOpState():
    print(op_state)

def ShowOdometry():
    print(odometry)

def DoMove(vel):
    lin_vel, ang_vel = vel.split(',')
    move = SerialMessage(SerialMessage.ACTION_MOVE_COMMAND, [float(lin_vel), float(ang_vel)])
    dataReceiver.Write(move.data)

def Test(num):
    if num == 1:
        print("Move forward, Move backward")
        print("Moving forward ...")
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
    elif num == 2:
        print("Rotate Left, Rotate Right")
        print("Rotating left ...")
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

def _handle_raw_message(line):
    print(line)

def _handle_received_line(line):
    global odometry
    global op_state
    global dataReceiver
    global configured

    try:
        msg = SerialMessage(data=line)
    except SerialMessageError:
        #print(line)
        return

    if msg.msg_class == SerialMessage.STATUS_CLASS:
        if msg.msg_type == SerialMessage.STATUS_OP_STATE_MESSAGE:
            op_state = line

            if not msg.drive_geometry_received:
                print("Sending drive geometry")
                dataReceiver.Write(drive_geometry.data)

            if not msg.op_state_received:
                print("Sending operational state")
                dataReceiver.Write(operational_state.data)

            if not configured:
                if msg.drive_geometry_received:
                    print("Drive Geometry received")

                if msg.op_state_received:
                    print("Op State received")

            if msg.drive_geometry_received and msg.op_state_received:
                configured = True


        elif msg.msg_type == SerialMessage.STATUS_ODOMETRY_MESSAGE:
            odometry = line
            #print(odometry)
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

    #dataReceiver = SerialDataGateway("COM3", 115200, _handle_received_line)
    dataReceiver = SerialDataGateway("/dev/ttyUSB2", 115200, _handle_received_line)
    dataReceiver.Start()

    DoMove("0.0,0.0")

    while not done:

        while not configured:
            time.sleep(1)


        print("The following commands are supported:")
        print("    Show Op State - s")
        print("    Show Odometry - o")
        print("    Move - m:<linear velocity>,<angular velocity>")
        print("    Test - t:<number>")
        print("    Exit - x")
        result = input("Enter a command: ")
        if result == 's':
            ShowOpState()
        elif result == 'o':
            ShowOdometry()
        elif 'm' in result:
            cmd, vel = result.split(':')
            DoMove(vel)
        elif 't' in result:
            cmd, num = result.split(':')
            Test(int(num))
        elif result == 'x':
            DoMove("0.0,0.0")
            done = True

    dataReceiver.Stop()


if __name__ == "__main__":
    main()
