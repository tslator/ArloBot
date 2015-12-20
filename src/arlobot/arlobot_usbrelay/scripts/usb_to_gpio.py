from __future__ import print_function

import Adafruit_GPIO as GPIO
import Adafruit_GPIO.FT232H as FT232H


class UsbToGpioError(Exception):
    pass


class UsbToGpio:
    '''
    This class wraps the Adafuit GPIO and FT232H libraries in a Borg-style implementation so that other services can
    access the USB-to-GPIO FT232H hardware module.  The USB-to-GPIO FT232H hardware module is connected to the PC and
    provides access to 12 GPIO pins and other serial protocols (I2C, SPI, RS-485, etc).  At this point, the intent is
    to use the hardware module to control the Activity board power via a relay, monitor the relay status, and also
    monitor the relay status of the left and right motor relays.  Note: those relays are controlled from the Activity
    board.
    '''

    """
    Implements the Borg pattern which provides shared state among multiple objects
    """
    __shared_state = {}


    '''
    The FT232H supports 12 pins of GPIO: D4 to D7 and C0 to C7
    Pins numbers are mapped as follows:
        0 - 7 -> D0 - D7
        8 - 15 -> C0 - C7
    '''

    PIN_D0 = 0
    PIN_D1 = 1
    PIN_D2 = 2
    PIN_D3 = 3
    PIN_D4 = 4
    PIN_D5 = 5
    PIN_D6 = 6
    PIN_D7 = 7

    PIN_C0 = 8
    PIN_C1 = 9
    PIN_C2 = 10
    PIN_C3 = 11
    PIN_C4 = 12
    PIN_C5 = 13
    PIN_C6 = 14
    PIN_C7 = 15

    BOOL_TO_GPIO_MAP = {True : GPIO.HIGH, False: GPIO.LOW}
    GPIO_TO_BOOL_MAP = {GPIO.HIGH : True, GPIO.LOW : False}

    def __init__(self):
        """
        Implements the Borg pattern which provides shared state among multiple objects
        """

        self.__dict__ = self.__shared_state

        try:
            # Temporarily disable the built-in FTDI serial driver on Mac & Linux platforms.
            FT232H.use_FT232H()
        except RuntimeError as rte:
            raise UsbToGpioError(rte)

        try:
            # Create an FT232H object that grabs the first available FT232H device found.
            self._ft232h = FT232H.FT232H()
        except RuntimeError as rte:
            raise UsbToGpioError(rte)


    def SetOutput(self, pin, value):
        try:
            self._ft232h.setup(pin, GPIO.OUTPUT)
            self._ft232h.output(pin, self.BOOL_TO_LOGIC_MAP[value])
        except RuntimeError as rte:
            raise UsbToGpioError(rte)

    def GetInput(self, pin):
        result = False
        try:
            self._ft232h.setup(pin, GPIO.INPUT)
            result = self.GPIO_TO_BOOL_MAP(self._ft232h.input(pin))
        except RuntimeError as rte:
            raise UsbToGpioError(rte)

        return result


if __name__ == "__main__":
    u2g = UsbToGpio()

    try:
        u2g.SetOutput(0, GPIO.HIGH)
        value = us2.GetIntput(0)
        assert(value == GPIO.HIGH)

        u2g.SetOutput(0, GPIO.LOW)
        value = us2.GetIntput(0)
        assert(value == GPIO.LOW)

    except UsbToGpioError as utge:
        print(utge)

