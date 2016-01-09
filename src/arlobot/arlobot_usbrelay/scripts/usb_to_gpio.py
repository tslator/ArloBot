from __future__ import print_function

import Adafruit_GPIO as GPIO
import Adafruit_GPIO.FT232H as FT232H
import time


class ADS7828(FT232H.I2CDevice):
    __CHANNEL = [0x84,0xC4,0x94,0xD4,0xA4,0xE4,0xB4,0xF4]
    __WRITE_MODIFIER = 0x00
    __READ_MODIFIER = 0x01

    def __init__(self, ft232h, addr):
        FT232H.I2CDevice.__init__(self, ft232h)
        self._addr = addr

    def ReadRaw(self, channel):
        '''
        :param channel:
        :return:

        ack = i2c_startpoll(&i2c_bus, write_addr);
        ack = i2c_writeByte(&i2c_bus, CHANNEL[channel]);
        ack = i2c_startpoll(&i2c_bus, read_addr);
        a2d_val_high = i2c_readByte(&i2c_bus, 0);
        a2d_val_low = i2c_readByte(&i2c_bus, 1);
        i2c_stop(&i2c_bus);
        return ((a2d_val_high & 0xffff) << 8) | (a2d_val_low & 0xffff)

        '''

        self._idle()
        self._transaction_start()
        self._start()
        self._i2c_write_bytes([self._addr + self.__WRITE_MODIFIER, self.__CHANNEL[channel]])
        self._start()
        self._i2c_write_bytes([self._addr + self.__READ_MODIFIER])
        self._i2c_read_bytes(2)
        self._i2c_stop()
        response = self._transaction_end()
        self._verify_acks(response[:-2])
        return (response[-2] << 8) | response[-1]

    def ReadVoltage(self, channel):
        return self.ReadRaw(channel)


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

    BOOL_TO_GPIO_MAP = {True : FT232H.HIGH, False: FT232H.LOW}
    GPIO_TO_BOOL_MAP = {FT232H.HIGH : True, FT232H.LOW : False}

    LOW_HIGH_LOW = 0
    HIGH_LOW_HIGH = 1

    __ADS7828_ADDR_0 = 0x90
    __ADS7828_ADDR_1 = 0x91

    class ADCDevices:
        __ADC_SELECT = {}
        __MASK = 8
        __num_devices = 0
        def AddDevice(self, device):
            self.__ADC_SELECT[self.__num_devices] = device
            self.__num_devices += 1

        def ReadVoltage(self, input):
            return self.__ADC_SELECT[bool(input & self.__MASK)].ReadVoltage(input)


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

        # Create the I2C ADC devices
        try:
            # Create an I2C device at address ADS7828_ADDR_0
            self.ADCDevices.AddDevice(ADS7828(self._ft232h, self.__ADS7828_ADDR_0))

        except RuntimeError as rte:
            raise UsbToGpioError(rte)

        try:
            # Create an I2C device at address ADS7828_ADDR_1
            self.ADCDevices.AddDevice(ADS7828(self._ft232h, self.__ADS7828_ADDR_1))

        except RuntimeError as rte:
            raise UsbToGpioError(rte)

    def SetOutput(self, pin, value):
        try:
            self._ft232h.setup(pin, FT232H.OUT)
            self._ft232h.output(pin, self.BOOL_TO_GPIO_MAP[value])
        except RuntimeError as rte:
            raise UsbToGpioError(rte)

    def GetInput(self, pin):
        result = False
        try:
            self._ft232h.setup(pin, FT232H.IN)
            result = self.GPIO_TO_BOOL_MAP[self._ft232h.input(pin)]
        except RuntimeError as rte:
            raise UsbToGpioError(rte)

        return result

    def PulseOutput(self, pin, transition = LOW_HIGH_LOW, duration = -1):
        '''
        :param pin: the number of the pin on which the pulse will occur
        :param transition: LOW_TO_HIGH, HIGH_TO_LOW
        :param duration: the time the pulse will state HIGH (or LOW)
        :return: None
        '''
        if duration < 0:
            return

        self._ft232h.setup(pin, FT232H.OUT)
        if transition is self.LOW_HIGH_LOW:
            self._ft232h.output(pin, FT232H.LOW)
            self._ft232h.output(pin, FT232H.HIGH)
            time.sleep(duration)
            self._ft232h.output(pin, FT232H.LOW)
        else:
            self._ft232h.output(pin, FT232H.HIGH)
            self._ft232h.output(pin, FT232H.LOW)
            time.sleep(duration)
            self._ft232h.output(pin, FT232H.HIGH)

    def PulseInput(self, pin, initial_state = FT232H.LOW, transition = LOW_HIGH, timeout = 1):
        '''
        :param pin: the number of pin on which the pulse will be read
        :return: the number of microseconds the pulse is HIGH (or LOW)
        '''
        pulse_end = 0
        pulse_start = 0
        timeout_start = time.time()
        self._ft232h.output(pin, initial_state)
        self._ft232h.setup(pin, FT232H.IN)
        while self._ft232h.input(pin) == initial_state and (time.time() - timeout_start < timeout):
            pulse_start = time.time()

        while self._ft232h.input(pin) != initial_state and (time.time() - timeout_start < timeout):
            pulse_end = time.time()

        return pulse_end - pulse_start

    def SetOutputs(self, pin_start, pin_end, pattern):
        '''
        :param pin_start: first pin in series (LSB)
        :param pin_end: last pin in series (MSB)
        :param patter: value to place on pins
        :return: None
        '''
        pins = {}
        if pin_start < pin_end:
            for ii in range(pin_start, pin_end + 1):
                pins[ii] = bool(pattern & 0x1)
                pattern = pattern >> 1

            self._ft232h.output_pins(pins)

    def ReadAdc(self, input):
        return self.ADCDevices.ReadVoltage(input)


if __name__ == "__main__":
    u2g = UsbToGpio()

    try:
        u2g.SetOutput(UsbToGpio.PIN_D0, GPIO.HIGH)
        value = u2g.GetInput(UsbToGpio.PIN_D1)
        print("Value: ", value)
        assert(value == True)

        u2g.SetOutput(UsbToGpio.PIN_D0, GPIO.LOW)
        value = u2g.GetInput(UsbToGpio.PIN_D1)
        print("value: ", value)
        assert(value == False)

    except UsbToGpioError as utge:
        print(utge)

