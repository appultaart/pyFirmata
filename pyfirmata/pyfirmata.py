import inspect
import time
import sys

import serial

from .util import two_byte_iter_to_str, to_two_bytes, from_two_bytes
from .boards import BOARDS

# Message command bytes (128-255/0x80-0xFF) - straight from Firmata.h
DIGITAL_MESSAGE = 0x90      # send data for a digital pin
ANALOG_MESSAGE = 0xE0       # send data for an analog pin (or PWM)
REPORT_ANALOG = 0xC0        # enable analog input by pin #
REPORT_DIGITAL = 0xD0       # enable digital input by port pair
SET_PIN_MODE = 0xF4         # set a pin to INPUT/OUTPUT/PWM/etc
REPORT_VERSION = 0xF9       # report firmware version
SYSTEM_RESET = 0xFF         # reset from MIDI
START_SYSEX = 0xF0          # start a MIDI SysEx msg
END_SYSEX = 0xF7            # end a MIDI SysEx msg
DIGITAL_PULSE = 0x91        # SysEx command to send a digital pulse
# PULSE_MESSAGE = 0xA0      # proposed pulseIn/Out msg (SysEx)
# SHIFTOUT_MESSAGE = 0xB0   # proposed shiftOut msg (SysEx)


# extended command set using sysex (0-127/0x00-0x7F)
# 0x00-0x0F reserved for user-defined commands */
SERVO_CONFIG = 0x70         # set max angle, minPulse, maxPulse, freq
STRING_DATA = 0x71          # a string message with 14-bits per char
SHIFT_DATA = 0x75           # a bitstream to/from a shift register
I2C_REQUEST = 0x76          # send an I2C read/write request
I2C_REPLY = 0x77            # a reply to an I2C read request
I2C_CONFIG = 0x78           # config I2C settings such as delay times and power pins
CAPABILITY_QUERY = 0x6B     # ask for supported modes and resolution of all pins
CAPABILITY_RESPONSE = 0x6C  # reply with supported modes and resolution
ANALOG_MAPPING_QUERY = 0x69 # ask for mapping of analog to pin numbers
ANALOG_MAPPING_RESPONSE = 0x6A # reply with mapping info
PIN_STATE_QUERY = 0x6D      # ask for a pin's current mode and value
PIN_STATE_RESPONSE = 0x6E   # reply with pin's current mode and value
REPORT_FIRMWARE = 0x79      # report name and version of the firmware
SAMPLING_INTERVAL = 0x7A    # set the poll rate of the main loop
SYSEX_NON_REALTIME = 0x7E   # MIDI Reserved for non-realtime messages
SYSEX_REALTIME = 0x7F       # MIDI Reserved for realtime messages

# Pin modes.
# except from UNAVAILABLE taken from Firmata.h
UNAVAILABLE = -1 
INPUT = 0          # as defined in wiring.h
OUTPUT = 1         # as defined in wiring.h
ANALOG = 2         # analog pin in analogInput mode
PWM = 3            # digital pin in PWM output mode
SERVO = 4          # digital pin in SERVO output mode
SHIFT = 5          # shiftIn/shiftOut mode
I2C = 6            # pin included in I2C setup

# Pin types
DIGITAL = OUTPUT   # same as OUTPUT below
# ANALOG is already defined above


class PinAlreadyTakenError(Exception):
    pass


class InvalidPinDefError(Exception):
    pass
    

class NoInputWarning(RuntimeWarning):
    pass
    

class Board(object):
    """
    Base class for any board
    
    A 'board' instance relates to a physical board onto which the Firmata
    protocol is loaded. Such a 'board' instance:
     - can communicate through serial communication with the physical board
     - can receive and send Firmata-protocol messages to/from the board
     - knows about the Pins and Ports that it contains in the physical world
     - has, or can obtain, version number (etc) and debug information
    """
    firmata_version = None
    firmware = None
    firmware_version = None
    _command_handlers = {}
    pins = {}           # stores information on all Pins (including Rx, Tx, analog)
    ports = {}          # stores Port information
    analog_pins = {}    # maps analog pin number to 'generic' Pin instance
    messages_received = []
    
    def __init__(self, port, layout="autodetect", baudrate=57600, name=None):
        self.sp = serial.Serial(port, baudrate)
        """Initialization of a 'board' instance
        
        A board can be initialized in two ways: by autodetect feature (which
        is set to be the default), or by giving a predefined layout.
        """
        # Allow 5 secs for Arduino's auto-reset to happen
        # Alas, Firmata blinks it's version before printing it to serial
        # For 2.3, even 5 seconds might not be enough.
        # TODO Find a more reliable way to wait until the board is ready
        self.pass_time(6)       # DV: 6 seconds works better than the original 5 s
        self.name = name
        if not self.name:
            self.name = port
        
        # Setup default handlers for standard incoming commands
        self.add_cmd_handler(ANALOG_MESSAGE, self._handle_analog_message)
        self.add_cmd_handler(DIGITAL_MESSAGE, self._handle_digital_message)
        self.add_cmd_handler(REPORT_VERSION, self._handle_report_version)
        self.add_cmd_handler(REPORT_FIRMWARE, self._handle_report_firmware)
        self.add_cmd_handler(CAPABILITY_RESPONSE, self._handle_capability_query)
        self.add_cmd_handler(ANALOG_MAPPING_RESPONSE, self._handle_analog_mapping_query)
        self.add_cmd_handler(PIN_STATE_RESPONSE, self._handle_pin_state_query)
        self.add_cmd_handler(STRING_DATA, self._handle_string_data)

        # Iterate over the first messages to get firmware data
        while self.bytes_available():
            self.iterate()

        # Add layout-specific details to the board
        if layout == "autodetect":
            self.autodetect_layout()
            # Iterate over the incoming Query Result message in this function
            # might take some time... If problems, check the
            # autodetect-function and increase the pass_time() line in there.
        else:
            self.predefined_layout(layout)
            
        
    def __str__(self):
        return "Board {0.name} on {0.sp.port}".format(self)
        

    def __del__(self):
        """ 
        The connection with the a board can get messed up when a script is
        closed without calling board.exit() (which closes the serial
        connection). Therefore also do it here and hope it helps.
        """
        self.exit()


    def autodetect_layout(self):
        """Autodetect a board layout
        
        Use the SysEX Capability Query command to request information about 
        the board layout automatically.
        """
        print("INFO: Now trying to obtain the board's pin layout automatically...")

        # Send the Capability Query request
        self.sp.write(bytearray([0xF0, 0x6B, 0xF7]))
        # On Arduino Duemilanove, reading the full query takes ~ 0.03 s.
        # If a board hangs on this step (i.e., the data is not returned fast
        # enough), try to increase the pass_time period which adds more time 
        # to receive all sent data.
        self.pass_time(1)
        while self.bytes_available():
            self.iterate()

        # Send the Analog Mapping Query request to fetch which pins have
        # analog capability
        self.sp.write(bytearray([0xF0, 0x69, 0xF7]))
        self.pass_time(1)
        while self.bytes_available():
            self.iterate()
        
        # Construct the Ports. 
        #  It appears that two different approaches can be taken:
        #  1. define Ports as 'having 8 pins', start with first pin reported, 
        #     add until 8 pins are found. This approach is taken by the 
        #     Firmata stand-alone test program found at http://www.pjrc.com/tmp/firmata/readme.html
        #     and is used also here.
        #  2. define Ports as looking at the chip layout. This approach is taken
        #     by the Arduino's native code, where for example PORTB maps to 
        #     pins 8-13 (i.e., 6 pins for this port). 
        # The Firmata protocol seems to use case (1), so let's follow that for now
        
        #
        # First, obtain a sequence of pin numbers for construction of a port.
        # Note that range(0, len(self.pins)) won't work: it is not guaranteed 
        # that pin numbers are all sequential and start at 0
        pin_nrs = []
        for pin in self.pins:
            pin_nrs.append(self.pins[pin].pin_number)
        pin_nrs = sorted(pin_nrs)       # all pin numbers are now known, sorted

        # bin the pins to ports
        total_ports = divmod(len(self.pins), 8)[0] + 1
        for portNr in range(total_ports):
            pins_in_this_port = pin_nrs[0 + portNr*8:8 + portNr*8]
            self.ports[portNr] = Port(self, portNr, pins_in_this_port)

        # (mainly for testing purposes: print out the detected board info)
        print("INFO: The board reports the following information:")
        self.report_board_information()


    def predefined_layout(self, board_layout):
        """
        Setup the Pin instances based on a given board-layout. 
        
        (Use this if the 'autodetect' function does not work, 
        if startup speed is important, and the like)
        """
        # Assume that every digital port has INPUT, OUTPUT mode
        # Number of pins is equal to the digital pins available (including 
        # UNAVAILABLE pins, such as Rx and Tx -- this to keep in line with
        # the 'autodetect' functionality that does report such pins)
        for nr in board_layout['digital']:
            self.pins[nr] = Pin(self, nr)
            self.pins[nr].INPUT_CAPABLE = True
            self.pins[nr].OUTPUT_CAPABLE = True
        
        # Mark the unavailable pins
        for nr in board_layout['unavailable']:
            self.pins[nr].mode = UNAVAILABLE
        
        # Mark PWM, SERVO, I2C pins:
        for nr in board_layout['pwm']:
            self.pins[nr].PWM_CAPABLE = True
        for nr in board_layout['i2c']:
            self.pins[nr].I2C_CAPABLE = True
        for nr in board_layout['servo']:
            self.pins[nr].SERVO_CAPABLE = True

        # mark Analog pins
        for nr in board_layout['analog']:
            self.pins[nr].ANALOG_CAPABLE = True
        
        # Create an Analog to Digital mapping. Assume the first pin with 
        # Analog_capable is A0, the next A1, etc
        # This code is analogous to the code in '_handle_analog_mapping_query
        #
        # 1. Obtain a list of all the digital pin numbers. Note that it is not 
        #    guaranteed that the pin numbers are sequential and start with 0.
        pin_nrs = []
        for pin in self.pins:
            pin_nrs.append(self.pins[pin].pin_number)
        pin_nrs = sorted(pin_nrs)

        # 2. For each of the pins detected, find their associated analog pin.
        curr_analog_counter = 0
        for dig_pin_nr in pin_nrs:
            if self.pins[dig_pin_nr].ANALOG_CAPABLE == True:
                self.analog_pins[curr_analog_counter] = self.pins[dig_pin_nr]
                self.analog_pins[curr_analog_counter].ANALOG_QUERIED = curr_analog_counter
                curr_analog_counter += 1

        # bin the pins to ports. Identical as in 'autodecect' function
        total_ports = divmod(len(self.pins), 8)[0] + 1
        for portNr in range(total_ports):
            pins_in_this_port = pin_nrs[0 + portNr*8:8 + portNr*8]
            self.ports[portNr] = Port(self, portNr, pins_in_this_port)

        print("INFO: The following information is available for this board:")
        self.report_board_information()


    def add_cmd_handler(self, cmd, func):
        """ 
        Adds a command handler to a SysEx command.
        """
        len_args = len(inspect.getargspec(func)[0])
        def add_meta(f):
            def decorator(*args, **kwargs):
                f(*args, **kwargs)
            decorator.bytes_needed = len_args - 1 # exclude self
            decorator.__name__ = f.__name__
            return decorator
        func = add_meta(func)
        self._command_handlers[cmd] = func


    def get_pin(self, pin_def):
        """
        Returns the activated pin given by the pin definition.
        
        :arg pin_def: Pin definition as described in TODO,
            but without the arduino name. So for example ``a:1:i``.
        
        Comment DV:
        Kept for backwards compatibility reasons.

        I propose that this function becomes depreciated. It combines
        two separate functions:
          a) assign a pin instance to variable
          b) set the pin mode
        Generally speaking, combining two operations within one function may
        lead to unclear code. In this case, the pin needs to be assigned only
        once, but pin mode can switch in the course of the program. 
        
        In addition, this string-interpretation solution is not elegant and 
        might lead to less readable code. To illustrate:

        This method:
            pin3 = board.getpin('d:3:p')

        Alternative method:
            pin3 = board.pins[3]    
            pin3.mode = 'pwm'
        
        Error handling in the 'Pin.read' and 'Pin.write' function will catch
        the not correct assignment of a pin...
        """
        if type(pin_def) == list:
            bits = pin_def
        else:
            bits = pin_def.split(':')

        a_d = bits[0].lower()           # in case 'A:1:P' is used
        pin_nr = int(bits[1])
        new_mode = bits[2].lower()      # in case 'A:1:P' is used

        # Sanity checks
        # If pin number requested does not exist
        if a_d not in {'a', 'd'}:
            raise InvalidPinDefError("Command does not start with 'a' or 'd'")

        if a_d == 'a' and pin_nr not in self.analog_pins:
            raise InvalidPinDefError("There is no analog pin with number {0}".format(pin_nr))
        elif a_d == 'd' and pin_nr not in self.pins:
            raise InvalidPinDefError("There is no digital pin with number {0}".format(pin_nr))

        # The pin exists, so lets grab the pin instance.
        if a_d == 'a':
            curr_pin = self.analog_pins[pin_nr]
        else:
            curr_pin = self.pins[pin_nr]

        # If pin is marked as unavailable:
        if curr_pin.mode == UNAVAILABLE:
            raise InvalidPinDefError("Pin {0} is marked as being UNAVAILABLE".format(pin_nr))
        # If pin is taken:
        if curr_pin.taken:
            raise PinAlreadyTakenError("{0} pin {1} has been taken already".format(a_d, pin_nr))

        # ok, we can use this pin instance
        # Note that pin type checking (digital/analog) happens in in Pin._set_mode()
        if new_mode == 'p':
            curr_pin.mode = PWM
        elif new_mode == 's':
            curr_pin.mode = SERVO
        elif new_mode == 'o':
            curr_pin.mode = OUTPUT
        elif new_mode == 'i':
            if a_d == 'a':
                curr_pin.mode = ANALOG
            else:
                curr_pin.mode = INPUT
        elif new_mode == 'a':
            curr_pin.mode = ANALOG
        elif new_mode == 'i2c':
            curr_pin.mode = I2C

        # If analog or digital_input, run enable_reporting
        curr_pin.enable_reporting()

        return curr_pin         # return the pin instance


    def set_pin(self, pin, new_mode):
        """Set the pin mode.
        
        This function is analogous to the Arduino 'pinMode' function.
        It takes a pin number, and assignes a new mode to this pin.
        
        ## Suggested for removal ###
        Comment DV: even though I've added this function, I think it is more
                    elegant to change the mode directly (after all, we have
                    getter/setter functionality in place). So perhaps this 
                    function is not needed?
                    
                    board.set_pin(11, PWM)          # which is this function
                        
                        looks equivalent to the more elegant:
                    pin11 = board.pins[11]
                    pin11.mode = 'pwm'
        """
        curr_pin = self.pins[pin]
        
        # check if ok to modify pin mode...
        if curr_pin.mode == UNAVAILABLE:
            raise InvalidPinDefError("Pin {0} is UNAVAILABLE".format(pin))
        if new_mode not in {UNAVAILABLE, INPUT, OUTPUT, ANALOG, PWM, SERVO, I2C,
                             'unavailable', 'input', 'output', 'analog', 'pwm', 'servo', 'i2c'}:
            raise InvalidPinDefError("Mode {0} is not recognized".format(new_mode))

        # ok - pin can be set to the new mode
        curr_pin.mode = new_mode


    def read_pin(self, pin):
        """Reads the value from a pin. 
        
        There are two paradigms to read a pin value:
            1)  board.read_pin(11)
            2)  board.pins[11].read()

        Where the latter can be made more readable by first assigning a pin
        instance to a variable, and read out that variable:
                pin11 = board.pins[11]
                pin11.read()
        
        The paradigms differ in the view of whether a 'pin' instance should 
        be able to itself read/write or if this must be controlled from the
        'board' instance. In the physical world, it is the board that writes 
        and the pin has no notion about its own status and value. This 
        corresponds to (1). However, from a programming point it makes sense
        to let the pin read/write stuff (2). 
        
        From a practical point of view there is no difference: under the hood,
        the same variable is read ;)
        """
        curr_pin = self.pins[pin]
        
        # Check if pin is unavailable, or taken.
        if curr_pin.mode == UNAVAILABLE:
            raise InvalidPinDefError("Pin {0} is UNAVAILABLE".format(pin))
        if curr_pin.taken is False:
            raise InvalidPinDefError("Pin {0} is not yet taken, value is undetermined. Use 'setPin()' command first".format(pin))
        
        # if a pin is not tagged for reporting, raise error also....
        if curr_pin.reporting == False:
            raise InvalidPinDefError("Pin {0} has 'reporting' set to False".format(pin))

        return curr_pin.value
        # TODO the reporting of a pin is not 100% correct. 
        

    def write_pin(self, pin, value):
        """
        Write a value to the pin.
        
        Depending on the pin mode, the value passed to the pin will differ.
        If pin mode is: 
            OUTPUT:     the value will be rounded to 0 or 1.
            PWM:        the value is expected to be float between 0 and 1.
            SERVO:      the value should be in degrees.
        """
        curr_pin = self.pins[pin]

        if curr_pin.mode is UNAVAILABLE:
            raise IOError("ERROR: Pin {0} can not be used ".format(pin))
        if curr_pin.mode is INPUT:
            raise IOError("ERROR: Pin {0} is set up as an INPUT and can therefore not be written to".format(pin))

        if value is not curr_pin.value:
            curr_pin.value = value

            if curr_pin.mode is OUTPUT:
                # Change value to boolean
                curr_pin.value = int(round(curr_pin.value))
                # find to which Port this pin belongs to
                myPort = curr_pin.get_port()
                self.ports[myPort].write()

            elif curr_pin.mode is PWM:
                value = int(round(value * 255))
                msg = bytearray([ANALOG_MESSAGE + curr_pin.pin_number, value % 128, value >> 7])
                self.sp.write(msg)

            elif curr_pin.mode is SERVO:
                value = int(value)
                msg = bytearray([ANALOG_MESSAGE + curr_pin.pin_number, value % 128, value >> 7])
                self.sp.write(msg)


    def pass_time(self, t):
        """ 
        Non-blocking time-out for ``t`` seconds.
        """
        cont = time.time() + t
        while time.time() < cont:
            time.sleep(0)


    def send_sysex(self, sysex_cmd, data):
        """
        Sends a SysEx message to the board.
        
        :arg sysex_cmd: A sysex command byte
        :arg data: a bytearray of 7-bit bytes of arbitrary data
        """
        msg = bytearray([START_SYSEX, sysex_cmd])
        msg += data
        msg += bytearray([END_SYSEX])
        self.sp.write(msg)


    def set_sampling_interval(self, time_interval):
        """Set the interval with which Firmata reports.
        
        The sampling interval sets how often analog data and i2c data is 
           reported to the client. The default value is 19 milliseconds.         
        
        The 'time_interval' should be an integer (0 ... 32767)
        """
        self.send_sysex(SAMPLING_INTERVAL, to_two_bytes(time_interval))


    def bytes_available(self):
        return self.sp.inWaiting()


    def _manual_input_hex_values(self):
        """Enter hex values to the board directly
        
        This function enables a programmer to interrupt his program,
        and feed the Firmata raw hex values by entering them.
        
        Added for debugging and quick testing purpose
        """
    
        keep_going = True
        while keep_going:
            if sys.version_info.major == 2:
                # Python 2.x
                my_values = raw_input("Add hex values, Q is stop:")
            elif sys.version_info.major == 3:
                # Python 3.x
                my_values = input("Add hex values, Q is stop: ")
            
            bytes_to_send = bytearray()
            binary_representation = ""
        
            for i in my_values.split():
                if i in {'Q', 'q'}:
                    keep_going = False
                    break 
                else:
                    i = int(i, 16)
                    bytes_to_send.append(i)
                    binary_nr = "{0:#010b}".format(i)
                    binary_representation += "{0} {1} - ".format(binary_nr[2:6], binary_nr[6:])
            
            print("The bytearray that will be sent: ", bytes_to_send)
            print("Translated as bits: ", binary_representation)
            self.sp.write(bytesToSend)
        return


    def iterate(self):
        """ 
        Reads and handles data from the microcontroller over the serial port.
        This method should be called in a main loop, or in an
        :class:`Iterator` instance to keep this boards pin values up to date
        """
        byte = self.sp.read()
        if not byte:
            return
        data = ord(byte)
        received_data = []
        handler = None

        if data < START_SYSEX:
            # These commands can have 'channel data' like a pin nummber appended.
            try:
                handler = self._command_handlers[data & 0xF0]
            except KeyError:
                return
            received_data.append(data & 0x0F)
            while len(received_data) < handler.bytes_needed:
                received_data.append(ord(self.sp.read()))

        elif data == START_SYSEX:
            data = ord(self.sp.read())
            handler = self._command_handlers.get(data)
            if not handler:
                return
            data = ord(self.sp.read())
            while data != END_SYSEX:
                received_data.append(data)
                data = ord(self.sp.read())

        else:
            try:
                handler = self._command_handlers[data]
            except KeyError:
                return
            while len(received_data) < handler.bytes_needed:
                received_data.append(ord(self.sp.read()))
        # Handle the data
        try:
            handler(*received_data)
        except ValueError:
            pass

            
    def get_firmata_version(self):
        """
        Returns the firmata firmware version of the board.
        
        Return value is a tuple (major, minor)
        """
        return self.firmata_version

        
    def servo_config(self, pin, min_pulse=544, max_pulse=2400, angle=0):
        """
        Configure a servo pin.

        Configure a pin as servo with min_pulse, max_pulse and first angle.
        ``min_pulse`` and ``max_pulse`` default to the arduino defaults.
        
        Note that this method is also implemented in the 'Pin' class.
        """
        if self.pins[pin].SERVO_CAPABLE == False:
            raise IOError("Pin {0} is not a valid servo pin".format(pin))
        if self.pins[pin].mode == UNAVAILABLE:
            raise IOError("Pin {0} is UNAVAILABLE for Servo use".format(pin))
                
        data = bytearray([pin])
        data += to_two_bytes(min_pulse)
        data += to_two_bytes(max_pulse)
        self.send_sysex(SERVO_CONFIG, data)
        
        # set pin._mode to SERVO so that it sends analog messages
        self.pins[pin].mode = SERVO
        self.pins[pin].write(angle)

        
    def exit(self):
        """ Call this to exit cleanly. """
        # First detach all servo's, otherwise it somehow doesn't want to close...
        # FIXME
        for pin in self.pins:
            if self.pins[pin].mode == SERVO:
                self.pins[pin].mode = OUTPUT
        if hasattr(self, 'sp'):
            self.sp.close()

        
    # Command handlers
    def _handle_analog_message(self, pin_nr, lsb, msb):
        """Handle the incoming analog message.
        
        Note that the 'pin_nr' that is returned is the __analog__ pin number.
        Because of this, the 'board.analog_pins' dict was created
        to refer to either the A or D functionality of a pin.
        """

        value = round(float((msb << 7) + lsb) / 1023, 4)
        # Only set the value if we are actually reporting
        try:
            if self.analog_pins[pin_nr].reporting:
                self.analog_pins[pin_nr].value = value
        except IndexError:
            raise ValueError


    def _handle_digital_message(self, port_nr, lsb, msb):
        """
        Digital messages always go by the whole port. This means we have a
        bitmask with which the port is updated.
        """
        mask = (msb << 7) + lsb
        try:
            self.ports[port_nr]._update(mask)
        except IndexError:
            raise ValueError


    def _handle_report_version(self, major, minor):
        """Obtain the firmware version, as reported by the Firmata protocol"""
        self.firmata_version = (major, minor)


    def _handle_report_firmware(self, *data):
        """Obtain the firmware version and name, as reported by the Firmata protocol"""
        major = data[0]
        minor = data[1]
        self.firmware_version = (major, minor)
        self.firmware = two_byte_iter_to_str(data[2:])


    def _handle_capability_query(self, *data):
        """
        Query the microcontroller board for pin capabilities.
        This is the answer to the query 'F0 6B F7'.
        
        From the Firmata protocol: 
         'Each pin shall have 2 bytes for each supported mode, and a 
          single 127 (0x7F) to mark the end of that pin's data. 
          The number of pins supported shall be inferred by the message length. 
          (...). The resolution information may be used to adapt to future 
          implementations where PWM, Analog and others may have different resolution. 
        """

        print("Analyzing the board's Capability Query response: {0} pins detected".format(data.count(0x7F)))
                
        for nr in range(data.count(0x7F)):
            pin_info = data[:data.index(0x7F)]

            # Instantiate a new pin
            self.pins[nr] = Pin(self, nr)
            
            if len(pin_info) == 0:
                # pin data is only 127 (0x7F): pin is reported as unavailable
                # (NB this is confirmed for D0 and D1 (RX/TX) on Arduino Duemilanove, not for other boards)
                self.pins[nr].mode = UNAVAILABLE

            elif len(pin_info) % 2 != 0:
                # each pin has 2 bytes per mode; if odd length, something is wrong
                raise IOError("Pin {0} reports incorrect mode information: {1}".format(nr, pin_info))

            else:
                for mode_info in range(0, len(pin_info), 2):
                    if pin_info[mode_info] == INPUT:
                        self.pins[nr].INPUT_CAPABLE = True
                    if pin_info[mode_info] == OUTPUT:
                        self.pins[nr].OUTPUT_CAPABLE = True
                    if pin_info[mode_info] == ANALOG:
                        self.pins[nr].ANALOG_CAPABLE = True
                    if pin_info[mode_info] == PWM:
                        self.pins[nr].PWM_CAPABLE = True
                    if pin_info[mode_info] == SERVO:
                        self.pins[nr].SERVO_CAPABLE = True
                    if pin_info[mode_info] == I2C:
                        self.pins[nr].I2C_CAPABLE = True

                # Proceed to next pin's data
            data = data[data.index(0x7F)+1:]


    def _handle_analog_mapping_query(self, *data):
        """
        Query the microcontroller board for which pins are mapped as analog.
        This is the answer to the query 'F0 69 F7'.        
        """
        print("INFO: Analyzing the board Analog Mapping response...")
        print("Analyzing the board's Analog Mapping response: {0} pins to be analyzed".format(len(data)))

        for nr in range(len(data)):
            if data[nr] != 0x7F:            # 0xF7/127: no analog pin
                self.pins[nr].ANALOG_QUERIED = data[nr]
                self.analog_pins[data[nr]] = self.pins[nr]


    def _handle_string_data(self, *data):
        """Handle incoming string data (0x71)
        
        Note that string messages can be relatively long -- make sure that the
        whole string to up to the END_SYSEX is received or the string will not
        be displayed. Easiest is to open an 'it = iter.Iterator(board)' instance"""

        msg = []
        for pair in range(0, len(data), 2):
            together = from_two_bytes((data[pair], data[pair+1]))
            msg.append(chr(together))
        
        print("Firmata sent the message '{0}'".format("".join(msg)))

        # Since this function is called from within a handle, to simply return
        # the value will not (always) work as expected. Therefore, the 
        # messages received are stored inside the board.messages_received' 
        # variable -- especially relevant for I2C reports. In this way, the 
        # program can read back all messages received by querying this variable.
        self.messages_received.append("".join(msg))


    def _handle_pin_state_query(self, *data):
        """
        Query the microcontroller board for one pin's state.
        This is the answer to the query 'F0 6D [pin_number] F7'. 
        
        (Note: make sure that all data are captured, for example by 
        running the 'iter.Iterator(board)' function continuously.
        """
        queried_pin = data[0]
        queried_mode = data[1]
        queried_state = data[2:]

        print("Pin {0} has mode {1} with data {2}".
                format(queried_pin, queried_mode, queried_state))


    def report_board_information(self):
        """Print general information about the current board"""

        analog = []
        servo = []
        pwm = []
        i2c = []
        unavailable = []
        analog_queried = []

        for pin in self.pins:
            if self.pins[pin].ANALOG_CAPABLE is True:
                analog.append(pin)
            if self.pins[pin].SERVO_CAPABLE is True:
                servo.append(pin)
            if self.pins[pin].PWM_CAPABLE is True:
                pwm.append(pin)
            if self.pins[pin].I2C_CAPABLE is True:
                i2c.append(pin)
            if self.pins[pin].mode is UNAVAILABLE:
                unavailable.append(pin)
            if self.pins[pin].ANALOG_QUERIED is not None:
                analog_queried.append("A{0.ANALOG_QUERIED}=D{0.pin_number}".
                                    format(self.pins[pin]))
        
        report = """
        > Board {0.name}, running '{0.firmware}' version {0.firmata_version[0]}.{0.firmata_version[1]}.
        > Port {0.sp.port}, serial info {0.sp}

        > Pin information
          ANALOG        {1}
          ANALOG_MAPPED {2}
          PWM           {3}
          SERVO         {4}
          I2C pins      {5}
          UNAVAILABLE:  {6}""".format(self, analog, analog_queried, pwm, servo, i2c, unavailable)
        print(report)
        
        for i in self.ports:
            print("\tPort {0} with pins {1}".format(i, self.ports[i].pins))


class Port(object):
    """ An 8-bit port on the board """
    def __init__(self, board, port_number, pins):
        self.board = board
        self.port_number = port_number
        self.reporting = False
        self.pins = tuple(pins)     # tuple of pin numbers associated with this port


    def __str__(self):
        """Return readable information of a Port instance"""
        return "Port {0.port_number} on {0.board}".format(self)


    def enable_reporting(self):
        """ Enable reporting of values for the whole port """
        self.reporting = True
        msg = bytearray([REPORT_DIGITAL + self.port_number, 1])
        self.board.sp.write(msg)

        for pin in self.pins:
            if self.board.pins[pin].mode == INPUT:
                self.board.pins[pin].reporting = True # TODO Shouldn't this happen at the pin?


    def disable_reporting(self):
        """ Disable the reporting of the port """
        self.reporting = False
        msg = bytearray([REPORT_DIGITAL + self.port_number, 0])

        for pin in self.pins:
            if self.board.pins[pin] == INPUT:
                self.board.pins[pin].reporting = False


    def write(self):
        """Set the output pins of the port to the correct state"""
        mask = 0

        # only report for pins that are taken, and OUTPUT, and have value 
        for pin in self.pins:
            if self.board.pins[pin].taken == True:
                if self.board.pins[pin].mode == OUTPUT:
                    if self.board.pins[pin].value == 1:
                        pin_nr = self.pins.index(pin)
                        mask |= 1 << pin_nr

        msg = bytearray([DIGITAL_MESSAGE + self.port_number, mask % 128, mask >> 7])
        self.board.sp.write(msg)


    def _update(self, mask):
        """
        Update the values for the pins marked as input with the mask.
        """
        if self.reporting:
            for pin in self.pins:
                if self.board.pins[pin].mode == INPUT:
                    pin_nr = self.pins.index(pin)
                    self.board.pins[pin].value = (mask & (1 << pin_nr)) > 0


class Pin(object):
    """ A Pin representation 
    
    Each 'pin' instance is linked to a physical pin on the board. 
    It stores information about its own state, value, capabilities, etc.,
    but does not necessarily know a lot about other Pins, Ports, and the board.
    """
    def __init__(self, board, pin_number):
        self.board = board
        self.pin_number = pin_number
        self.value = None
        self.reporting = False
        self.taken = False
        self._mode = INPUT           # See http://arduino.cc/en/Tutorial/DigitalPins:
                                     # 'Arduino (Atmega) pins default to inputs (...)'
        self.INPUT_CAPABLE = False
        self.OUTPUT_CAPABLE = False
        self.ANALOG_CAPABLE = False
        self.ANALOG_QUERIED = None
        self.PWM_CAPABLE = False
        self.SERVO_CAPABLE = False
        self.I2C_CAPABLE = False


    def __str__(self):
        """Print Pin information in readable format"""
        return "Pin {0.pin_number}, mode {0.mode}, value {0.value}".format(self)


    def _get_mode(self):
        """Getter function for the pin mode"""
        return self._mode


    def _set_mode(self, new_mode):
        """Setter function for the pin mode"""

        if type(new_mode) is str:
            new_mode = new_mode.lower()

        # Some sanity checks
        if new_mode not in {UNAVAILABLE, INPUT, OUTPUT, ANALOG, PWM, SERVO, I2C,
                             'unavailable', 'input', 'output', 'analog', 'pwm', 'servo', 'i2c'}:
            raise InvalidPinDefError("Mode {0} is not recognized".format(new_mode))
        if self._mode in {UNAVAILABLE, 'unavailable'}:
            raise InvalidPinDefError("Pin {0} is UNAVAILABLE".format(self.pin_nr))
        if self.taken == True:
            print("WARNING: Pin {0} is already taken".format(self.pin_number))

        # Set the mode
        if new_mode in {UNAVAILABLE, 'unavailable'}:
            # Make sure no function accidently will use any of the Boolean indicators
            self._mode = UNAVAILABLE
            self.taken = True
            self.reporting = False
            self.value = None
            return

        elif new_mode in {INPUT, 'input'}:
            if self.INPUT_CAPABLE:
                self._mode = INPUT
                self.reporting = False      # reporting is set per digital port
                self.taken = True
            else:
                raise InvalidPinDefError("Pin {0} has no INPUT mode".format(self.pin_number))

        elif new_mode in {OUTPUT, 'output'}:
            if self.OUTPUT_CAPABLE:
                self._mode = OUTPUT
                self.taken = True
                self.reporting = False
            else:
                raise InvalidPinDefError("ERROR: Pin {0} has no OUTPUT mode".format(self.pin_number))

        elif new_mode in {ANALOG, 'analog'}:
            if self.ANALOG_CAPABLE:
                self._mode = ANALOG
                self.reporting = False
                self.taken = True
            else:
                print("WARNING: Pin {0} has no ANALOG mode".format(self.pin_number))

        elif new_mode in {SERVO, 'servo'}:
            if self.SERVO_CAPABLE:
                self._mode = SERVO
                self.taken = True
                self.reporting = False

            else:
                raise InvalidPinDefError("ERROR: Pin {0} is not SERVO capable".format(self.pin_number))

        elif new_mode in {PWM, 'pwm'}:
            if self.PWM_CAPABLE:
                self.taken = True
                self._mode = PWM
                self.reporting = False
            else:
                raise InvalidPinDefError("ERROR: Pin {0} is not PWM capable".format(self.pin_number))

        elif new_mode in {I2C, 'i2c'}:
            if self.I2C_CAPABLE:
                self.taken = True
                self._mode = I2C
                self.reporting = False
                self.value = None
            else:
                raise InvalidPinDefError("ERROR: Pin {0} is not I2C capable".format(self.pin_number))

        self.board.sp.write(bytearray([SET_PIN_MODE, self.pin_number, self.mode]))
        self.enable_reporting()


    mode = property(_get_mode, _set_mode)
    """
    Mode of operation for the pin. 

    Can be one of the pin modes: UNAVAILABLE, INPUT, OUTPUT, ANALOG, PWM, 
    SERVO, I2C, 'unavailable', 'input', 'output', 'analog', 'pwm', 'servo', 'i2c'
    (See the Pin._set_mode() function for details)
    """


    def enable_reporting(self):
        """ Set an input pin to report values """

        if self.ANALOG_CAPABLE and self.mode == ANALOG:
            # Firmata protocol: analog reporting goes per pin
            self.reporting = True
            msg = bytearray([REPORT_ANALOG + self.ANALOG_QUERIED, 1])
            self.board.sp.write(msg)

        if self.INPUT_CAPABLE and self.mode == INPUT:
            # Firmata protocol: digital reporting goes per port
            myPort = self.get_port()
            self.board.ports[myPort].enable_reporting()


    def disable_reporting(self):
        """ Disable the reporting of an input pin """
        if self.mode == ANALOG:
            self.reporting = False
            msg = bytearray([REPORT_ANALOG + self.pin_number, 0])
            self.board.sp.write(msg)
        else:
            myPort = self.get_port()
            self.board.ports[myPort].disable_reporting()


    def get_port(self):
        """Return the Port instance to which this Pin belongs to.
        """
        for myPort in self.board.ports:
            if self.pin_number in self.board.ports[myPort].pins:
                return myPort

        raise InvalidPinDefError("ERROR: pin {0} is not assigned to any port.".format(self.pin_number))


    def read(self):
        """Returns the output value of the Pin.
        
        Note: this function is a proxy for the 'board.read_pin()' method.
        """
        return self.board.read_pin(self.pin_number)


    def write(self, value):
        """Output a voltage from the Pin.
        
        Note: this function is a proxy for the 'board.write_pin()' method.
        """
        return self.board.write_pin(self.pin_number, value)


    def query_state(self):
        """Query the state of this pin
        
        From the Firmata protocol:
        The pin state query allows the GUI to read the current configuration 
        of any pin. Normally this is needed when a GUI-based program starts up, 
        so it can populate on-screen controls with an accurate representation 
        of the hardware's configuration. This query can also be used to 
        verify pin mode settings are received properly.
        """
        self.board.sp.write(bytearray([START_SYSEX, PIN_STATE_QUERY, self.pin_number, END_SYSEX]))


    def servo_config(self, min_pulse=544, max_pulse=2400, angle=0):
        """
        Configure a servo pin.

        Configure a pin as servo with min_pulse, max_pulse and first angle.
        ``min_pulse`` and ``max_pulse`` default to the arduino defaults.
        
        NOTE: While there is also a 'board.servo_config()' method, the advantage of
        this method inside the Pin class is that a pin can be assigned to a 
        variable, and then the servo_config can be activated:
            pin11 = board.pins[7]
            pin11.servo_config()        
        """
        if self.SERVO_CAPABLE == False:
            raise IOError("Pin {0} is not a valid servo pin".format(pin))
        if self.mode == UNAVAILABLE:
            raise IOError("Pin {0} is UNAVAILABLE for Servo use".format(pin))
                
        data = bytearray([self.pin_number])
        data += to_two_bytes(min_pulse)
        data += to_two_bytes(max_pulse)
        self.board.send_sysex(SERVO_CONFIG, data)
        
        # set pin._mode to SERVO so that it sends analog messages
        self.mode = SERVO
        self.write(angle)


    def report_pin_information(self):
        """Print the Pin state onto the screen"""

        report = """> Report on Pin {0.pin_number}:
                    > mode            = {0.mode}
                    > value           = {0.value}
                    > INPUT_CAPABLE   = {0.INPUT_CAPABLE}
                    > OUTPUT_CAPABLE  = {0.OUTPUT_CAPABLE}
                    > ANALOG_CAPABLE  = {0.ANALOG_CAPABLE}
                    > ANALOG_QUERIED  = {0.ANALOG_QUERIED}
                    > SERVO_CAPABLE   = {0.SERVO_CAPABLE}
                    > PWM_CAPABLE     = {0.PWM_CAPABLE}
                    > I2C_CAPABLE     = {0.I2C_CAPABLE}
                    > pin is taken    = {0.taken}
                    > reporting       = {0.reporting}""".format(self)
        print(report)
        return
