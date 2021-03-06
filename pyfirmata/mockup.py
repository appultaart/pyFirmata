from collections import deque

import pyfirmata


class MockupSerial(deque):
    """ 
    A Mockup object for python's Serial. Functions as a fifo-stack. Push to
    it with ``write``, read from it with ``read``.
    
    >>> s = MockupSerial('someport', 4800)
    >>> s.read()
    ''
    >>> s.write(chr(100))
    >>> s.write('blaat')
    >>> s.write(100000)
    >>> s.read(2)
    ['d', 'blaat']
    >>> s.read()
    100000
    >>> s.read()
    ''
    >>> s.read(2)
    ['', '']
    >>> s.close()
    """
    def __init__(self, port, baudrate, timeout=0.02):
        self.port = port or 'somewhere'
        
    def read(self, count=1):
        if count > 1:
            val = []
            for i in range(count):
                try:
                    val.append(self.popleft())
                except IndexError:
                    val.append(None)
        else:
            try:
                val = self.popleft()
            except IndexError:
                val = None

        if val is not None:
            return bytearray([val])
        else:
            return

    def write(self, value):
        """
        Appends items flat to the deque. So iterables will be unpacked.
        """
        if hasattr(value, '__iter__'):
            self.extend(value)
        else:
            self.append(value)

            
    def close(self):
        self.clear()
        
    def inWaiting(self):
        return len(self)
        
class MockupBoard(pyfirmata.Board):

    def __init__(self, port, layout, values_dict={}):
        self.sp = MockupSerial(port, 57600)
        self.setup_layout(layout)
        self.values_dict = values_dict
        self.id = 1
        self.name = "MockupBoard"
        self.add_cmd_handler(pyfirmata.ANALOG_MESSAGE, self._handle_analog_message)
        self.add_cmd_handler(pyfirmata.DIGITAL_MESSAGE, self._handle_digital_message)
        self.add_cmd_handler(pyfirmata.REPORT_VERSION, self._handle_report_version)
        self.add_cmd_handler(pyfirmata.REPORT_FIRMWARE, self._handle_report_firmware)


    def reset_taken(self):
        for pin in self.pins:
            self.pins[pin].taken = False
        
    def update_values_dict(self):
        for port in self.ports:
            self.ports[port].values_dict = self.values_dict
            self.ports[port].update_values_dict()
        for pin in self.pins:
            self.pins[pin].values_dict = self.values_dict


class MockupPort(pyfirmata.Port):
    def __init__(self, board, port_number):
        self.board = board
        self.port_number = port_number
        self.reporting = False
        
        self.pins = []
        for i in range(8):
            pin_nr = i + self.port_number * 8
            self.pins.append(MockupPin(self.board, pin_nr, type=pyfirmata.DIGITAL, port=self))


    def update_values_dict(self):
        for pin in self.pins:
            pin.values_dict = self.values_dict
        

class MockupPin(pyfirmata.Pin):
    def __init__(self, *args, **kwargs):
        self.values_dict = kwargs.get('values_dict', {})

        try:
            del kwargs['values_dict']
        except KeyError:
            pass
        super(MockupPin, self).__init__(*args, **kwargs)
    
    def read(self):
        if self.value is None:
            try:
                type = self.port and 'd' or 'a'
                return self.values_dict[type][self.pin_number]
            except KeyError:
                return None
        else:
            return self.value
            
    def get_in_output(self):
        if not self.port and not self.mode: # analog input
            return 'i'
        else:
            return 'o'
            
    def set_active(self, active):
        self.is_active = active
        
    def get_active(self):
        return self.is_active
        
    def write(self, value):
        if self.mode == pyfirmata.UNAVAILABLE:
            raise IOError("Cannot read from pin {0}".format(self.pin_number))
        if self.mode == pyfirmata.INPUT:
            raise IOError("{0} pin {1} is not an output".format(self.port and "Digital" or "Analog", self.get_pin_number()))
        if not self.port:
            raise AttributeError("AnalogPin instance has no attribute 'write'")
        # if value != self.read():
        self.value = value
        
class Iterator(object):
    def __init__(self, *args, **kwargs):
        pass
    def start(self):
        pass
    def stop(self):
        pass

if __name__ == '__main__':
    import doctest
    doctest.testmod()
