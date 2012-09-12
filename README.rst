-*- restructuredtext -*-

=========
pyFirmata
=========
Firmata (http://firmata.org) is a generic protocol for communicating with microcontrollers (such as the popular Arduino boards) from a host computer. 
The ``pyFirmata`` module aims to provide a python host software API to communicate with Firmata.

Usage
=====

Initiate a board
----------------

Firmata v2.2 supports autodetect functionality, which automatically determines what pins are available on the board and what their capabilities are. To use:

    >>> from pyfirmata import *
    >>> PORT = '/dev/ttyUSB0'
    >>> board = pyfirmata.Board(PORT, layout="autodetect", name='Arduino')

If the autodetect functionality does not work, a predefined board layout can be used. Such a layout can be modified in ``pyFirmata.boards.py``. An example of loading an Arduino using predefined layout:

    >>> from pyfirmata import *
    >>> PORT = '/dev/ttyUSB0'
    >>> board = pyfirmata.Board(PORT, layout=boards.BOARDS['arduino'], name='Arduino')
 

Iterator thread
---------------

When data is sent by the board at a regular interval, it is best to catch those data by an iterator thread that processes the data. Otherwise, the serial connection might overflow by incoming data, or (long) incoming data messages might become processed only partially. Especially when reading a pin's analog values, using the iterator ensures that all data are read properly. (When only few digital ON/OFF signals need to be sent, running the iterator is less useful).

The iterator thread runs in the background, so you only need to call it once, usually at the beginnning of your program:

    >>> it = util.Iterator(board)
    >>> it.start()


Analog and Digital pins
-----------------------
All of the pins on Arduino-like boards can be used as digital input/output. However, some pins can read 10-bit (0...1023) values, a property that make them useful to read analog sensors, for example. Hence, these pins are termed ``analog pins``. 
In the current pyFirmata code, the ``board`` instance stores pin information in two ways. The ``board.pins`` dict stores all pins (*i.e.*, all digital pins), including pins that are reported as unavailable such as Rx and Tx. The ``board.analog_pins`` dict stores those pins with analog capability, where the key is the analog pin number.
Thus, the following two variables point to the identical pin instance::

    >>> digitalPin = board.pins[14]
    >>> analogPin = board.analog_pins[0]


Assigning pins
--------------
To access a pin, it is easiest to assign it to a variable. 
There are various ways to assign a variable to a specific pin. These approaches should be interchangeable, and depend on your preference.

1. Assign a pin first, then change its mode. While this is a two-step procedure, it makes your code easy to read. The mode is a number (see Firmata protocol, e.g. INPUT = 1), or a string ('input', 'output', 'servo', 'pwm', 'i2c', 'analog').

    >>> analog_0 = board.analog_pins[0]
    >>> analog_0.mode = 'analog'
    >>> analog_0.read()
    0.6614
    >>> pin3 = board.pins[3]
    >>> pin3.mode = 'pwm'
    >>> pin3.write(0.6)

2. Use the ``get_pin`` method. You specify what pin you need by a string, such as ``a:2:i``. The string consists of three parts, separated by ':'. The first part us either 'a' or 'd', depending on wether you need an analog or digital pin. The second part indicates the pin number. The mode is the third part of the string. Mode values are 'i' for input, 'o' for output, 'p' for pwm, 's' for servo, 'i2c' for I2C, and 'a' for analog input. 

    >>> analog_0 = board.get_pin('a:0:1')
    >>> analog_0.enable_reporting()
    >>> analog_0.read()
    0.6614
    >>> pin3 = board.get_pin('d:3:p')
    >>> pin3.write(0.6)

Enable reporting
----------------
Before reading data from a pin, Firmata must be instructed to enable reporting for a pin. This is done with the ''enable_reporting'' method. The pin's value can now be read by the pin's ''read'' method as is done above, or by directly reporting its ''value'' variable.

    >>> anaPin = board.analog_pins[2]
    >>> anaPin.mode = 'analog'
    >>> anaPin.enable_reporting()
    >>> print("Value:", anaPin.value)
    0.6614

Setting the sampling interval
-----------------------------
By default, Firmata reports values approximately every 19 milliseconds (ms). You can change this sampling interval time by submitting a new interval time in ms::

    >>> board.set_sampling_interval(1000)	
    >>> # values are reported every second

Board layout
============

If you want to use a board with a different layout than the standard Arduino, or the Arduino Mega (for wich there exist the shortcut classes ``pyfirmata.Arduino`` and ``pyfirmata.ArduinoMega``), instantiate the Board class with a dictionary as the ``layout`` argument. This is the layout dict for the Mega for example::

    >>> mega = {
    ...         'digital' : tuple(x for x in range(54)),
    ...         'analog' : tuple(x for x in range(16)),
    ...         'pwm' : tuple(x for x in range(2,14)),
    ...         'use_ports' : True,
    ...         'disabled' : (0, 1, 14, 15) # Rx, Tx, Crystal
    ...         }

Todo
====
The adaptation of the pyFirmata protocol for Python3 turned out to involve quite a bit of code rewrite. For one reason, handling of string/byte data is handled differently in Python2 and Python3. For another reason, the implementation of automatic capability query required internal restructuring of the Pin and Port instances.

Therefore, TESTING TESTING TESTING is needed.
