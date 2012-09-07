# Copyright (c) 2012, Fabian Affolter <fabian@affolter-engineering.ch>
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of the pyfirmata team nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ''AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
# ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import pyfirmata

# Adjust that the port match your system, see samples below:
# On Linux: /dev/tty.usbserial-A6008rIF, /dev/ttyACM0, 
# On Windows: \\.\COM1, \\.\COM2
PORT = '/dev/ttyUSB0'

# Creates a new board -- use autodetect function 
board = pyfirmata.Board(PORT, layout="autodetect", name='Arduino')

it = pyfirmata.util.Iterator(board)
it.start()

# Definition of the analog pin
PINS = (0, 1, 2, 3)

# Start reporting for defined pins
for pin in PINS:
    board.analog_pins[pin].mode = "analog"
    board.analog_pins[pin].enable_reporting()


# Loop for reading the input. Duration approx. 10 s
for i in range(1, 11):
    print("\nValues after {0} second(s)".format(i))
    for pin in PINS:
        print("Pin {0} : {1}".format(pin, board.analog_pins[pin].read()))
        print("Query: pin {0}:".format(board.analog_pins[pin].query_state()))
    board.pass_time(2)

board.exit()
