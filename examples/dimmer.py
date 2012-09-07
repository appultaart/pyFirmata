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

PORT = '/dev/ttyACM0'

# Time (approx. seconds) to get to the maximum/minimum
DURATION = 5
# Numbers of steps to get to the maximum/minimum
STEPS = 10 

# Creates a new board and define the pin
board = pyfirmata.Arduino(PORT)
# Pin use one that supoorts ~ (PWM) (e.g. 3, 5, 6, 9, 10, or 11)
digital_0 = board.get_pin('d:11:p')

# Waiting time between the 
wait_time = DURATION/float(STEPS)

# Up
for i in range(1, STEPS + 1):
    value = i/float(STEPS)
    digital_0.write(value)
    board.pass_time(wait_time)

# Down
increment = 1/float(STEPS)
while STEPS > 0:
    value = increment * STEPS
    digital_0.write(value)
    board.pass_time(wait_time)
    STEPS = STEPS - 1

board.exit()
