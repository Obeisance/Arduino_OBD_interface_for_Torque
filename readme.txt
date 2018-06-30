This code is designed to enable an arduino UNO
to replace the function of an ELM327 microcontroller
OBD interface that will respond to data queries
via Serial bluetooth interface. This should also
work with the Torque OBD android application.

I wrote this to use the ISO-9141 protocol that
matches my car. Feel free to modify it to meet
whatever protocol is relevant for you.

An HC05 Serial-to-bluetooth adapter is needed,
installed on Arduino digital pins 2 (Rx) and 3 (Tx).

The OBD interface is facilitated by an LM339 quad comparator
which serves as a voltage level shifter (5V TTL on 
Arduino to 12V on the K-line). Be sure to power
the quad comparator with 12V and share a common ground
with the car and Arduino. The K-line from the
OBD port must be connected to an output pin from one
comparator on the LM339 (with a pullup resistor
to 12V) as well as an (in-) pin on the comparator.
The (in+) on the K-line input (Rx) comparator must
be pulled to be between 12 and 0V. The K-line 
output on the comparator (Tx) should be driven 
by the Arduino Tx (pin 9) on an (in-) pin (remember
the 5V pullup). The (in+) pin for this comparator
should be pulled to be between 5 and 0V via a
voltage divider. The Arduino Rx (pin 8) should 
come from the comparator driven by the K-line input.

So far the code is untested.