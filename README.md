# RP2040-Instruments
These instruments are for use in flight simulator

Arduino Instruments on RP2040 1.28" Display

How to update the display:

Values are sent over serial through the com port at a baud rate of 9600

Air speed indicator
Send: 's' followed by exactly 4 digits indicating the speed in knots
Example: 's0060' for 60 knots and 's0124' for 124 knots

Altimeter indicator
Send first: 'k' followed by exactly 4 digits indicating the Kollsman in inHg at see level leaving out the decimal.
Example: 'k2992' which is 29.92inHg or 'k2978' for 29.78inHg
Note: The indicator range is from 28.00inHg to 32.00inHg
Send second: 'a' followed by exactly 6 digits indicating altitude
Example: 'a000100' which is 100 feet above sea level or 'a012052' which is 12,052 feet above sea level
Full Example: 'k2992a000100' for 29.92inHg at sea level and 100 feet above sea level.

Attitude indicator
Send first: 'p' followed by exactly 3 digits indicating pitch in degrees
Example: 'p010' which is 10deg or 'p-020' for -20deg
Send second: 'b' followed by exactly 3 digits indicating bank
Example: 'b060' which is 60deg or 'b-030' for -30deg
Full Example: 'p010b060' for pitch of 10 deg and bank of 60 deg

Heading indicator
Send: 'h' followed by exactly 3 digits indicating the heading
Example: 'h030' for a heading of 30 degrees and 'h335' for a heading of 335 degrees

Turn Coordinator indicator
Send first: 'r' followed by exactly 3 digits indicating the rate in a custom unit
Send second: 'b' followed by exactly 3 digits indicating ball position from between -127 and 127
Example: 'r060b050'
Full Example: 'p010b060' for pitch of 10 deg and bank of 60 deg

Vertical Speed indicator
Send: 'v' followed by exactly 4 digits indicating the vertical speed in feet per minute divided by 100
Example: 'v1000' for 1000feet per minute or 'v-0700' for -700 feet per minute

