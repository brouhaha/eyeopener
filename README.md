Vector generator test firmware
RetroChallenge 2015/07 project
Copyright 2015 Eric Smith <spacewar@gmail.com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License version 3 as
published by the Free Software Foundation

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

"System Libraries" as defined in section 1 of the GPL is construed
to include the source and/or object code of the Silicon Labs EFM32 SDK,
which is available for download by Silicon Labs "Simplicity Studio":
  http://www.silabs.com/products/mcu/Pages/simplicity-studio.aspx

For use with an updated version of the vector generator by
Steve Ciarcia and Hal Chamberlin:

  "Make Your Next Peripheral a Real Eye Opener"
  Steve Ciarcia, BYTE magazine, November 1976

  "A Graphics Display for the 8008" Part 1-3
  Hal Chamberlin, The Computer Hobbyist Volume 1 Numbers 1-3, 1974-1975

Updated vector generator schematic:
  http://www.brouhaha.com/~eric/retrocomputing/retrochallenge-2015.07/eyeopener2.pdf

This code presently drives a single channel of the
vector generator for test purposes. However, driving
two (or even three) channels requires only minor
modifications.

Uses USART1 to control Microchip MCP4822 DACs.
Uses LEUART0 serial port at 9600 bps for control.
Uses GPIO pins for DAC chip selects and to control analog
switches for vector generation.

The "0" through "9" commands (digits) are used to set endpoint
coordinates, scaled from 0 to 4095.

The "v" (vector) command draws a vector between the last two
endpoints entered.
