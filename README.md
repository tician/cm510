cm510
=====

An attempt to make C/C++ development on the CM-510 a bit easier.

Warning
-----
I am not sure this is the most up to date version. The CM-510 code uses defines from the dynamixel\_address\_tables.h, so is not immediately compatible with the CM-900 version.
Probable error in countdown timer (TIMER5), but made a change I hope will fix it.

Description
-----
This is mostly the embedded-c code provided by Robotis collected into a handful
of files and given easy to use wrapper functions.  Not as nice as arduino, but
it does take care of most of the low-level hardware stuff you would otherwise
have to work out on your own from the scattered code examples from Robotis.

Features
-----
- No need to deal with port or pin registers.
- HaViMo2 functions built; identical to those in CM-900/510/700
