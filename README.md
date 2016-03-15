# Marlin 3D Printer Firmware
<img align="top" width=175 src="Documentation/Logo/Marlin%20Logo%20GitHub.png" />
 Documentation has moved to the [wiki](https://github.com/MarlinFirmware/MarlinDev/wiki).

## This is the source code which produces the Marlin Firmware
Complete instructions for setting up the build environment and configuring the firmware can be found at the [wiki](https://github.com/MarlinFirmware/MarlinDev/wiki).  In order to build the firmware, you will also need to setup the build environment. The official build environment is based on the [Arduino IDE 1.6.7](https://www.arduino.cc/Main/Software).

## Development Only

__Not for production use – use with caution!__

## In Transition

The files that make up the Marlin firmware are being reorganized to provide a more managable structure.

Since the 1.1.0-RC2, the following changes have been made:

1) File Layout --
   The bulk of the code is moved into the <sketchbook>/libraries/marlin/ folder.
   A trivial sketch that can be used to compile and upload the firmware is located at <sketchbook>/libraries/marlin/examples/MarlinFirmware/MarlinFirmware.ino

2) Configuration Files --
   The configuration files are being reorganized. The configuration files from the Marlin 1.1 series are still available in the configurations folder.
   The procedure that you use to customize your configuration has changed. Please see <sketchbook>/libraries/marlin/examples/MarlinFirmware/FirmwareCustomization.h for more information.

3) Compilation Support --
   Although they lack some of the build features provided, the code can also be compiled from the Arduino IDE 1.6.5 and by other methods.
   If you need them, you can retrieve earlier support routines from the [RC branch](https://github.com/MarlinFirmware/Marlin/tree/RC) of the [Marlin Release Repository](https://github.com/MarlinFirmware/Marlin).

## Branches
### Development
The [dev branch](https://github.com/MarlinFirmware/MarlinDev/tree/dev) contains the most up-to-date development Marlin code. Please remember that this code is for developers only and may well contain non-working situations. Instructions for setting up the build environment and compiling the firmware can be found in our [wiki](https://github.com/MarlinFirmware/MarlinDev/wiki).

### Testing
The [RC branch](https://github.com/MarlinFirmware/Marlin/tree/RC) contains the latest Marlin 1.1 pre-release candidate.
The [RCBugFix branch](https://github.com/MarlinFirmware/Marlin/tree/RCBugFix) has some additional corrections which will be included in the next release candidate.

### Released Code
For the latest tagged version of Marlin (currently 1.0.2 – January 2015) you should switch to the [Release Repository](https://github.com/MarlinFirmware/Marlin). Please see that repository for additional information.

## Current Status

Testing of the 1.1 Release Candidate is under way.

The support structure for the new development work has recently been put in place. It is likely that there will be some modifications made to it as we gain experience using it.

[![Coverity Scan Build Status](https://scan.coverity.com/projects/2224/badge.svg)](https://scan.coverity.com/projects/2224)
[![Travis Build Status](https://travis-ci.org/MarlinFirmware/MarlinDev.svg)](https://travis-ci.org/MarlinFirmware/MarlinDev)

##### [RepRap.org Wiki Page](http://reprap.org/wiki/Marlin)

## Contact

__Google Hangout:__ <a href="https://plus.google.com/hangouts/_/gxn3wrea5gdhoo223yimsiforia" target="_blank">. Hangout</a> We have a hangout every 2 weeks. Search the issue list for "Hangout" and you will see what time and date the next event is going on.

## Credits

The current Marlin dev team consists of:

 - Scott Lahteine [@thinkyhead] - English
 - [@Wurstnase] - Deutsch, English
 - F. Malpartida [@fmalpartida] - English, Spanish
 - [@CONSULitAS] - Deutsch, English
 - [@maverikou]
 - Chris Palmer [@nophead]
 - [@paclema]
 - Edward Patel [@epatel] - Swedish, English
 - Erik van der Zalm [@ErikZalm]
 - David Braam [@daid]
 - Bernhard Kubicek [@bkubicek]
 - Roxanne Neufeld [@Roxy-3DPrintBoard] - English

More features have been added by:
  - Alberto Cotronei [@MagoKimbra]
  - Lampmaker,
  - Bradley Feldman,
  - and others...

## License

Marlin is published under the [GPL v3 license](/LICENSE) because we believe in open development. The GPL comes with both rights and obligations. Whether you use Marlin firmware as the driver for your open or closed-source product, you must keep Marlin open, and you must provide your compatible Marlin source code to end users upon request. The most straightforward way to comply with the Marlin license is to make a fork of Marlin on Github, perform your modifications, and direct users to your modified fork.

While we can't prevent the use of this code in products (3D printers, CNC, etc.) that are closed source or crippled by a patent, we would prefer that you choose another firmware or, better yet, make your own.

[![Flattr this git repo](http://api.flattr.com/button/flattr-badge-large.png)](https://flattr.com/submit/auto?user_id=ErikZalm&url=https://github.com/MarlinFirmware/MarlinDev&title=Marlin&language=&tags=github&category=software)
