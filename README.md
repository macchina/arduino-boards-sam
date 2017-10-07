# Macchina SAM Boards for Arduino IDE

Note that this is a work-in-progress, so please report any issues.

## Installation

Under "Preferences", in the "additional Boards Manager URLs" field, paste:

    https://macchina.cc/package_macchina_index.json

and hit "OK"

Then select "Board Manager" under Tools -> Boards, and install the following:

 - Arduino SAM Boards (32-bit ARM Cortex-M3) by Arduino
 - Macchina SAM Boards by Macchina

You should now have a "Macchina SAM Boards" category under the Tools -> Boards menu in the Arduino IDE.

Manual installation instructions (not using the Board Manager) can be found in [CONTRIBUTING.md](https://github.com/macchina/arduino-boards-sam/blob/master/CONTRIBUTING.md).

## Contributing

Contributions welcome!  For information on how to install this via `git clone` for easier hacking, please see [CONTRIBUTING.md](https://github.com/macchina/arduino-boards-sam/blob/master/CONTRIBUTING.md).

## Pin Names
See: [variant.h](https://github.com/macchina/arduino-boards-sam/blob/master/sam/variants/m2/variant.h) or [Pin Mapping](http://docs.macchina.cc/m2/technical-references/pin-mapping.html) in the Macchina Documentation.

## Acknowledgments
A big thank you to [@TDoust](https://github.com/TDoust) for initially putting together these files. The result is a much cleaner, easy to set up, and less confusing development experience.

## References
The [Arduino IDE 3rd-party Hardware Specification](https://github.com/arduino/Arduino/wiki/Arduino-IDE-1.5-3rd-party-Hardware-specification) provides the guidelines on how the content is structured and what can be included.

The [Arduino IDE package_index.json format specification](https://github.com/arduino/Arduino/wiki/Arduino-IDE-1.6.x-package_index.json-format-specification) contains the details for integrating with the Arduino Board Manager.  You may also be interested in [arduino-boards-index](https://github.com/macchina/arduino-boards-index) which generates our index file.
