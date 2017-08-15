# Macchina SAM Boards for Arduino IDE

Note that this is a work-in-progress, so please report any issues.

## Installation

Under "Preferences", in the "additional Boards Manager URLs" field, paste:

> https://macchina.cc/package_macchina_index.json

and hit "OK"

Then select "Board Manager" under Tools -> Boards, and install the following:

 - Arduino SAM Boards (32-bit ARM Cortex-M3) by Arduino
 - Macchina SAM Boards by Macchina

You should now have a "Macchina SAM Boards" category under the Tools -> Boards menu in the Arduino IDE.

## Contributing

Contributions welcome!  For information on how to install this via `git clone` for easier hacking, please see [CONTRIBUTING.md](https://github.com/macchina/arduino-boards-sam/blob/master/CONTRIBUTING.md).

## Pin Names
See: [variant.h](https://github.com/macchina/arduino-boards-sam/blob/master/sam/variants/m2/variant.h) or [Pin Mapping](http://docs.macchina.cc/m2/processor/pin-mapping.html) in the Macchina Documentation.

## Acknowledgments
A big thank you to [@TDoust](https://github.com/TDoust) for initially putting together these files. The result is a much cleaner, easy to set up, and less confusing development experience.

## References
The [Arduino IDE 3rd-party Hardware Specification](https://github.com/arduino/Arduino/wiki/Arduino-IDE-1.5-3rd-party-Hardware-specification) provides the guidelines on how the content is structured and what can be included.

The [Arduino IDE package_index.json format specification](https://github.com/arduino/Arduino/wiki/Arduino-IDE-1.6.x-package_index.json-format-specification) contains the details for integrating with the Arduino Board Manager.  You may also be interested in [arduino-boards-index](https://github.com/macchina/arduino-boards-index) which generates our index file.

## Motivation
If you select "Arduino Due (Native USB Port)" from the Tools -> Board menu in the Arduino IDE, you'd need to utilize the [SamNonDuePin Library](https://github.com/macchina/SamNonDuePin.git) as in the following code:

```c
/*
  Demonstrates using "Non-Due" pins for Button inputs and LED outputs.
*/

#include "Arduino.h"
#include "SamNonDuePin.h"

// constants won't change. They're used here to
// set pin numbers:
const int SW1 = X1;                // Pushbutton SW1
const int SW2 = PIN_EMAC_ERX1;     // Pushbutton SW2

const int Yellow =  X0;      // the number of the LED pin
const int Red =  32;         // the number of the LED pin

// others are: 32(RED), X0(YELLOW), 27(YELLOW), 24(YELLOW), 23(GREEN), 12(RGB_GREEN), 5(RGB_BLUE), 11(RGB_RED)

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status
int buttonState2 = 0;        // variable for reading the pushbutton status

void setup() {

  pinModeNonDue(Yellow, OUTPUT);
  pinMode(Red, OUTPUT);

  pinModeNonDue(SW1, INPUT);
  pinModeNonDue(SW2, INPUT);

  digitalWriteNonDue(Yellow, LOW);
  digitalWrite(Red, LOW);
}

void loop() {

  buttonState = digitalReadNonDue(SW1);
  if (buttonState == HIGH) {  // NOT pressed
    digitalWriteNonDue(Yellow, HIGH);   // turn LED OFF:
  }
  else {
    digitalWriteNonDue(Yellow, LOW);    // turn LED ON:
  }

  buttonState2 = digitalReadNonDue(SW2);
  if (buttonState2 == HIGH) {
    digitalWrite(Red, HIGH);     // turn LED OFF:
  }
  else {
    digitalWrite(Red, LOW);     // turn LED ON:
  }

}
```

Using these board files you select "Macchina M2" from the Tools -> Board menu in the Arduino IDE and use the following code:

```c
/*
  Demonstrates using "Non-Due" pins for Button inputs and LED outputs using M2 board Defs.
*/

// constants won't change. They're used here to
// set pin numbers:
const int SW1 = Button1;      // Pushbutton SW1
const int SW2 = Button2;      // Pushbutton SW2

const int Yellow =  DS3;      // the number of the LED pin
const int Red =  DS2;         // the number of the LED pin

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status
int buttonState2 = 0;        // variable for reading the pushbutton status

void setup() {

  pinMode(Yellow, OUTPUT);
  pinMode(Red, OUTPUT);

  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);

  digitalWrite(Yellow, LOW);
  digitalWrite(Red, LOW);
}

void loop() {

  buttonState = digitalRead(SW1);
  if (buttonState == HIGH) {  // NOT pressed
    digitalWrite(Yellow, HIGH);   // turn LED OFF:
  }
  else {
    digitalWrite(Yellow, LOW);    // turn LED ON:
  }

  buttonState2 = digitalRead(SW2);
  if (buttonState2 == HIGH) {
    digitalWrite(Red, HIGH);     // turn LED OFF:
  }
  else {
    digitalWrite(Red, LOW);     // turn LED ON:
  }

}
```
