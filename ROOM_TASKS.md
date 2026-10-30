# Lecture Room 1 & 2 FreeRTOS Tasks

The two tasks match the GPIO connections in [`diagram.json`](diagram.json).

| Room | HC-SR04 trigger | HC-SR04 echo | LED output |
| --- | ---: | ---: | ---: |
| Lecture Room 1 | GPIO 5 | GPIO 18 | GPIO 14 |
| Lecture Room 2 | GPIO 4 | GPIO 19 | GPIO 27 |

Each FreeRTOS task repeatedly:

1. Sends a 10 microsecond trigger pulse.
2. Measures the echo pulse and converts it to centimeters.
3. Turns its room LED on when an object is at or below 150 cm.
4. Turns the LED off when the room is beyond 150 cm.

The LEDs start off. A sensor timeout leaves the current LED state unchanged,
so a temporary missing echo does not cause an unexpected light change.

The HC-SR04 echo pin is connected directly in the Wokwi diagram. For physical
hardware, use a level shifter or voltage divider because many HC-SR04 modules
produce a 5 V echo signal and ESP32 GPIO inputs are 3.3 V.
