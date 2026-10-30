# Lecture Room 1 & 2 FreeRTOS Task Subsystem

This subsystem owns the automatic lighting control for both lecture rooms. Each
room has its own FreeRTOS task, so a sensor timeout in one room does not stop
the other room from being scanned.

## Hardware mapping

The GPIO assignments match [`main/diagram.json`](main/diagram.json):

| Room | HC-SR04 trigger | HC-SR04 echo | Relay input |
| --- | ---: | ---: | ---: |
| Lecture Room 1 | GPIO 5 | GPIO 18 | GPIO 14 |
| Lecture Room 2 | GPIO 4 | GPIO 19 | GPIO 27 |

## Task behavior

1. Each task sends a 10 microsecond trigger pulse to its HC-SR04.
2. The task measures the echo pulse and converts it to centimeters using
   `distance / 58`.
3. A distance of 150 cm or less is treated as room occupancy.
4. The room relay is switched on while occupied and off when the room is clear.
5. Each room is sampled every 250 ms.
6. If the echo signal times out, the task logs a warning and retains the
   previous relay state instead of changing the light unexpectedly.

The relay outputs are initialized low, so both lights start off. The task stack
size is 3072 bytes and both tasks run at FreeRTOS priority 5.

## Running in Wokwi

Open `main/diagram.json` in Wokwi and run the ESP-IDF project. Move an object
within the configured 150 cm threshold of either HC-SR04 to observe its relay
and corresponding room LED change. Serial output is tagged `room_tasks` and
reports task startup, state changes, and sensor timeouts.

For physical HC-SR04 hardware, protect the ESP32 echo input from the sensor's
5 V signal with an appropriate level shifter or voltage divider.
