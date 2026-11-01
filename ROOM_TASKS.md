# Lecture Room Lighting Logic

## 1. Purpose

The room-lighting subsystem automatically controls the lights in Lecture Rooms
1 and 2. It uses an HC-SR04 ultrasonic sensor in each room to detect
occupancy, an LDR (light-dependent resistor) to detect daylight, and one LED
output per room to represent the room light.

The implementation is contained in the `rooms_task()` FreeRTOS task in
`src/main.c`. One task services both rooms so that the same lighting rules are
applied consistently.

## 2. Sensor selection: ultrasonic versus PIR

HC-SR04 ultrasonic sensors were selected instead of PIR motion sensors because
the lighting system needs an indication of whether a person is currently
within the room's detection area, rather than only an indication that motion
occurred recently.

An ultrasonic sensor measures distance by transmitting an ultrasonic pulse and
measuring the time taken for the reflected echo to return. This provides a
direct distance measurement in centimeters. The program can therefore apply a
clear occupancy rule: a valid object closer than 150 cm is treated as possible
presence. The distance value is also useful for diagnostics because it can be
displayed and logged as `dist_cm`.

A PIR sensor detects changes in infrared radiation caused by movement from a
warm body. It normally reports motion rather than distance or continuous
presence. A person who is sitting still could eventually stop triggering a
PIR sensor, even though the room is still occupied. A PIR sensor also cannot
directly distinguish whether the detected person is close to the sensor or at
the far end of the room.

The main reasons for choosing ultrasonic sensors are:

1. **Distance-based detection:** The 150 cm threshold can be adjusted in
   software to suit the room layout and sensor position.
2. **Better support for stationary occupants:** The system can detect a person
   who remains within the measured area, instead of relying only on movement.
3. **Predictable filtering:** The firmware can reject invalid readings and use
   a two-out-of-three sample rule to reduce false activations.
4. **Useful feedback:** The measured distance can be shown in the `rooms`
   command and used when diagnosing sensor placement or room behavior.
5. **Simple simulation and integration:** HC-SR04 sensors are directly
   supported by the Wokwi circuit used for this project and require only a
   trigger output and an echo input for each room.

PIR sensors would still be a suitable alternative where low cost, low power,
and simple motion detection are more important than distance measurement.
However, a PIR-only implementation would need additional timing logic to keep
lights on after the last movement and would be more likely to turn the lights
off while a person is sitting still. The ultrasonic approach is therefore
better aligned with this project's occupancy-based lighting requirement.

## 3. Hardware connections

The pin assignments below are the current assignments in `src/main.c` and
match [`diagram.json`](diagram.json).

| Room | HC-SR04 trigger | HC-SR04 echo | LED output |
| --- | ---: | ---: | ---: |
| Lecture Room 1 | GPIO 32 | GPIO 34 | GPIO 14 |
| Lecture Room 2 | GPIO 33 | GPIO 35 | GPIO 27 |

The shared LDR is connected to the ESP32 ADC input on GPIO 36 (VP), represented
in the code by `ADC_CHANNEL_0`. The LDR reading is stored in the shared
variable `g_ldr_raw`.

The HC-SR04 echo pin is connected directly in the Wokwi diagram. For physical
hardware, a level shifter or voltage divider must be used because many HC-SR04
modules produce a 5 V echo signal while ESP32 GPIO inputs are designed for
3.3 V.

## 4. Configuration values

The main lighting thresholds are defined near the beginning of `src/main.c`:

| Setting | Value | Meaning |
| --- | ---: | --- |
| `ROOM_PRESENCE_CM` | 150 cm | Maximum distance treated as possible presence |
| `ROOM_OFF_DELAY_MS` | 30,000 ms | Time without presence before switching off |
| `ROOM_WARN_MS` | 5,000 ms | Warning period before automatic switch-off |
| `LDR_BRIGHT_RAW` | 2500 | ADC value above which daylight is detected |
| `ROOM_LIGHT_WATTS` | 36 W | Estimated power used for energy-saving statistics |

These constants make the system easy to calibrate for different room sizes,
sensors, or light fixtures.

## 5. Ultrasonic distance measurement

The `sonar_read_cm(room)` function selects the trigger and echo pins for the
requested room. It then performs the following sequence:

1. Sets the trigger pin LOW briefly to establish a known starting state.
2. Sends a HIGH trigger pulse for 10 microseconds.
3. Sets the trigger pin LOW again.
4. Waits for the echo pin to become HIGH.
5. Measures how long the echo remains HIGH.
6. Converts the echo duration to centimeters.

The conversion uses the HC-SR04 approximation:

```c
distance_cm = echo_time_microseconds / 58
```

The function has timeouts while waiting for the echo to start and finish. If
no valid echo is received, it returns `-1`. This prevents the task from waiting
indefinitely when a sensor is disconnected, obstructed, or malfunctioning.

Each room is measured approximately every 250 milliseconds. The main task
itself runs with a 100 millisecond delay, but a new sonar measurement is only
started when the room's 250 millisecond measurement interval has elapsed.

## 6. Presence filtering

A single ultrasonic reading is not immediately accepted as occupancy. For
each room, the task stores the last three presence results in a history buffer.
A reading is considered a possible presence when:

```c
distance > 0 && distance < ROOM_PRESENCE_CM
```

The room is marked as occupied only when at least two of the last three
readings indicate presence:

```c
present[i] =
    (hist[i][0] + hist[i][1] + hist[i][2]) >= 2;
```

This majority filter reduces false switching caused by an occasional invalid
echo, electrical noise, or a person briefly moving outside the sensor's
detection area. A sensor timeout returns `-1` and therefore does not count as
presence.

## 7. Daylight detection

The LDR is read by `io_task()` approximately every 500 milliseconds using the
ESP32 ADC. The most recent value is stored in `g_ldr_raw`.

The `ldr_bright()` function compares that value with `LDR_BRIGHT_RAW`:

```c
bool bright = g_ldr_raw > LDR_BRIGHT_RAW;
```

With the current configuration, an ADC value above 2500 means the room is
bright enough that artificial lighting is unnecessary. `LDR_INVERT` can be
changed if the installed sensor produces the opposite relationship between
ADC value and brightness.

Daylight is a shared measurement, so both rooms use the same brightness
decision. The ultrasonic sensors still determine occupancy independently for
each room.

## 8. Light activation logic

For each room, the task evaluates emergency mode, presence, and daylight in
that order.

### Normal operation

When presence is detected:

1. `last_seen[i]` is updated to the current time.
2. If the light is already on, it remains on.
3. If the light is off and the LDR reports darkness, the light is switched on.
4. If the LDR reports daylight, the light remains off and a
   `DAYLIGHT_SKIP` event is logged.

The light is therefore demand-controlled: occupancy is required, but the
system avoids using artificial light when daylight is available.

### Emergency operation

When emergency mode is active, the normal daylight restriction is overridden:

```c
if (emg) {
    last_seen[i] = now;
    light[i] = true;
}
```

Both room lights are forced on so that rooms remain illuminated during an
emergency or evacuation. Emergency mode is controlled elsewhere by the fire
button and the system event group.

## 9. Automatic switch-off and warning blink

When no presence is detected, the task does not switch the light off
immediately. It calculates the time since the last presence detection:

```c
now - last_seen[i]
```

If the light has been on continuously for 30 seconds without a new presence
detection, it is switched off. The automatic switch-off counter
`g_rooms[i].auto_offs` is incremented.

During the final five seconds of this delay, the physical LED output blinks
every 250 milliseconds. This provides a visible warning that the light is
about to switch off and gives a person an opportunity to move back into the
sensor range. The internal `light[i]` state remains ON during this warning
period; only the GPIO output is temporarily toggled.

The warning blink is disabled during emergency mode and while presence is
detected.

## 10. GPIO output

After calculating the desired state, the task writes the result to the
appropriate LED pin:

```c
gpio_set_level(led[i], out);
```

The LED array maps room index `0` to Room 1 and room index `1` to Room 2:

```c
const int led[2] = { PIN_LED_R1, PIN_LED_R2 };
```

The LEDs are configured as digital outputs during `gpio_setup()`. In the
Wokwi simulation they represent the room lights. In a physical installation,
the GPIO should control a suitable transistor, relay, or rated lighting
controller rather than drive a mains-powered lamp directly.

## 11. Shared room state and logging

The task maintains private working variables for distance, presence, light
state, sensor history, and timing. It periodically copies the useful values
into the shared `g_rooms` array:

- `dist_cm`: most recent measured distance.
- `presence`: whether the room is currently considered occupied.
- `light`: logical light state.
- `off_in_s`: seconds remaining before automatic switch-off, or `-1` when
  the countdown is inactive.
- `auto_offs`: number of automatic switch-offs.
- `saved_s`: accumulated seconds during which the light was off during open
  hours.

Whenever presence or light state changes, the task sends a room-state message
to `coord_task()`. The coordinator updates the shared state and writes a
`ROOM` event to the log. This allows the OLED display, serial commands, and
diagnostic logging to use the latest room information without directly
accessing the task's local variables.

## 12. Energy-saving calculation

While the system is within the configured opening schedule, the task adds
elapsed time to `saved_acc[i]` whenever a room light is off:

```c
if (!light[i] && open_hours)
    saved_acc[i] += dt;
```

The accumulated time is periodically transferred to `g_rooms[i].saved_s`.
The `rooms` command estimates saved energy using the configured 36 W light
power. The result is presented in watt-hours and indicates how much energy
was avoided by keeping the light off when the room was empty or daylight was
available.

## 13. Overall decision flow

For each room, the complete control cycle is:

1. Read the ultrasonic sensor when its measurement interval expires.
2. Add the result to the three-sample history.
3. Confirm presence using the two-out-of-three majority rule.
4. Read the latest daylight decision from the LDR.
5. If emergency mode is active, force the light on.
6. Otherwise, if presence exists and it is dark, turn the light on.
7. If there is no presence, keep the light on during the 30-second grace
   period.
8. Blink the light during the final five seconds of the grace period.
9. Turn the light off after the grace period expires.
10. Update shared room data, energy statistics, and event messages.

This design combines occupancy detection, daylight harvesting, delayed
switch-off, emergency override, and status reporting in one predictable
control loop.
