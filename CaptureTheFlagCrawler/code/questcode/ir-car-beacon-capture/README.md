# IR Beacon

Infrared IR/UART beacons for crawler capture the flag!

Features:
- Sends UART payload -- | START | myColor | myID | Checksum? |
- Outputs 38kHz using RMT for IR transmission
- Onboard LED blinks corresponding to device ID value (myID)
- Button press to change device ID
- RGB LED shows traffic light state (red, green, yellow)
- Timer controls traffic light state (r - 10s, g - 10s, y - 2s)
- Runs at 1200 Baud

|Indicator Color|Color ID|
|---|---|
|Red|'R'|
|Yellow|'Y'|
|Green|'G'|

Resistors are fine-tuned for a special RGB LED (behavior will differ for if using individual LEDs)

### Wiring

|Function|Pin|
|---|---|
|RMT Pulse          |pin 26 -- A0|
|UART Transmitter   |pin 25 -- A1|
|UART Receiver      |pin 34 -- A2|
|Hardware interrupt |pin 4 -- A5|
|ID Indicator       |pin 13 -- Onboard LED|
|Red LED            |pin 33|
|Green LED          |pin 32|
|Blue LED           |Pin 14|
