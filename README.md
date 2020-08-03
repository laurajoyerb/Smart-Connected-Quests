# Smart-Connected-Quests

This repo is a collection of projects developed during ENG EC 444: Smart and Connected Systems taught by Professor Thomas Little and Dr. Emily Lam at Boston University. 

Each project uses IoT principles, ESP32s, and other skills showcased in the [Smart and Connected Skills](https://github.com/laurajoyerb/Smart-Connected-Skills) repo.

## Quests

### Retro Alarm Clock
This quest uses an alphanumeric display and two servos to display the time both digitally on the alphanumeric and in "analog" using the servos. It also has a console interface for the user to set an alarm for any time. 

### Sensor Central
This quest creates a multi-purpose sensor using a thermistor, an IR rangefinder, an ultrasonic sensor, and an ADC voltage reading pin on the ESP32. All of the readings are dynamically graphed to a web server.

### Wearable Sensor Watch
The "wearable" tracks biometrics and sends them back to a hub over WiFi. The biometrics include a step counter, heart rate, temperature, and battery level. It builds on the previous quests' usage of sensors. 

### Autonomous Driving Crawler
This quest is the first to use the crawlers. The crawlers are small vehicles controlled by an ESP32. It can successfully drive autonomously in a straight line and has collision avoidance to avoid hitting anything in front. It uses a variety or sensors to maintain a set distance from a straight wall as well as constant speed and stopping 10cm before any obstacles ahead of it. 

### Secure IR Beacon Keys
In this quest, we created portable, secure key fobs as well as beacons that served as our locks. We used NFC (near field communication) and IR for communication between the fobs and the hub beacons. The beacons logged attempts to enter and checked against database entries before either opening or locking the hub based on the key code given. 

### Capture the Flag Crawler
This is the final quest for the crawlers and the course. The crawlers have the ability to drive autonomously in straight lines and turn to make corners. They can also be controlled remotely from a web interface. The crawlers also obey "traffic lights" on the course simulated by IR beacons that switch between green, yellow, and red. The crawler uses a webcam to stream live video to the web interface as well as to decode a QR code at the end of the course and "capture the flag".
