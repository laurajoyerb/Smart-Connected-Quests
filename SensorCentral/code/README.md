# Code Readme

The three most important files are sensor.c, sensors.js, and sensors.html.

The ESP32 code can be found in the sensor_central folder. In /sensor_central/main, the sensor.c file can be found. Build, flash, and monitor this file before running the node files. This ensures that the sensor data is being outputted to the console before node begins reading from the console.

The sensors.js file reads the sensor data from the console file (the data that was printed by sensor.c). It then sends that chart information (ie, data points) to the sensors.html file. 

The sensors.html file displays the chart with sensor data in a browser. 
