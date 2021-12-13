# Replacement controller for my Softub

This code is intended to interface with the existing Softub control panel and temperature sensors, and behave similarly to the original Softub controller.
It should be able to directly replace the existing control board with an Arduino and a relay with the appropriate rating.

My initial build is using a [Leonardo clone](https://www.amazon.com/dp/B0786LJQ8K), and one of [these combined relay and 5v power supplies](https://www.amazon.com/dp/B077W1NVLM). Currently I'm planning to use a [prototyping shield](https://www.amazon.com/dp/B00Q9YB7PI) to distribute power and mount plugs for the connections.

I'm planning to mount the relay/supply unit in a [junction box](https://www.homedepot.com/p/Commercial-Electric-1-2-in-Gray-2-Gang-7-Holes-Non-Metallic-Weatherproof-Box-WDB750PG/300851103) with a [blank cover](https://www.homedepot.com/p/Commercial-Electric-Gray-2-Gang-Non-Metallic-Weatherproof-Blank-Cover-WBC200PG/300851669) and some [cable](https://www.homedepot.com/p/3-4-in-Strain-Relief-Cord-Connector-LPCG757-1/100171642) [seals](https://www.homedepot.com/p/Arlington-Industries-1-2-in-Low-Profile-Strain-Relief-Cord-Connector-LPCG507-1/308920052) so that all AC power is isolated from the arduino. I haven't yet worked out what sort of enclosure the Arduino will go in.

## Current state of the project:

I've reverse-engineered the protocol the original box uses to talk to the panel. (It's a pretty straightfoward binary protocol over TTL Serial at 2400 baud.)
The plug pinout for the panel is:

1. Yellow - Gnd
2. Green - TX (controller to panel)
3. Red - RX (panel to controller)
4. Black - ???
5. (no connection)
6. Blue - +5v

The temperature sensors seem to have a built-in resistor network. Supplying them with +5v and ground on the red and black wires, respectively, and reading the voltage on the green wire with an analog input gives a result that varies with temperature.

I've done a first pass of the logic that approximates the behavior of the original controller, with a couple of minor tweaks. 

## To-do:

- Create a calibration table to map the sensor readings to degrees Farenheit
- Finish up the prototyping shield
- Figure out an enclosure for the Arduino

