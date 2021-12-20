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

The temperature sensors seem to be a linear analog sensor, probably an LM34. Supply +5v/ground on the red and blue wires, and read a voltage proportional to temperature back on the green wire. The voltage read times 100 gives the temperature in Farenheit.

The control logic approximates the behavior of the original controller, with a couple of minor tweaks. Holding the temperature up/down buttons will adjust the temperature quickly. There's a "panic" mode if the two sensors go out of agreement or the temperature reading goes above 110 degrees farenheit, which shuts off the motor and flashes the two temperature readings on the display panel. The user can clear panic mode by holding the "light" and "jets" buttons together for 5 seconds (although if the panic condition is still present it will immediately reenter the panic state).

I've also added support for an ESP32-based board (the [WeMos D1 R32](https://www.amazon.com/gp/product/B07WFZCBH8), which appears to be the same hardware as the [DOIT ESPduino32](https://www.amazon.com/dp/B0775WFN9P)). I've included a board definition and pins file for this in the repository, as the [sp32doit-espduino board definition](https://docs.platformio.org/en/latest/boards/espressif32/esp32doit-espduino.html) built into PlatformIO has a couple of minor issues (mostly that the SPI library expects SCK/MISO/MOSI/SS to be defined, which breaks the build). 

On ESP32, the code can optionally fire up a web server that reports the current temperature, set temperature, and pump on/off status on one endpoint, and some debug information on another. I'm using this to pull the data onto another machine and build graphs with RRD.

I've also included an option for updating the sketch over the network, since pulling the board out of the pump pod to tweak settings is going to be annoying.

The relay/power supply unit I started with doesn't seem to have quite enough power for the ESP32 when WiFi is enabled -- the board is prone to brown out and reset (which doesn't happen when it's powered via USB). I've ordered a [different power supply](https://www.amazon.com/gp/product/B07V5XP92F) I hope will solve the problem, as well as a separate [single relay module](https://www.amazon.com/gp/product/B07TWH7DZ1)
