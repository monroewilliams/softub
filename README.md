# Replacement controller for my Softub

This code is intended to interface with the existing Softub control panel and temperature sensors, and behave similarly to the original Softub controller.
It should be able to directly replace the existing control board with an Arduino and a relay with the appropriate rating.

Details about the hardware and connections are [here](./hardware/README.md).

## Current state of the project:

I've reverse-engineered the protocol the original box uses to talk to the panel. (It's a pretty straightfoward binary protocol over TTL Serial at 2400 baud.) For anyone interested in the reverse-engineering process I went through to figure out the protocol, I've documented it [here](./hardware/reverse-engineering.md).

The plug pinout for the panel is:

1. Yellow - Gnd
2. Green - TX (controller to panel)
3. Red - RX (panel to controller)
4. Black - ???
5. (no connection)
6. Blue - +5v

The temperature sensors seem to be a linear analog sensor, probably an LM34. Supply +5v/ground on the red and blue wires, and read a voltage proportional to temperature back on the green wire. The voltage read times 100 gives the temperature in Farenheit.

The control logic approximates the behavior of the original controller, with a couple of minor tweaks. Holding the temperature up/down buttons will adjust the temperature quickly. There's a "panic" mode if the two sensors go out of agreement or the temperature reading hits 5 degrees above the max allowed setpoint (currently 110 + 5 or 115 degrees), which shuts off the motor and flashes the two temperature readings on the display panel. The user can clear panic mode by holding the "light" and "jets" buttons together for 5 seconds (although if the panic condition is still present it will immediately reenter the panic state).

On ESP32, the code can optionally (dependent on `WEBSERVER` being defined) fire up a web server that has several endpoints:
- `/` -- Human-friendly status page with buttons for temperature up/down and turning the pump on/off manually (buttons are conditional on `WEBSERVER_REMOTE_CONTROL` being defined)
- `/set` -- Endpoint used to change the temp setting and pump status (conditional on `WEBSERVER_REMOTE_CONTROL` being defined)
- `/stats` -- Machine-friendly status page that reports current temp, set temp, whether the pump is running, and the last known valid water temp (I use this to build graphs with the scripts shown [here](./graphs/README.md))
- `/debug` -- Internal data useful for debugging the code (dependent on `WEBSERVER_DEBUG` being defined)

I've also included an option for using OTA update to update the sketch over the network, since pulling the board out of the pump pod to tweak settings is annoying. This support is conditional on `OTA_UPDATE` being defined.

The SSID/password for the WiFI connection and the OTA update authentication credentials can be set either inline in platformio.ini, or in a separate credentials.ini file (which is in .gitignore to prevent accidentally including them in git commits). See the `[credentials]` section in [platformio.ini](./platformio.ini) for more details on how this works.

## To-do list

It seems that the temperature reported by the sensors is higher than the actual water temperature (around 10 degrees higher when the water is 100 degrees). I've added a linear interpolation table in the sensor_temp_to_water_temp() function to account for this, but currently the table just offsets by 10 degrees at all temperatures. I expect that the offset will be different at different water temperatures, and I'll need to gather some data on this to calibrate the table at some point (probably the next time I drain and refill the tub, if I have the patience to watch the temperature while it heats...)

I'm considering adding a third LM34 temperature sensor to the shield so that I can also graph the temp at the controller. I'm fairly certain that the temp inside the pod is a lot higher than what the water sensors show (at least while the pump is running), but I'd like to know exactly how much higher. It's possible that this data could feed into the temperature offset, if I can figure out how they're related, and it would probably be prudent to have a thermal shutdown/panic case in the code if it goes too high.

